/*
 * Built-in IOMobileFramebuffer for SWEET: the ABL's continuous-splash
 * framebuffer (1080x2400 a8r8g8b8 in RAM, the panel scans it out by
 * itself) as the main display, driven by CoreAnimation's CPU renderer.
 *
 * IOMobileFramebuffer.framework (iOS 16.7) counts /arm-io children with
 * device_type "display-subsystem", waits for that many services matching
 * IOServiceMatching("IOMobileFramebuffer"), opens each with type 0 and
 * reads canvas-width/canvas-height from the service's provider. The class
 * here (AppleCLCD, an IOMobileFramebuffer) matches the /arm-io/disp0 nub.
 *
 * User client selectors used by CoreAnimation:
 *   3  GetLayerDefaultSurface [layer, ?] -> [IOSurface port name]
 *   4  SwapBegin -> [swap id]
 *   5  SwapEnd, struct 0x46c: +0x50 swap id, +0x54 IOSurface id per layer
 *      (4 layers), +0x64 rect a / +0xc4 rect b per layer (int x,y,w,h),
 *      +0x104 enabled layer mask
 *   6  SwapWait, 7 GetID, 8 GetDisplaySize -> [w, h],
 *   12 RequestPowerChange [state]: 0 = display off (backboardd, power
 *      button / auto-lock), else on. Off fades the panel out over a few
 *      vsyncs (halving each pixel) and keeps it black until on again;
 *      the panel itself stays powered (AMOLED black pixels are off).
 *   18 IsMainDisplay,
 *   28 GetDotPitch, 72 Enable/DisableNotifications [func, refcon, type, 0]
 * Everything else is accepted and logged once.
 *
 * SwapEnd copies the enabled layers' surfaces into the panel memory (no
 * scaling, CPU copy). Vsync (notification type 5) is a 60 Hz timer that
 * sends (count, mach_absolute_time, 0) as 32-bit halves to clients that
 * enabled it, through the port they registered (IOConnectSetNotificationPort).
 *
 * The panel memory itself is published as an IOSurface and returned as the
 * default framebuffer surface. The kernel text console keeps drawing over
 * the GUI by default: boot-arg sweetfb_noconsole=1 stops drawing it from
 * the first swap on (it still reaches the persistent log; panics draw).
 */
#include "SweetSurface.h"

extern "C" int sweet_console_muted;     /* osfmk/console/serial_console.c */
extern "C" void sweet_console_retarget(vm_offset_t va);   /* osfmk/console/video_console.c */
extern "C" void (*sweet_console_show_hook)(void);
extern "C" void topaz_modem_init(void);
extern "C" void topaz_audio_init(void);
extern "C" void sweet_amfi_service_init(void);
extern "C" void sweet_busy_init(void);              /* iokit/Kernel/arm/SweetBusy.cpp (boot-arg sweetbusy=N) */             /* iokit/Kernel/arm/TopazModem.cpp (boot-arg modem=1) */
extern "C" int sweet_console_copy_dirty(void *dst);
extern "C" void topaz_adreno_init(void);           /* iokit/Kernel/arm/TopazAdreno.cpp (boot-arg gpu=1) */
bool topaz_surface_drawn(SweetSurface *surface, uint64_t milliseconds);   /* RedwoodGPU.cpp */

/*
 * topaz: a surface the MDP may still be fetching counts as in use, so
 * CoreAnimation does not render into it. IOSurfaceClientIsInUse (iOS 16.7)
 * reads the u64 at +0x18 of the shared state page; held from SwapEnd until
 * one vsync after the next frame replaced it on the panel.
 */
static uint64_t sweet_fps_tick;         /* mach time of the last vsync tick (SWEET FPS) */

/* boot-arg sweetfb_nohold=1 (a test): frames on the panel are not marked in use */
static bool
sweet_fb_nohold(void)
{
	static int on = -1;
	if (on < 0) {
		uint32_t v = 0;
		on = PE_parse_boot_argn("sweetfb_nohold", &v, sizeof(v)) && v != 0;
	}
	return on;
}

static void
sweet_hold(SweetSurface *s, bool on)
{
	if (sweet_fb_nohold()) {
		return;
	}
	OSAddAtomic64(on ? 1 : -1, (volatile SInt64 *)((uint8_t *)s->state->getBytesNoCopy() + 0x18));
}

static void
sweet_hold_set(SweetSurface **set, bool on)
{
	for (uint32_t i = 0; i < 4; i++) {
		if (set[i] != NULL && !on) {
			sweet_hold(set[i], false);
			set[i]->release();
			set[i] = NULL;
		}
	}
}
class AppleCLCD;
static AppleCLCD *sweet_cons_fb;
static void sweet_cons_show(void);
class OSDictionary;
static void sweet_bl_log(const char *who, OSDictionary *d);
extern "C" void sweet_dcache_clean(vm_offset_t addr, unsigned length);  /* osfmk/arm/caches.c */
extern "C" vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

#if defined(SWEET)
#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <libkern/c++/OSCollectionIterator.h>
#include <kern/thread_call.h>
#include <IOKit/IOKitKeys.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/system.h>
#include <pexpert/pexpert.h>
#include <kern/clock.h>
#include <sys/proc.h>

enum {
	kSweetFBGetLayerDefaultSurface  = 3,
	kSweetFBSwapBegin               = 4,
	kSweetFBSwapEnd                 = 5,
	kSweetFBSwapWait                = 6,
	kSweetFBGetID                   = 7,
	kSweetFBGetDisplaySize          = 8,
	kSweetFBRequestPowerChange      = 12,
	kSweetFBIsMainDisplay           = 18,
	kSweetFBGetDotPitch             = 28,
	kSweetFBNotifications           = 72,
	kSweetFBGetBlock                = 79,
};

#define SWEET_FB_SWAP_REC_SIZE          0x46c
#define SWEET_FB_LAYERS                 4
#define SWEET_FB_FADE_STEPS             30      /* ~0.5 s at 60 Hz */
#define SWEET_FB_MAX_NITS               500     /* IOMFBBrightnessLevel at panel DBV 0x7ff */
#define SWEET_FB_NOTIFY_TYPES           8
#define SWEET_FB_NOTIFY_VSYNC           5
#define SWEET_FB_VSYNC_US               16667
#define SWEET_FB_DOT_PITCH              458     /* ppi, as the iPhone X panel */
/*
 * The iOS userland only has status bar / layout data for real iPhone
 * screens (UIKit: "No status bar visual provider for screen", the red
 * rdar://45025538 bar for 360x800 points). The display is therefore
 * presented as the iPhone X panel, 1125x2436 (375x812 points @3x), and the
 * centered 1080x2400 part of it is shown on the 1080x2400 panel.
 * Boot-arg sweetfb_native=1 presents the real panel size instead.
 */
#define SWEET_FB_LOGICAL_W              1125
#define SWEET_FB_LOGICAL_H              2436
#define SWEET_FB_FORMAT_BGRA            0x42475241

class IOMobileFramebuffer : public IOService {
	OSDeclareDefaultStructors(IOMobileFramebuffer);
};

OSDefineMetaClassAndStructors(IOMobileFramebuffer, IOService);

class IOMobileFramebufferUserClient;

class AppleCLCD : public IOMobileFramebuffer {
	OSDeclareDefaultStructors(AppleCLCD);

	IOWorkLoop         *fWorkLoop;
	IOTimerEventSource *fVsync;
	uint64_t            fVsyncCount;

	static void vsyncFired(OSObject *owner, IOTimerEventSource *sender);
	void publishModes(void);

public:
	uint64_t      fPhys;
	uint32_t      fWidth, fHeight, fRowBytes;
	uint32_t      fLogicalW, fLogicalH;     /* size reported to userland */
	int32_t       fOffX, fOffY;             /* logical origin of the panel */
	/* rain: sweetfb_fit=1 scales the logical canvas to the panel width (letterboxed) */
	bool          fFit;
	uint32_t      fFitH, fFitTop;           /* scaled canvas height, black bar above it */
	int32_t       fFitXs[1088];             /* per panel column: source x */
	uint16_t      fFitXf[1088];             /* per panel column: x weight (0..256) */
	uintptr_t     fPixels;          /* kernel mapping of the panel memory */
	IOMemoryMap  *fMap;
	SweetSurface *fSurface;         /* the panel memory as an IOSurface */
	IOLock       *fLock;            /* swaps and client list */
	OSArray      *fClients;
	uint32_t      fSwapID;
	bool          fConsolePaused;
	uint8_t       fPending[SWEET_FB_SWAP_REC_SIZE];     /* latest swap, not yet shown */
	bool          fDirty;
	/* topaz direct MDP scanout (VIG0 / CTL0 from TopazGpuW disp.c) */
	volatile uint32_t *fVig0;
	volatile uint32_t *fCtl0;
	volatile uint32_t *fIntf1;      /* INTF_1 (DSI0) timing engine: refresh diagnostics */
	uint32_t      fIntfFrames;
	/* boot-arg sweetfb_lock=1: the vsync timer follows the panel (vsyncNext) */
	bool          fVLock;
	uint32_t      fVTotal;        /* lines a panel frame, blanking included */
	uint32_t      fVLockFrames;   /* INTF frame counter and time at the last measurement */
	uint64_t      fVLockAt;
	uint32_t      fPanelUS;       /* a panel frame, measured */
	uint32_t      vsyncNext(void);
	uint64_t      fScanPhys;      /* what VIG0 currently fetches */
	uint32_t      fScanStride;
	uint32_t      fBootSrc, fBootStride;
	bool          directPresent(const uint8_t *rec);
	/*
	 * Direct frames go to a thread of ours (presenter): SwapEnd returns at once and backboardd is
	 * at its next frame while this one waits for the GPU and the MDP's latch (8-10 ms a frame,
	 * which held SwapEnd past CoreAnimation's next tick: 20 frames a second). Two may wait.
	 */
	IOLock       *fQLock;
	uint8_t       fQRec[2][SWEET_FB_SWAP_REC_SIZE];
	SweetSurface *fQHeld[2][4];
	unsigned      fQFirst, fQWaiting;
	bool          fQStarted;
	void          presenter(void);
	static void   presenterMain(void *self, wait_result_t result);
	void          showDirect(const uint8_t *rec, SweetSurface **held);
	void          showPending(const uint8_t *rec, SweetSurface **held);
	bool          directEligible(const uint8_t *rec, SweetSurface **held);
	void          scanFrom(uint64_t pa, uint32_t stride);
	uint32_t      fFlushTicks;    /* vsync ticks between panel copies */
	bool          fPowerOff;      /* RequestPowerChange(0) */
	uint32_t      fFadeStep;      /* fade-out steps done while off */
	/* topaz fade: cached copy of the last frame and a contiguous output frame */
	IOBufferMemoryDescriptor *fFadeOrig;
	/* kernel-drawn frames (console, fade) are double buffered like iOS's:
	 * built in the back buffer, then the MDP flips to it at vsync */
	IOBufferMemoryDescriptor *fFlip[2];
	uint64_t      fFlipPhys[2];
	uint32_t      fFlipIdx;
	bool          fConsShown;     /* console frames on the panel (no iOS frame yet) */
	void          flipFrom(const uint8_t *src, uint32_t srcStride);
	uint8_t      *backBuffer(void) { return (uint8_t *)fFlip[fFlipIdx ^ 1]->getBytesNoCopy(); }
	void          flipBack(void);
	IOBufferMemoryDescriptor *fCons;    /* console shadow frame (cached, contiguous) */
	uint64_t      fConsPhys;
	const uint8_t *fLastVA;       /* last directly scanned frame (panel window) */
	uint32_t      fLastStride;
	SweetSurface *fLastSurf;      /* keeps fLastVA alive */
	SweetSurface *fPendHold[4], *fShowHold[4], *fRetireHold[4];   /* in-use holds */
	void          fadeStep(const uint8_t *src, uint32_t srcStride, uint32_t step);

	bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	IOReturn setProperties(OSObject *properties) APPLE_KEXT_OVERRIDE;
	/* topaz panel brightness: DCS 0x51 over DSI0 (MakrSas/topaz-woa TopazGpuW disp.c) */
	volatile uint8_t *fDsi;
	IOBufferMemoryDescriptor *fDsiBuf;
	uint64_t      fDsiBufPhys;
	IOLock       *fBlLock;
	uint32_t      fBlLevel;       /* panel DBV 1..0x7ff, kept while the display is off */
	uint32_t      fBlSent;        /* last level sent to the panel */
	thread_call_t fBlCall;        /* sends the latest level off backboardd's thread */
	static void   blWork(thread_call_param_t p0, thread_call_param_t p1);
	bool          dsiDcsLong(const uint8_t *payload, uint32_t len);
	bool          dsiSend(const uint8_t hdr[4], const uint8_t *payload, uint32_t len);
	void          setPanelOn(bool on);
	void          setPanelLevel(uint32_t level);
	void addClient(IOMobileFramebufferUserClient *client);
	void removeClient(IOMobileFramebufferUserClient *client);
	void swap(const uint8_t *rec);
	void present(const uint8_t *rec);
};

/*
 * CoreBrightness (-[CBDisplayContaineriOS findBacklight]) matches
 * {IOPropertyMatch: {backlight-control: true}} and requires the class to
 * conform to AppleARMBacklight; without it there is no display module and
 * the brightness slider is dead. It then sets "brightness-nits" here
 * (SetBLDriverNitsValue) and IOMFBBrightnessLevel on the framebuffer.
 */
class AppleARMBacklight : public IOService {
	OSDeclareDefaultStructors(AppleARMBacklight);
public:
	AppleCLCD *fFB;
	IOReturn setProperties(OSObject *properties) APPLE_KEXT_OVERRIDE;
};

OSDefineMetaClassAndStructors(AppleARMBacklight, IOService);

/*
 * The graphics accelerator's service class. QuartzCore (CAMetalContextCreate,
 * once per process) polls IOServiceMatching("IOAcceleratorES") 300 times,
 * 100 ms apart, before it goes on: thirty seconds at every backboardd start.
 * One without MetalPluginName/MetalPluginClassName is found and then passed
 * over by Metal (MTLRegisterDevices), which leaves the renderers as they were.
 */
class IOAcceleratorES : public IOService {
	OSDeclareDefaultStructors(IOAcceleratorES);
};

OSDefineMetaClassAndStructors(IOAcceleratorES, IOService);
OSDefineMetaClassAndStructors(AppleCLCD, IOMobileFramebuffer);

class IOMobileFramebufferUserClient : public IOUserClient {
	OSDeclareDefaultStructors(IOMobileFramebufferUserClient);

public:
	AppleCLCD          *fFB;
	task_t              fTask;
	mach_port_t         fPort;
	io_user_reference_t fFunc[SWEET_FB_NOTIFY_TYPES];
	io_user_reference_t fRefcon[SWEET_FB_NOTIFY_TYPES];

	bool initWithTask(task_t owningTask, void *securityID, UInt32 type) APPLE_KEXT_OVERRIDE;
	bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	IOReturn clientClose(void) APPLE_KEXT_OVERRIDE;
	IOReturn registerNotificationPort(mach_port_t port, UInt32 type,
	    io_user_reference_t refCon) APPLE_KEXT_OVERRIDE;
	IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments *args,
	    IOExternalMethodDispatch *dispatch, OSObject *target,
	    void *reference) APPLE_KEXT_OVERRIDE;
	void notify(uint32_t type, const uint64_t *values, uint32_t count);
};

OSDefineMetaClassAndStructors(IOMobileFramebufferUserClient, IOUserClient);

bool
AppleCLCD::start(IOService *provider)
{
	PE_Video video;

	if (!IOService::start(provider)) {
		return false;
	}
	topaz_modem_init();
	topaz_audio_init();
	sweet_amfi_service_init();
	sweet_busy_init();
	PE_current_console(&video);
	fPhys = video.v_baseAddr;
	fWidth = (uint32_t)video.v_width;
	fHeight = (uint32_t)video.v_height;
	fRowBytes = (uint32_t)video.v_rowBytes;
	if (fPhys == 0 || fWidth == 0 || fHeight == 0 || fRowBytes < fWidth * 4) {
		IOLog("SWEET FB: no boot framebuffer\n");
		return false;
	}
	int native = 0;
	PE_parse_boot_argn("sweetfb_native", &native, sizeof(native));
	fLogicalW = fWidth;
	fLogicalH = fHeight;
	if (!native && fWidth <= SWEET_FB_LOGICAL_W && fHeight <= SWEET_FB_LOGICAL_H) {
		fLogicalW = SWEET_FB_LOGICAL_W;
		fLogicalH = SWEET_FB_LOGICAL_H;
	}
	/* x rounded down to 4 pixels: the panel window must start 16-byte aligned (MDP fetch) */
	fOffX = ((int32_t)(fLogicalW - fWidth) / 2) & ~3;
	fOffY = (int32_t)(fLogicalH - fHeight) / 2;
	{
		uint32_t fit = 0;
		PE_parse_boot_argn("sweetfb_fit", &fit, sizeof(fit));
		/* 1: keep the shape, bars above and below; 2: stretch to the panel's height (rain: +5.8 %) */
		fFitH = fit == 2 ? fHeight : (uint32_t)(((uint64_t)fLogicalH * fWidth) / fLogicalW);
		fFit = fit != 0 && fLogicalW > fWidth && fWidth <= 1088 && fFitH <= fHeight;
		fFitTop = fFit ? (fHeight - fFitH) / 2 : 0;
		if (fFit) {
			IOLog("RAIN FB: fit %ux%u -> %ux%u at y %u\n", fLogicalW, fLogicalH, fWidth, fFitH, fFitTop);
		}
	}
	fLock = IOLockAlloc();
	fClients = OSArray::withCapacity(4);
	fSurface = sweet_surface_create_physical(fPhys, fWidth, fHeight, fRowBytes, SWEET_FB_FORMAT_BGRA);
	fPixels = fSurface != NULL ? fSurface->kernelAddress() : 0;
	fWorkLoop = IOWorkLoop::workLoop();
	if (fLock == NULL || fClients == NULL || fPixels == 0 || fWorkLoop == NULL) {
		IOLog("SWEET FB: setup failed\n");
		OSSafeReleaseNULL(fSurface);    /* stays published: also held by the table */
		OSSafeReleaseNULL(fWorkLoop);
		return false;
	}
	fVsync = IOTimerEventSource::timerEventSource(this, &AppleCLCD::vsyncFired);
	if (fVsync == NULL || fWorkLoop->addEventSource(fVsync) != kIOReturnSuccess) {
		IOLog("SWEET FB: no vsync timer\n");
		return false;
	}
	fVig0 = (volatile uint32_t *)ml_io_map(0x05e05000, 0x1000);
	fCtl0 = (volatile uint32_t *)ml_io_map(0x05e02000, 0x1000);
	topaz_adreno_init();
	setProperty("RedwoodWaitsForGPU", true);         /* present() waits for the GPU (eagl.c reads it) */
	fDsi = (volatile uint8_t *)ml_io_map(0x05e94000, 0x1000);
	fDsiBuf = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
	    kIODirectionInOut | kIOMemoryPhysicallyContiguous, page_size, 0x00000000fffff000ull);
	fDsiBufPhys = fDsiBuf ? fDsiBuf->getPhysicalSegment(0, NULL, kIOMemoryMapperNone) : 0;
	fBlLock = IOLockAlloc();
	fBlLevel = 0x133;                       /* panel init level (stock DT) */
	fBlSent = ~0u;
	fBlCall = thread_call_allocate(&AppleCLCD::blWork, this);
	if (fDsi != NULL) {
		IOLog("SWEET FB: DSI0 ctrl %#x, brightness via DCS 0x51\n", *(volatile uint32_t *)(fDsi + 0x004));
		setProperty("IOMFBBrightnessLevel", (unsigned long long)(200u << 16), 32);
		/*
		 * Off unless sweetbl=1: with a backlight service CoreBrightness also
		 * runs digital dimming, a colour filter over every frame, which the
		 * CPU renderer cannot afford (the UI dropped to under 1 fps).
		 */
		uint32_t blOn = 0;
		(void)PE_parse_boot_argn("sweetbl", &blOn, sizeof(blOn));
		AppleARMBacklight *bl = blOn ? OSTypeAlloc(AppleARMBacklight) : NULL;
		if (bl != NULL && bl->init()) {
			bl->fFB = this;
			bl->setProperty("backlight-control", true);
			bl->setProperty("brightness-nits", (unsigned long long)200, 32);
			bl->setProperty("max-nits", (unsigned long long)SWEET_FB_MAX_NITS, 32);
			if (bl->attach(this)) {
				bl->registerService();
				IOLog("SWEET FB: AppleARMBacklight published\n");
			}
		}
		OSSafeReleaseNULL(bl);
	}
	uint32_t accelOn = 1;
	(void)PE_parse_boot_argn("sweetaccel", &accelOn, sizeof(accelOn));
	IOAcceleratorES *accel = accelOn ? OSTypeAlloc(IOAcceleratorES) : NULL;
	if (accel != NULL && accel->init()) {
		accel->setName("sgx");
		if (accel->attach(this)) {
			accel->registerService();
			IOLog("SWEET FB: empty IOAcceleratorES published (no 30 s wait in CAMetalContextCreate)\n");
		}
	}
	OSSafeReleaseNULL(accel);
	/* dpu sm6115 catalog: intf_1 at mdp+0x6a800, mdp at 0x5e01000 */
	fIntf1 = (volatile uint32_t *)ml_io_map(0x05e6b000, 0x1000);
	if (fIntf1 != NULL) {
		volatile uint32_t *r = fIntf1 + 0x800 / 4;
		r[0x0a8 / 4] = 0x3;     /* FRAME_LINE_COUNT_EN */
		uint32_t htotal = r[0x008 / 4] >> 16;
		fVTotal = htotal != 0 ? r[0x00c / 4] / htotal : 0;
		IOLog("SWEET INTF1: te_en %#x config %#x hsync %#x vsync_period %#x hctl %#x active_hctl %#x vstart %#x vend %#x fmt %#x cmd_trig %#x\n",
		    r[0x000 / 4], r[0x004 / 4], r[0x008 / 4], r[0x00c / 4], r[0x03c / 4], r[0x040 / 4],
		    r[0x01c / 4], r[0x024 / 4], r[0x090 / 4], r[0x084 / 4]);
	}
	if (fVig0 != NULL && fCtl0 != NULL) {
		fBootSrc = fVig0[0x14 / 4];
		fBootStride = fVig0[0x24 / 4];
		fScanPhys = fBootSrc;
		fScanStride = fBootStride;
		IOLog("SWEET FB: MDP VIG0 src %#x stride %u (boot), direct scanout ready\n",
		    fBootSrc, fBootStride);
		sweet_scanout_set_geometry(fWidth, fHeight, fLogicalW, fLogicalH);
		vm_size_t fsz = (vm_size_t)fRowBytes * fHeight;
		uint32_t noshadow = 0;      /* rain: boot-arg sweetfb_noshadow=1 skips flip/shadow buffers */
		PE_parse_boot_argn("sweetfb_noshadow", &noshadow, sizeof(noshadow));
		IOLog("RAIN FB: step1 geometry set, fsz %lu noshadow %u\n", (unsigned long)fsz, noshadow);
		for (int i = 0; i < 2; i++) {
			fFlip[i] = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
			    kIODirectionInOut | kIOMemoryPhysicallyContiguous, fsz, 0x00000000fffff000ull);
			fFlipPhys[i] = fFlip[i] ? fFlip[i]->getPhysicalSegment(0, NULL, kIOMemoryMapperNone) : 0;
		}
		IOLog("RAIN FB: step2 flip buffers %#llx %#llx\n", fFlipPhys[0], fFlipPhys[1]);
		fFadeOrig = noshadow ? NULL : IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, kIODirectionInOut, fsz, page_size);
		IOLog("RAIN FB: step3 fade buffer %p\n", fFadeOrig);
		fCons = noshadow ? NULL : IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
		    kIODirectionInOut | kIOMemoryPhysicallyContiguous, fsz, 0x00000000fffff000ull);
		if (fFlip[0] == NULL || fFlip[1] == NULL || (fFadeOrig == NULL && !noshadow)) {
			OSSafeReleaseNULL(fFlip[0]);
			OSSafeReleaseNULL(fFlip[1]);
			OSSafeReleaseNULL(fFadeOrig);
			OSSafeReleaseNULL(fCons);
		}
		if (fCons != NULL) {
			uint8_t *c = (uint8_t *)fCons->getBytesNoCopy();
			IOLog("RAIN FB: step4 shadow %p, copying boot frame\n", c);
			memcpy(c, (const void *)fPixels, fsz);  /* keep what the console drew */
			IOLog("RAIN FB: step5 copied\n");
			sweet_dcache_clean((vm_offset_t)(uintptr_t)c, (unsigned)fsz);   /* for panics */
			fConsPhys = fCons->getPhysicalSegment(0, NULL, kIOMemoryMapperNone);
			sweet_cons_fb = this;
			sweet_console_show_hook = sweet_cons_show;
			IOLog("RAIN FB: step6 retarget console\n");
			sweet_console_retarget((vm_offset_t)c);
			IOLog("RAIN FB: step7 retargeted, flipping\n");
			fConsShown = true;
			flipFrom(c, fRowBytes);
			IOLog("SWEET FB: console double buffered (shadow %#llx)\n", fConsPhys);
		} else {
			OSSafeReleaseNULL(fCons);
		}
	}
	uint32_t hz = 10;
	PE_parse_boot_argn("sweetfb_hz", &hz, sizeof(hz));
	fFlushTicks = (hz == 0 || hz >= 60) ? 1 : 60 / hz;
	uint32_t lock = 0;
	fVLock = PE_parse_boot_argn("sweetfb_lock", &lock, sizeof(lock)) && lock != 0 && fIntf1 != NULL && fVTotal > 100;
	fPanelUS = 8333;
	IOLog("SWEET FB: vsync %s the panel (%u lines a frame)\n", fVLock ? "follows" : "does not follow", fVTotal);
	fVsync->setTimeoutUS(SWEET_FB_VSYNC_US);
	uint32_t sync = 0;
	if (!(PE_parse_boot_argn("sweetfb_sync", &sync, sizeof(sync)) && sync != 0) && (fQLock = IOLockAlloc()) != NULL) {
		thread_t thread;
		if (kernel_thread_start(&AppleCLCD::presenterMain, this, &thread) == KERN_SUCCESS) {
			thread_deallocate(thread);
			IOLog("SWEET FB: direct frames through a presenter thread (SwapEnd returns at once)\n");
		} else {
			IOLockFree(fQLock);
			fQLock = NULL;
		}
	}

	publishModes();
	setProperty(kIOUserClientClassKey, "IOMobileFramebufferUserClient");
	setProperty("external", false);
	setProperty("DisplayPipePlaneBaseAlignment", 64ULL, 32);
	registerService();
	IOLog("SWEET FB: AppleCLCD %ux%u stride %u at 0x%llx, default surface %u, presented as %ux%u\n",
	    fWidth, fHeight, fRowBytes, fPhys, fSurface->id, fLogicalW, fLogicalH);
	return true;
}

void
AppleCLCD::vsyncFired(OSObject *owner, IOTimerEventSource *sender)
{
	AppleCLCD *fb = (AppleCLCD *)owner;
	uint64_t now = mach_absolute_time();
	uint64_t count = ++fb->fVsyncCount;
	sweet_fps_tick = now;
	/*
	 * What QuartzCore reads (darwin-redwood MiniIOMobileFramebuffer): three 64-bit numbers as six
	 * halves -- the time of this refresh, the time it was presented, a counter. The counter came
	 * first here: with CA_FORCE_TIMER its frame clock took the count for a time.
	 */
	uint64_t values[6] = {
		(uint32_t)now, (uint32_t)(now >> 32),
		(uint32_t)now, (uint32_t)(now >> 32),
		(uint32_t)count, (uint32_t)(count >> 32),
	};

	IOLockLock(fb->fLock);
	for (unsigned int i = 0; i < fb->fClients->getCount(); i++) {
		IOMobileFramebufferUserClient *client =
		    (IOMobileFramebufferUserClient *)fb->fClients->getObject(i);
		client->notify(SWEET_FB_NOTIFY_VSYNC, values, 6);
	}
	if (fb->fIntf1 != NULL && (count % 300) == 0) {
		uint32_t frames = fb->fIntf1[(0x800 + 0x0ac) / 4];
		IOLog("SWEET INTF1: %u frames in 5 s (line %u, te_en %#x)\n",
		    frames - fb->fIntfFrames, fb->fIntf1[(0x800 + 0x0b0) / 4], fb->fIntf1[0x800 / 4]);
		fb->fIntfFrames = frames;
	}
	if (fb->fPowerOff) {
		/*
		 * topaz: fade in cached memory and flip whole frames to the MDP.
		 * Fading the uncached boot framebuffer in place crawled down the panel.
		 */
		/*
		 * topaz: the DCS commands (0x51 0, 0x28, 0x10) do not reach the panel
		 * yet (video mode). Show one black frame of the kernel's own instead:
		 * black AMOLED pixels are dark. Not a fade from the last frame: with the
		 * GPU pool the last frame is a surface that may already be reused
		 * (garbage, then a hang).
		 */
		if (fb->fDsi != NULL && fb->fFlip[0] != NULL && fb->fFlip[1] != NULL) {
			bool first = fb->fFadeStep == 0;
			fb->fFadeStep = 1;
			IOLockUnlock(fb->fLock);
			if (first) {
				bzero(fb->backBuffer(), (size_t)fb->fRowBytes * fb->fHeight);
				fb->flipBack();
			}
			sender->setTimeoutUS(SWEET_FB_VSYNC_US);
			return;
		}
		if (fb->fFadeOrig != NULL) {
			uint32_t step = fb->fFadeStep;
			if (step <= SWEET_FB_FADE_STEPS) {
				fb->fFadeStep = step + 1;
			}
			bool front = fb->fFlip[0] != NULL && fb->fScanPhys == fb->fFlipPhys[fb->fFlipIdx];
			const uint8_t *src = front ? (const uint8_t *)fb->fFlip[fb->fFlipIdx]->getBytesNoCopy() :
			    fb->fLastVA ? fb->fLastVA :
			    fb->fConsShown ?
			    (const uint8_t *)fb->fCons->getBytesNoCopy() : (const uint8_t *)fb->fPixels;
			uint32_t srcStride = (!front && fb->fLastVA) ? fb->fLastStride : fb->fRowBytes;
			IOLockUnlock(fb->fLock);
			if (step <= SWEET_FB_FADE_STEPS) {
				fb->fadeStep(src, srcStride, step);
			}
			sender->setTimeoutUS(SWEET_FB_VSYNC_US);
			return;
		}
		if (fb->fVig0 != NULL && fb->fScanPhys != fb->fBootSrc) {
			fb->scanFrom(fb->fBootSrc, fb->fBootStride);
		}
		/* Fade out: halve every channel each vsync, then black, then idle. */
		uint32_t step = fb->fFadeStep;
		if (step <= 6) {
			fb->fFadeStep = step + 1;
		}
		IOLockUnlock(fb->fLock);
		if (step <= 6) {
			for (uint32_t y = 0; y < fb->fHeight; y++) {
				volatile uint32_t *row = (volatile uint32_t *)(fb->fPixels + (uint64_t)y * fb->fRowBytes);
				for (uint32_t x = 0; x < fb->fWidth; x++) {
					row[x] = step < 6 ? (row[x] >> 1) & 0x7f7f7f7f : 0;
				}
			}
		}
		sender->setTimeoutUS(SWEET_FB_VSYNC_US);
		return;
	}
	bool flush = fb->fDirty && (count % fb->fFlushTicks) == 0;
	uint8_t *rec = NULL;
	SweetSurface *taken[4] = {};
	if (flush) {
		rec = (uint8_t *)IOMallocData(SWEET_FB_SWAP_REC_SIZE);
		if (rec != NULL) {
			memcpy(rec, fb->fPending, SWEET_FB_SWAP_REC_SIZE);
			fb->fDirty = false;
			memcpy(taken, fb->fPendHold, sizeof(taken));
			bzero(fb->fPendHold, sizeof(fb->fPendHold));
		}
	}
	/* frames replaced at the previous vsync have been latched by now */
	SweetSurface *retire[4];
	memcpy(retire, fb->fRetireHold, sizeof(retire));
	bzero(fb->fRetireHold, sizeof(fb->fRetireHold));
	IOLockUnlock(fb->fLock);
	sweet_hold_set(retire, false);
	if (fb->fConsShown && sweet_console_copy_dirty(fb->backBuffer())) {
		fb->flipBack();
	}
	if (rec != NULL) {
		fb->present(rec);
		IOFreeData(rec, SWEET_FB_SWAP_REC_SIZE);
		IOLockLock(fb->fLock);
		memcpy(fb->fRetireHold, fb->fShowHold, sizeof(taken));
		memcpy(fb->fShowHold, taken, sizeof(taken));
		IOLockUnlock(fb->fLock);
	}
	sender->setTimeoutUS(fb->fVLock ? fb->vsyncNext() : SWEET_FB_VSYNC_US);
}

/*
 * boot-arg sweetfb_lock=1: when to send the next vsync. The panel runs at its own rate (about
 * 127 frames a second, INTF_1's counter) and the 16.7 ms timer drifted across it: a frame
 * CoreAnimation finished could wait up to a panel frame for the MDP to take it, and frames
 * went out unevenly. Every second panel frame instead, a little after it starts.
 */
uint32_t
AppleCLCD::vsyncNext(void)
{
	volatile uint32_t *r = fIntf1 + 0x800 / 4;
	uint64_t now = mach_absolute_time();
	uint32_t frames = r[0x0ac / 4];
	uint32_t line = r[0x0b0 / 4] & 0xffff;
	uint64_t ns;

	absolutetime_to_nanoseconds(now - fVLockAt, &ns);
	if (fVLockAt != 0 && frames - fVLockFrames >= 60 && ns > 0) {
		uint32_t us = (uint32_t)(ns / 1000 / (frames - fVLockFrames));
		if (us >= 5000 && us <= 20000) {
			fPanelUS = us;
		}
		fVLockFrames = frames;
		fVLockAt = now;
	} else if (fVLockAt == 0) {
		fVLockFrames = frames;
		fVLockAt = now;
	}
	if (line >= fVTotal) {
		line = 0;
	}
	uint32_t left = (uint32_t)((uint64_t)(fVTotal - line) * fPanelUS / fVTotal);
	uint32_t next = left + 300;
	while (next < 12000) {
		next += fPanelUS;
	}
	return next;
}

/*
 * The display's modes, the way CoreAnimation reads them (properties TimingElements and
 * ColorElements; the current one is block 0x40 of selector 79): one timing, the presented size
 * at 60 Hz, one colour mode, 8-bit RGB. Without them CoreAnimation ran its frame clock at a rate
 * of its own guessing: 20 frames a second, 50 ms apart. (After HackiOS MiniIOMobileFramebuffer
 * publishModes(), APSL 2.0.)
 */
#define SWEET_FB_TIMING_ID      1
#define SWEET_FB_COLOR_ID       1

static void
sweet_set_number(OSDictionary *d, const char *key, uint64_t value)
{
	OSNumber *n = OSNumber::withNumber(value, 32);
	if (n != NULL) {
		d->setObject(key, n);
		n->release();
	}
}

void
AppleCLCD::publishModes(void)
{
	OSDictionary *timing = OSDictionary::withCapacity(8);
	OSDictionary *horizontal = OSDictionary::withCapacity(4);
	OSDictionary *vertical = OSDictionary::withCapacity(4);
	OSDictionary *color = OSDictionary::withCapacity(8);
	OSArray *timings = OSArray::withCapacity(1);
	OSArray *colors = OSArray::withCapacity(1);
	const uint32_t rate = 60u << 16;        /* 16.16 */

	if (timing != NULL && horizontal != NULL && vertical != NULL && color != NULL && timings != NULL && colors != NULL) {
		sweet_set_number(color, "ID", SWEET_FB_COLOR_ID);
		sweet_set_number(color, "PixelEncoding", 0);
		sweet_set_number(color, "Depth", 8);
		sweet_set_number(color, "Colorimetry", 1);
		sweet_set_number(color, "EOTF", 1);
		sweet_set_number(color, "DynamicRange", 0);
		sweet_set_number(color, "Score", 1);
		color->setObject("IsVirtual", kOSBooleanFalse);
		colors->setObject(color);
		sweet_set_number(horizontal, "Active", fLogicalW);
		sweet_set_number(horizontal, "Total", fLogicalW);
		sweet_set_number(horizontal, "SyncRate", rate);
		sweet_set_number(horizontal, "PreciseSyncRate", rate);
		sweet_set_number(vertical, "Active", fLogicalH);
		sweet_set_number(vertical, "Total", fLogicalH);
		sweet_set_number(vertical, "SyncRate", rate);
		sweet_set_number(vertical, "PreciseSyncRate", rate);
		sweet_set_number(timing, "ID", SWEET_FB_TIMING_ID);
		sweet_set_number(timing, "Score", 1);
		timing->setObject("IsVirtual", kOSBooleanFalse);
		timing->setObject("IsOverscanned", kOSBooleanFalse);
		timing->setObject("HorizontalAttributes", horizontal);
		timing->setObject("VerticalAttributes", vertical);
		timing->setObject("ColorModes", colors);
		timings->setObject(timing);
		setProperty("TimingElements", timings);
		setProperty("ColorElements", colors);
		IOLog("SWEET FB: modes published: %ux%u at 60 Hz\n", fLogicalW, fLogicalH);
	}
	OSSafeReleaseNULL(timings);
	OSSafeReleaseNULL(colors);
	OSSafeReleaseNULL(timing);
	OSSafeReleaseNULL(horizontal);
	OSSafeReleaseNULL(vertical);
	OSSafeReleaseNULL(color);
}

void
AppleCLCD::addClient(IOMobileFramebufferUserClient *client)
{
	IOLockLock(fLock);
	fClients->setObject(client);
	IOLockUnlock(fLock);
}

void
AppleCLCD::removeClient(IOMobileFramebufferUserClient *client)
{
	IOLockLock(fLock);
	unsigned int i = fClients->getNextIndexOfObject(client, 0);
	if (i != (unsigned int)-1) {
		fClients->removeObject(i);
	}
	IOLockUnlock(fLock);
}

static uint32_t
get32(const uint8_t *p, uint32_t off)
{
	uint32_t v;
	memcpy(&v, p + off, 4);
	return v;
}

static uint32_t sweet_fb_swaps_logged;
static uint64_t sweet_fb_swap_count;

/* Copy each enabled layer's surface into the panel memory (no scaling). */
/*
 * A swap only records the frame: copying 10 MB into the panel on every
 * swap (backboardd presents at 60 Hz) took the whole CPU. The vsync timer
 * shows the latest frame every fFlushTicks ticks (boot-arg sweetfb_hz,
 * default 10 Hz).
 */
/*
 * Frames reaching swap(), a line a second while they come: how many, the longest gap between
 * two, and how many came more than 20 ms after the one before (a vsync missed).
 */
static uint64_t sweet_fps_wait_ns, sweet_fps_after_ns;  /* this second: latch waits, tick-to-swap */

static void
sweet_fps_count(void)
{
	static int on = -1;                     /* boot-arg sweetfps=1: the SWEET FPS line */
	if (on < 0) {
		uint32_t v = 0;
		on = PE_parse_boot_argn("sweetfps", &v, sizeof(v)) && v != 0;
	}
	if (!on) {
		return;
	}
	static uint64_t last, window, gap_max;
	static uint32_t frames, late;
	uint64_t now = mach_absolute_time(), ns = 0, gap = 0;

	if (last != 0) {
		absolutetime_to_nanoseconds(now - last, &gap);
	}
	last = now;
	if (window == 0 || gap > 500000000ull) {
		window = now;
		frames = late = 0;
		gap_max = 0;
		return;
	}
	frames++;
	late += gap > 20000000ull;
	if (sweet_fps_tick != 0 && now > sweet_fps_tick) {
		uint64_t after = 0;
		absolutetime_to_nanoseconds(now - sweet_fps_tick, &after);
		sweet_fps_after_ns += after;
	}
	gap_max = gap > gap_max ? gap : gap_max;
	absolutetime_to_nanoseconds(now - window, &ns);
	if (ns >= 1000000000ull) {
		IOLog("SWEET FPS: %u frames in %llu ms, longest gap %llu ms, %u late; a frame: %llu us in present + latch, "
		    "came %llu us after the tick\n", frames, ns / 1000000, gap_max / 1000000, late,
		    frames ? sweet_fps_wait_ns / 1000 / frames : 0, frames ? sweet_fps_after_ns / 1000 / frames : 0);
		sweet_fps_wait_ns = sweet_fps_after_ns = 0;
		window = now;
		frames = late = 0;
		gap_max = 0;
	}
}

void
AppleCLCD::swap(const uint8_t *rec)
{
	sweet_fps_count();
	SweetSurface *held[4] = {};
	uint32_t mask = get32(rec, 0x104);
	for (uint32_t i = 0; i < 4 && i < SWEET_FB_LAYERS; i++) {
		uint32_t sid = get32(rec, 0x54 + 4 * i);
		if ((mask & (1u << i)) && sid != 0 && (held[i] = sweet_surface_copy(sid)) != NULL) {
			if (!sweet_fb_nohold()) {
				sweet_hold(held[i], true);
			}
		}
	}
	/*
	 * topaz GPU: a frame that is one GPU-pool surface (the window server's, drawn by the GPU, scanned
	 * by the MDP itself) is shown at once rather than at the next tick of our timer, and the frame
	 * it replaces is let go as soon as the MDP has latched the new address (at the panel's next
	 * vsync, 8 ms at most). Held two ticks, the two surfaces CoreAnimation draws into paced it
	 * to 20 frames a second.
	 */
	if (directEligible(rec, held) && fQLock != NULL) {
		AbsoluteTime deadline;
		clock_interval_to_deadline(200, kMillisecondScale, &deadline);
		IOLockLock(fQLock);
		while (fQWaiting >= 2) {
			if (IOLockSleepDeadline(fQLock, &fQWaiting, deadline, THREAD_UNINT) == THREAD_TIMED_OUT && fQWaiting >= 2) {
				IOLockUnlock(fQLock);
				sweet_hold_set(held, false);    /* (the presenter is stuck: this frame is lost) */
				return;
			}
		}
		unsigned slot = (fQFirst + fQWaiting) % 2;
		memcpy(fQRec[slot], rec, SWEET_FB_SWAP_REC_SIZE);
		memcpy(fQHeld[slot], held, sizeof(fQHeld[slot]));
		fQWaiting++;
		IOLockWakeup(fQLock, fQRec, true);
		IOLockUnlock(fQLock);
		return;
	}
	if (directEligible(rec, held)) {
		showDirect(rec, held);
		return;
	}
	showPending(rec, held);
}

bool
AppleCLCD::directEligible(const uint8_t *rec, SweetSurface **held)
{
	uint32_t mask = get32(rec, 0x104);
	int layer = -1;
	for (uint32_t i = 0; i < 4 && i < SWEET_FB_LAYERS; i++) {
		if (mask & (1u << i)) {
			layer = layer < 0 ? (int)i : -2;
		}
	}
	return layer >= 0 && held[layer] != NULL && held[layer]->gpuPool && !fPowerOff && !fConsShown;
}

void
AppleCLCD::presenterMain(void *self, wait_result_t result __unused)
{
	((AppleCLCD *)self)->presenter();
}

void
AppleCLCD::presenter(void)
{
	static uint8_t rec[SWEET_FB_SWAP_REC_SIZE];     /* (one thread) */
	SweetSurface *held[4];

	for (;;) {
		IOLockLock(fQLock);
		while (fQWaiting == 0) {
			IOLockSleep(fQLock, fQRec, THREAD_UNINT);
		}
		memcpy(rec, fQRec[fQFirst], sizeof(rec));
		memcpy(held, fQHeld[fQFirst], sizeof(held));
		IOLockUnlock(fQLock);
		if (directEligible(rec, held)) {
			showDirect(rec, held);
		} else {
			showPending(rec, held);
		}
		IOLockLock(fQLock);
		fQFirst = (fQFirst + 1) % 2;
		fQWaiting--;
		IOLockWakeup(fQLock, &fQWaiting, false);
		IOLockUnlock(fQLock);
	}
}

void
AppleCLCD::showDirect(const uint8_t *rec, SweetSurface **heldIn)
{
	SweetSurface *held[4];
	memcpy(held, heldIn, sizeof(held));
	{
		SweetSurface *dropped[4], *shown[4], *retired[4];
		IOLockLock(fLock);
		memcpy(dropped, fPendHold, sizeof(dropped));
		bzero(fPendHold, sizeof(fPendHold));
		fDirty = false;                                 /* (anything older not shown at all) */
		IOLockUnlock(fLock);
		sweet_hold_set(dropped, false);
		uint64_t waited = mach_absolute_time(), wns = 0;  /* (present waits for the GPU too) */
		present(rec);
		for (int t = 0; t < 250 && (fCtl0[0x18 / 4] & 0x20001) != 0; t++) {
			IODelay(100);                           /* until the MDP has latched it */
		}
		absolutetime_to_nanoseconds(mach_absolute_time() - waited, &wns);
		sweet_fps_wait_ns += wns;
		IOLockLock(fLock);
		memcpy(shown, fShowHold, sizeof(shown));
		memcpy(retired, fRetireHold, sizeof(retired));
		memcpy(fShowHold, held, sizeof(held));
		bzero(fRetireHold, sizeof(fRetireHold));
		IOLockUnlock(fLock);
		sweet_hold_set(shown, false);
		sweet_hold_set(retired, false);
		if ((++sweet_fb_swap_count % 1000) == 0) {
			IOLog("SWEET FB: %llu swaps (direct)\n", sweet_fb_swap_count);
		}
	}
}

void
AppleCLCD::showPending(const uint8_t *rec, SweetSurface **heldIn)
{
	SweetSurface *held[4];
	memcpy(held, heldIn, sizeof(held));
	IOLockLock(fLock);
	memcpy(fPending, rec, SWEET_FB_SWAP_REC_SIZE);
	fDirty = true;
	SweetSurface *dropped[4];
	memcpy(dropped, fPendHold, sizeof(dropped));      /* replaced before shown */
	memcpy(fPendHold, held, sizeof(held));
	IOLockUnlock(fLock);
	sweet_hold_set(dropped, false);
	if ((++sweet_fb_swap_count % 1000) == 0) {
		IOLog("SWEET FB: %llu swaps\n", sweet_fb_swap_count);
	}
}

void
AppleCLCD::scanFrom(uint64_t pa, uint32_t stride)
{
	fVig0[0x14 / 4] = (uint32_t)pa;
	fVig0[0x24 / 4] = stride;
	__asm__ volatile("dsb sy" ::: "memory");
	fCtl0[0x18 / 4] = 0x20001;      /* flush VIG0 + CTL: latched at the next vsync */
	__asm__ volatile("dsb sy" ::: "memory");
	fScanPhys = pa;
	fScanStride = stride;
}

/*
 * DCS long write through the DSI command DMA path (topaz-woa: the TPG FIFO
 * path is a no-op on this DSI 2.4.1). The packet lives in a contiguous
 * buffer below 4 GiB; header = word count, data type 0x39, flags last|long.
 */
static inline uint32_t dsi_rd(volatile uint8_t *b, uint32_t o) { return *(volatile uint32_t *)(b + o); }
static inline void dsi_wr(volatile uint8_t *b, uint32_t o, uint32_t v)
{
	__asm__ volatile ("dsb sy" ::: "memory");
	*(volatile uint32_t *)(b + o) = v;
	__asm__ volatile ("dsb sy" ::: "memory");
}

bool
AppleCLCD::dsiDcsLong(const uint8_t *payload, uint32_t len)
{
	if (fDsi == NULL || fDsiBuf == NULL || len > 60) {
		return false;
	}
	uint8_t hdr[4] = { (uint8_t)len, (uint8_t)(len >> 8), 0x39, 0xc0 };   /* long write, last|long */
	return dsiSend(hdr, payload, len);
}

bool
AppleCLCD::dsiSend(const uint8_t hdr[4], const uint8_t *payload, uint32_t len)
{
	if (fDsi == NULL || fDsiBuf == NULL || len > 60) {
		return false;
	}
	uint8_t *p = (uint8_t *)fDsiBuf->getBytesNoCopy();
	uint32_t size = (4 + len + 3) & ~3u;
	memset(p, 0xff, 64);
	memcpy(p, hdr, 4);
	if (len) {
		memcpy(p + 4, payload, len);
	}
	sweet_dcache_clean((vm_offset_t)(uintptr_t)p, 64);
	uint32_t ctrl = dsi_rd(fDsi, 0x004), trig = dsi_rd(fDsi, 0x084);
	if ((trig & 7) != 4) {
		dsi_wr(fDsi, 0x084, (trig & ~7u) | 4);  /* DMA trigger: software */
	}
	dsi_wr(fDsi, 0x004, ctrl | 4 | 1);              /* CMD_MODE_EN | DSI_EN */
	dsi_wr(fDsi, 0x110, dsi_rd(fDsi, 0x110) | 1);   /* clear CMD_DMA_DONE */
	dsi_wr(fDsi, 0x15c, 0);                          /* no test pattern */
	dsi_wr(fDsi, 0x048, (uint32_t)fDsiBufPhys);
	dsi_wr(fDsi, 0x04c, size);
	uint32_t dma = dsi_rd(fDsi, 0x03c);
	/*
	 * Low-power mode (COMMAND_MODE_DMA_CTRL bit 26), as Linux sends commands
	 * for panels with MIPI_DSI_MODE_LPM: in high speed the topaz panel took
	 * none of them (no brightness, no display off). sweetdsi_hs=1: old way.
	 */
	static int hs = -1;
	if (hs < 0) {
		hs = 0;
		(void)PE_parse_boot_argn("sweetdsi_hs", &hs, sizeof(hs));
	}
	dsi_wr(fDsi, 0x03c, hs ? (dma & ~(1u << 26)) : (dma | (1u << 26)));
	dsi_wr(fDsi, 0x090, 1);                          /* SW trigger */
	bool done = false;
	for (int i = 0; i < 100 && !done; i++) {
		done = (dsi_rd(fDsi, 0x110) & 1) != 0;
		if (!done) {
			IODelay(200);
		}
	}
	if (!done) {
		static uint32_t told;
		if (told++ < 6) {
			IOLog("SWEET FB: DSI cmd %#x not done on cpu %d: ctrl %#x trig %#x dma %#x int %#x status %#x err %#x buf %#llx\n",
			    hdr[0], cpu_number(), dsi_rd(fDsi, 0x004), dsi_rd(fDsi, 0x084), dsi_rd(fDsi, 0x03c),
			    dsi_rd(fDsi, 0x110), dsi_rd(fDsi, 0x008), dsi_rd(fDsi, 0x0c4), (unsigned long long)fDsiBufPhys);
		}
	}
	dsi_wr(fDsi, 0x110, dsi_rd(fDsi, 0x110) | 1);
	dsi_wr(fDsi, 0x03c, dma);
	dsi_wr(fDsi, 0x004, ctrl);
	if (trig != dsi_rd(fDsi, 0x084)) {
		dsi_wr(fDsi, 0x084, trig);
	}
	return done;
}

/*
 * Panel power as the Linux panel drivers do it: off = brightness 0, DCS 0x28
 * display off, 0x10 sleep in (0x28 alone left the topaz AMOLED showing the
 * last frame); on = 0x11 sleep out, 120 ms, 0x29 display on (the caller
 * then sends the brightness).
 */
extern "C" void rain_touch_panel(int on);

void
AppleCLCD::setPanelOn(bool on)
{
	uint8_t first[4] = { (uint8_t)(on ? 0x11 : 0x28), 0, 0x05, 0x80 };
	uint8_t second[4] = { (uint8_t)(on ? 0x29 : 0x10), 0, 0x05, 0x80 };
	bool done = true;
	if (!on) {
		setPanelLevel(0);
	}
	IOLockLock(fBlLock);
	done &= dsiSend(first, NULL, 0);
	IOLockUnlock(fBlLock);
	IOSleep(on ? 120 : 20);
	IOLockLock(fBlLock);
	done &= dsiSend(second, NULL, 0);
	IOLockUnlock(fBlLock);
	if (!on) {
		IOSleep(120);
	}
	IOLog("SWEET FB: panel %s: %s\n", on ? "sleep out, display on" : "display off, sleep in",
	    done ? "done" : "NO DONE");
	rain_touch_panel(on);        /* rain: the panel IC is the touch controller too */
}

void
AppleCLCD::setPanelLevel(uint32_t level)
{
	uint8_t cmd[3] = { 0x51, (uint8_t)(level >> 8), (uint8_t)level };   /* MSB first */
	IOLockLock(fBlLock);
	bool done = dsiDcsLong(cmd, sizeof(cmd));
	IOLockUnlock(fBlLock);
	static uint32_t logs;
	if (logs++ < 8 || !done) {
		IOLog("SWEET FB: panel level %#x: %s\n", level, done ? "done" : "NO DONE");
	}
}

/*
 * CoreBrightness (SetIOMFBBrightness) sets IOMFBBrightnessLevel on the
 * framebuffer: nits in 16.16. It also animates the dim before the display
 * turns off, so the panel dims while iOS keeps drawing, as on an iPhone.
 */
IOReturn
AppleCLCD::setProperties(OSObject *properties)
{
	OSDictionary *d = OSDynamicCast(OSDictionary, properties);
	sweet_bl_log("iomfb", d);
	OSNumber *n = d ? OSDynamicCast(OSNumber, d->getObject("IOMFBBrightnessLevel")) : NULL;
	if (n == NULL || fDsi == NULL) {
		return kIOReturnUnsupported;
	}
	uint64_t v = n->unsigned64BitValue();
	setProperty("IOMFBBrightnessLevel", v, 32);
	/* 0..SWEET_FB_MAX_NITS nits -> DBV 1..0x7ff */
	uint64_t level = (v * 0x7ffull) / ((uint64_t)SWEET_FB_MAX_NITS << 16);
	if (level > 0x7ff) {
		level = 0x7ff;
	}
	if (level < 1) {
		level = 1;
	}
	IOLockLock(fLock);
	fBlLevel = (uint32_t)level;
	IOLockUnlock(fLock);
	/*
	 * CoreBrightness runs inside backboardd and ramps the level in many small
	 * synchronous steps; waiting for the DSI here stalled rendering. Only note
	 * the level; a thread call sends the latest one.
	 */
	if (fBlCall != NULL) {
		thread_call_enter(fBlCall);
	}
	return kIOReturnSuccess;
}

void
AppleCLCD::blWork(thread_call_param_t p0, __unused thread_call_param_t p1)
{
	AppleCLCD *fb = (AppleCLCD *)p0;
	IOLockLock(fb->fLock);
	uint32_t level = fb->fBlLevel;
	bool off = fb->fPowerOff;
	IOLockUnlock(fb->fLock);
	if (!off && level != fb->fBlSent) {
		fb->fBlSent = level;
		fb->setPanelLevel(level);
	}
}

static void
sweet_bl_log(const char *who, OSDictionary *d)
{
	static uint32_t logs;
	if (d == NULL || logs >= 24) {
		return;
	}
	OSCollectionIterator *it = OSCollectionIterator::withCollection(d);
	OSSymbol *k;
	while (it != NULL && (k = OSDynamicCast(OSSymbol, it->getNextObject())) != NULL) {
		OSObject *v = d->getObject(k);
		OSNumber *n = OSDynamicCast(OSNumber, v);
		if (logs++ < 24) {
			IOLog("SWEET BL: %s set %s = %s %llu\n", who, k->getCStringNoCopy(),
			    v ? v->getMetaClass()->getClassName() : "null", n ? n->unsigned64BitValue() : 0);
		}
	}
	OSSafeReleaseNULL(it);
}

IOReturn
AppleARMBacklight::setProperties(OSObject *properties)
{
	OSDictionary *d = OSDynamicCast(OSDictionary, properties);
	sweet_bl_log("backlight", d);
	OSNumber *n = d ? OSDynamicCast(OSNumber, d->getObject("brightness-nits")) : NULL;
	if (n != NULL && fFB != NULL) {
		OSDictionary *m = OSDictionary::withCapacity(1);
		/* same 16.16 nits scale as IOMFBBrightnessLevel */
		uint64_t v = n->unsigned64BitValue();
		if (v < 0x10000) {
			v <<= 16;               /* plain nits */
		}
		OSNumber *nn = OSNumber::withNumber(v, 64);
		m->setObject("IOMFBBrightnessLevel", nn);
		fFB->setProperties(m);
		nn->release();
		m->release();
		setProperty("brightness-nits", n);
	}
	return kIOReturnSuccess;
}

/* panic: show the console frame (MMIO only, no locks) */
static void
sweet_cons_show(void)
{
	AppleCLCD *fb = sweet_cons_fb;
	if (fb != NULL && fb->fVig0 != NULL && fb->fScanPhys != fb->fConsPhys) {
		fb->scanFrom(fb->fConsPhys, fb->fRowBytes);
	}
}

void
AppleCLCD::fadeStep(const uint8_t *src, uint32_t srcStride, uint32_t step)
{
	uint8_t *orig = (uint8_t *)fFadeOrig->getBytesNoCopy();
	uint8_t *out = backBuffer();
	if (step == 0) {
		/* snapshot what is on screen (one slow read of the frame) */
		for (uint32_t y = 0; y < fHeight; y++) {
			memcpy(orig + (uint64_t)y * fRowBytes, src + (uint64_t)y * srcStride, (size_t)fWidth * 4);
		}
	}
	/* linear fade: each channel * m / 32, m from 32 down to 0 */
	uint32_t m = 32 - (step * 32 + SWEET_FB_FADE_STEPS / 2) / SWEET_FB_FADE_STEPS;
	for (uint32_t y = 0; y < fHeight; y++) {
		const uint32_t *in = (const uint32_t *)(orig + (uint64_t)y * fRowBytes);
		uint32_t *o = (uint32_t *)(out + (uint64_t)y * fRowBytes);
		for (uint32_t x = 0; x < fWidth; x++) {
			uint32_t p = in[x];
			uint32_t lo = (((p & 0x00ff00ffu) * m) >> 5) & 0x00ff00ffu;
			uint32_t hi = ((((p >> 8) & 0x00ff00ffu) * m) >> 5) & 0x00ff00ffu;
			o[x] = lo | (hi << 8);
		}
	}
	flipBack();
}

/* Clean the back buffer and point the MDP at it (latched at its vsync). */
void
AppleCLCD::flipBack(void)
{
	uint32_t idx = fFlipIdx ^ 1;
	sweet_dcache_clean((vm_offset_t)(uintptr_t)fFlip[idx]->getBytesNoCopy(), (unsigned)fFlip[idx]->getLength());
	scanFrom(fFlipPhys[idx], fRowBytes);
	fFlipIdx = idx;
}

void
AppleCLCD::flipFrom(const uint8_t *src, uint32_t srcStride)
{
	uint8_t *dst = backBuffer();
	if (srcStride == fRowBytes) {
		memcpy(dst, src, (size_t)fRowBytes * fHeight);
	} else {
		for (uint32_t y = 0; y < fHeight; y++) {
			memcpy(dst + (uint64_t)y * fRowBytes, src + (uint64_t)y * srcStride, (size_t)fWidth * 4);
		}
	}
	flipBack();
}

/*
 * topaz: a swap that shows one full-screen surface from the scanout pool is
 * presented by pointing the MDP at it (no 10 MB CPU copy). Returns false to
 * fall back to the copy path.
 */
/* RAIN-FIT-BEGIN */
/*
 * rain: scale a rectangle of the logical canvas (lx,ly,lw,lh; its pixels start
 * at src + sy*stride + sx*4) into the panel at the canvas-to-panel width ratio,
 * bilinear, BGRA. Panel rows are offset by top. xs/xf: scratch of panelW entries.
 */
static inline uint32_t
sweet_fit_lerp(uint32_t a, uint32_t b, uint32_t f)
{
	uint32_t rb = ((a & 0x00ff00ffu) * (256u - f) + (b & 0x00ff00ffu) * f) >> 8;
	uint32_t ag = (((a >> 8) & 0x00ff00ffu) * (256u - f) + ((b >> 8) & 0x00ff00ffu) * f) >> 8;
	return (rb & 0x00ff00ffu) | ((ag & 0x00ff00ffu) << 8);
}

#define SWEET_FIT_ROW_MAX 2048         /* source row pixels the staging rows hold */

static void
sweet_fit_blit(uint8_t *dst, uint32_t dstStride, uint32_t panelW, uint32_t fitH, uint32_t top,
    uint32_t logicalW, uint32_t logicalH, const uint8_t *src, uint32_t stride, int32_t sx, int32_t sy,
    int32_t lx, int32_t ly, int32_t lw, int32_t lh, int32_t *xs, uint16_t *xf)
{
	if (lw <= 0 || lh <= 0 || panelW == 0 || logicalW == 0 || fitH == 0 || logicalH == 0) {
		return;
	}
	/* panel pixels whose centres fall inside the logical rectangle */
	int64_t px0 = ((int64_t)lx * panelW + logicalW - 1) / logicalW;
	int64_t px1 = ((int64_t)(lx + lw) * panelW + logicalW - 1) / logicalW;
	/* rows scale by fitH / logicalH: the same ratio as the columns, or taller when stretched */
	int64_t py0 = ((int64_t)ly * fitH + logicalH - 1) / logicalH;
	int64_t py1 = ((int64_t)(ly + lh) * fitH + logicalH - 1) / logicalH;
	if (px0 < 0) {
		px0 = 0;
	}
	if (py0 < 0) {
		py0 = 0;
	}
	if (px1 > (int64_t)panelW) {
		px1 = panelW;
	}
	if (py1 > (int64_t)fitH) {
		py1 = fitH;
	}
	/* logical coordinate of a panel pixel centre, in 1/256 px: (2p+1)*L*128/P - 128 */
	for (int64_t px = px0; px < px1; px++) {
		int64_t u = ((2 * px + 1) * (int64_t)logicalW * 128) / panelW - 128 - (int64_t)lx * 256;
		if (u < 0) {
			u = 0;
		}
		if (u > (int64_t)(lw - 1) * 256) {
			u = (int64_t)(lw - 1) * 256;
		}
		xs[px] = (int32_t)(u >> 8);
		xf[px] = (uint16_t)(xs[px] + 1 < lw ? (u & 255) : 0);
	}
	/*
	 * The source is usually a GPU surface in write-combined (uncached) memory, where every
	 * 4-byte read is its own bus transaction: about 4.5 M of them a frame made the GPU
	 * slower than the CPU. Each source row is copied once, with memcpy's wide loads, into
	 * a cached row (two in flight: y0 and y0 + 1 never share a slot), and scaled from there.
	 */
	static uint32_t rows[2][SWEET_FIT_ROW_MAX];     /* shortcut: one presenter at a time (frames are queued); per-thread rows if that changes */
	int32_t row_tag[2] = { -1, -1 };
	bool stage = lw <= SWEET_FIT_ROW_MAX;
	for (int64_t py = py0; py < py1; py++) {
		int64_t v = ((2 * py + 1) * (int64_t)logicalH * 128) / fitH - 128 - (int64_t)ly * 256;
		if (v < 0) {
			v = 0;
		}
		if (v > (int64_t)(lh - 1) * 256) {
			v = (int64_t)(lh - 1) * 256;
		}
		int32_t y0 = (int32_t)(v >> 8);
		uint32_t fy = y0 + 1 < lh ? (uint32_t)(v & 255) : 0;
		const uint32_t *r0 = (const uint32_t *)(src + (uint64_t)(sy + y0) * stride) + sx;
		const uint32_t *r1 = fy ? (const uint32_t *)((const uint8_t *)r0 + stride) : r0;
		if (stage) {
			for (int32_t k = 0; k < (fy ? 2 : 1); k++) {
				int32_t y = y0 + k, slot = y & 1;
				if (row_tag[slot] != y) {
					memcpy(rows[slot], (const uint8_t *)r0 + (uint64_t)k * stride, (size_t)lw * 4);
					row_tag[slot] = y;
				}
			}
			r0 = rows[y0 & 1];
			r1 = fy ? rows[(y0 + 1) & 1] : r0;
		}
		uint32_t *out = (uint32_t *)(dst + (uint64_t)(top + py) * dstStride);
		for (int64_t px = px0; px < px1; px++) {
			int32_t x = xs[px];
			uint32_t fx = xf[px];
			int32_t x1 = fx ? x + 1 : x;
			uint32_t t = sweet_fit_lerp(r0[x], r0[x1], fx);
			uint32_t b = fy ? sweet_fit_lerp(r1[x], r1[x1], fx) : t;
			out[px] = fy ? sweet_fit_lerp(t, b, fy) : t;
		}
	}
}
/* RAIN-FIT-END */

bool
AppleCLCD::directPresent(const uint8_t *rec)
{
	if (fVig0 == NULL) {
		return false;
	}
	uint32_t mask = get32(rec, 0x104), layer = 32, n = 0;
	for (uint32_t i = 0; i < SWEET_FB_LAYERS; i++) {
		if ((mask & (1u << i)) && get32(rec, 0x54 + 4 * i) != 0) {
			layer = i;
			n++;
		}
	}
	if (n != 1) {
		return false;
	}
	SweetSurface *s = sweet_surface_copy(get32(rec, 0x54 + 4 * layer));
	if (s == NULL) {
		return false;
	}
	bool ok = false;
	(void)topaz_surface_drawn(s, 100);
	if (fFit && fFlip[0] != NULL && s->bytesPerElement == 4 && s->elementWidth == 1 &&
	    s->width == fLogicalW && s->height == fLogicalH && s->memory != NULL &&
	    (uint64_t)s->offset + (uint64_t)s->height * s->bytesPerRow <= s->memory->getLength()) {
		uintptr_t va = s->kernelAddress();
		if (va != 0) {
			uint8_t *dst = backBuffer();
			bzero(dst, (size_t)fFitTop * fRowBytes);
			bzero(dst + (uint64_t)(fFitTop + fFitH) * fRowBytes, (size_t)(fHeight - fFitTop - fFitH) * fRowBytes);
			sweet_fit_blit(dst, fRowBytes, fWidth, fFitH, fFitTop, fLogicalW, fLogicalH,
			    (const uint8_t *)(va + s->offset), s->bytesPerRow, 0, 0, 0, 0,
			    (int32_t)fLogicalW, (int32_t)fLogicalH, fFitXs, fFitXf);
			flipBack();
			IOLockLock(fLock);
			fLastVA = NULL;         /* panel-sized copies (fade) read the flip buffer */
			IOLockUnlock(fLock);
			ok = true;
		}
		s->release();
		return ok;
	}
	if (s->scanoutPhys != 0 && s->bytesPerElement == 4 &&
	    s->width >= fWidth && s->height >= fHeight) {
		/* full-screen surface: panel window at the logical offset */
		uint32_t ox = (s->width - fWidth) / 2, oy = (s->height - fHeight) / 2;
		if (s->width == fLogicalW && s->height == fLogicalH) {
			ox = (uint32_t)fOffX;
			oy = (uint32_t)fOffY;
		}
		uint64_t pa = s->scanoutPhys + s->offset + (uint64_t)oy * s->bytesPerRow + (uint64_t)ox * 4;
		if ((pa & 0xf) == 0 && pa + (uint64_t)fHeight * s->bytesPerRow < (1ull << 32)) {
			uintptr_t va = s->kernelAddress();
			if (va != 0) {
				/*
				 * Copy into our back buffer and flip: CoreAnimation may start the
				 * next frame in this surface at once, so scanning it directly let
				 * the panel show frames half drawn. Cached to cached, a few ms.
				 */
				if (s->gpuPool) {
					/* GPU pool memory (write-combined): the MDP scans the surface itself, no copy */
					scanFrom(pa, s->bytesPerRow);
				} else if (fFlip[0] != NULL) {
					flipFrom((const uint8_t *)(va + s->offset + (uint64_t)oy * s->bytesPerRow + (uint64_t)ox * 4),
					    s->bytesPerRow);
				} else {
					sweet_dcache_clean((vm_offset_t)(va + s->offset + (uint64_t)oy * s->bytesPerRow), (unsigned)((uint64_t)fHeight * s->bytesPerRow));
					scanFrom(pa, s->bytesPerRow);
				}
				ok = true;
				IOLockLock(fLock);
				if (fLastSurf != s) {
					s->retain();
					if (fLastSurf != NULL) {
						fLastSurf->release();
					}
					fLastSurf = s;
				}
				fLastVA = (const uint8_t *)(va + s->offset + (uint64_t)oy * s->bytesPerRow + (uint64_t)ox * 4);
				fLastStride = s->bytesPerRow;
				IOLockUnlock(fLock);
			}
		}
	}
	s->release();
	return ok;
}

void
AppleCLCD::present(const uint8_t *rec)
{
	uint32_t mask = get32(rec, 0x104);
	static int console_only = -1;

	/* boot-arg sweetfb_console=1: debugging, the panel keeps showing the kernel console */
	if (console_only < 0) {
		console_only = 0;
		PE_parse_boot_argn("sweetfb_console", &console_only, sizeof(console_only));
	}
	if (console_only > 0 && fConsShown) {
		(void)mask;
		return;
	}
	fConsShown = false;
	if (directPresent(rec)) {
		if (!fConsolePaused) {
			int quiet = 0;
			fConsolePaused = true;
			if (PE_parse_boot_argn("sweetfb_noconsole", &quiet, sizeof(quiet)) && quiet != 0) {
				sweet_console_muted = 1;
			}
		}
		return;
	}
	IOLockLock(fLock);
	fLastVA = NULL;
	IOLockUnlock(fLock);
	/*
	 * copy path (several layers): compose into the cached back buffer and
	 * flip, like the direct path. Composing into the scanned, uncached boot
	 * framebuffer showed every multi-layer frame (dims, apps) drawn top-down.
	 */
	uint8_t *dst = fFlip[0] != NULL ? backBuffer() : (uint8_t *)fPixels;
	bool fit = fFit && fFlip[0] != NULL;
	if (fit) {
		bzero(dst, (size_t)fFitTop * fRowBytes);
		bzero(dst + (uint64_t)(fFitTop + fFitH) * fRowBytes, (size_t)(fHeight - fFitTop - fFitH) * fRowBytes);
	}
	if (fFlip[0] == NULL && fVig0 != NULL && fScanPhys != fBootSrc) {
		scanFrom(fBootSrc, fBootStride);
	}


	if (!fConsolePaused) {
		int quiet = 0;
		fConsolePaused = true;
		if (PE_parse_boot_argn("sweetfb_noconsole", &quiet, sizeof(quiet)) && quiet != 0) {
			/* Not PE_initialize_console(kPEDisableScreen): its console switch
			 * writes the read-only-late cons_ops_index and panics. */
			sweet_console_muted = 1;
		}
	}

	for (uint32_t layer = 0; layer < SWEET_FB_LAYERS; layer++) {
		uint32_t id = get32(rec, 0x54 + 4 * layer);
		const uint8_t *a = rec + 0x64 + 16 * layer;
		const uint8_t *b = rec + 0xc4 + 16 * layer;
		int32_t sx = (int32_t)get32(a, 0), sy = (int32_t)get32(a, 4);
		int32_t dx = (int32_t)get32(b, 0), dy = (int32_t)get32(b, 4);
		int32_t w = (int32_t)get32(b, 8), h = (int32_t)get32(b, 12);

		if (!(mask & (1u << layer)) || id == 0) {
			continue;
		}
		SweetSurface *s = sweet_surface_copy(id);
		if (sweet_fb_swaps_logged < 16) {
			sweet_fb_swaps_logged++;
			IOLog("SWEET FB: swap %u layer %u surface %u %ux%u fmt 0x%x bpr %u a %d,%d %ux%u b %d,%d %dx%d\n",
			    get32(rec, 0x50), layer, id, s ? s->width : 0, s ? s->height : 0,
			    s ? s->pixelFormat : 0, s ? s->bytesPerRow : 0, sx, sy, get32(a, 8),
			    get32(a, 12), dx, dy, w, h);
		}
		if (s == NULL || s == fSurface) {
			OSSafeReleaseNULL(s);
			continue;
		}
		(void)topaz_surface_drawn(s, 100);
		uintptr_t src = s->kernelAddress();
		uint32_t bpe = s->bytesPerElement ? s->bytesPerElement : 4;
		if (w <= 0 || h <= 0) {
			w = (int32_t)s->width;
			h = (int32_t)s->height;
		}
		if (sx < 0 || sy < 0) {
			sx = sy = 0;
		}
		if (dx < 0 || dy < 0) {
			dx = dy = 0;
		}
		if (fit) {
			if (w > (int32_t)s->width - sx) {
				w = (int32_t)s->width - sx;
			}
			if (h > (int32_t)s->height - sy) {
				h = (int32_t)s->height - sy;
			}
			uint64_t end = (uint64_t)s->offset + (uint64_t)(sy + h) * s->bytesPerRow;
			if (src != 0 && bpe == 4 && w > 0 && h > 0 && s->elementWidth == 1 && s->elementHeight == 1 &&
			    s->memory != NULL && end <= s->memory->getLength()) {
				sweet_fit_blit(dst, fRowBytes, fWidth, fFitH, fFitTop, fLogicalW, fLogicalH,
				    (const uint8_t *)(src + s->offset), s->bytesPerRow, sx, sy, dx, dy, w, h, fFitXs, fFitXf);
			}
			s->release();
			continue;
		}
		/* Logical (presented) display coordinates -> panel coordinates. */
		dx -= fOffX;
		dy -= fOffY;
		if (dx < 0) {
			sx -= dx;
			w += dx;
			dx = 0;
		}
		if (dy < 0) {
			sy -= dy;
			h += dy;
			dy = 0;
		}
		if (w > (int32_t)s->width - sx) {
			w = (int32_t)s->width - sx;
		}
		if (h > (int32_t)s->height - sy) {
			h = (int32_t)s->height - sy;
		}
		if (w > (int32_t)fWidth - dx) {
			w = (int32_t)fWidth - dx;
		}
		if (h > (int32_t)fHeight - dy) {
			h = (int32_t)fHeight - dy;
		}
		uint64_t last = (uint64_t)s->offset + (uint64_t)(sy + h - 1) * s->bytesPerRow +
		    (uint64_t)(sx + w) * 4;
		if (w <= 0 || h <= 0 || s->elementWidth != 1 || s->elementHeight != 1 ||
		    s->memory == NULL || last > s->memory->getLength()) {
			src = 0;
		}
		if (src != 0 && bpe == 4) {
			for (int32_t y = 0; y < h; y++) {
				memcpy((void *)(dst + (uint64_t)(dy + y) * fRowBytes + (uint64_t)dx * 4),
				    (const void *)(src + s->offset + (uint64_t)(sy + y) * s->bytesPerRow + (uint64_t)sx * 4),
				    (size_t)w * 4);
			}
		}
		s->release();
	}
	if (fFlip[0] != NULL) {
		flipBack();
	}
}

bool
IOMobileFramebufferUserClient::initWithTask(task_t owningTask, void *securityID, UInt32 type)
{
	if (!IOUserClient::initWithTask(owningTask, securityID, type)) {
		return false;
	}
	fTask = owningTask;
	return true;
}

bool
IOMobileFramebufferUserClient::start(IOService *provider)
{
	fFB = OSDynamicCast(AppleCLCD, provider);
	if (fFB == NULL || !IOUserClient::start(provider)) {
		return false;
	}
	fFB->addClient(this);
	return true;
}

IOReturn
IOMobileFramebufferUserClient::clientClose(void)
{
	if (fFB != NULL) {
		fFB->removeClient(this);
		IOLockLock(fFB->fLock);
		mach_port_t old = fPort;
		fPort = MACH_PORT_NULL;
		IOLockUnlock(fFB->fLock);
		if (old != MACH_PORT_NULL) {
			releaseNotificationPort(old);
		}
	}
	terminate();
	return kIOReturnSuccess;
}

IOReturn
IOMobileFramebufferUserClient::registerNotificationPort(mach_port_t port, __unused UInt32 type,
    __unused io_user_reference_t refCon)
{
	mach_port_t old;
	IOLockLock(fFB->fLock);
	old = fPort;
	fPort = port;
	IOLockUnlock(fFB->fLock);
	if (old != MACH_PORT_NULL) {
		releaseNotificationPort(old);
	}
	return kIOReturnSuccess;
}

/* Called with the framebuffer lock held. */
void
IOMobileFramebufferUserClient::notify(uint32_t type, const uint64_t *values, uint32_t count)
{
	OSAsyncReference64 ref;

	if (type >= SWEET_FB_NOTIFY_TYPES || fFunc[type] == 0 || fPort == MACH_PORT_NULL) {
		return;
	}
	bzero(ref, sizeof(ref));
	setAsyncReference64(ref, fPort, (mach_vm_address_t)fFunc[type], fRefcon[type], fTask);
	sendAsyncResult64WithOptions(ref, kIOReturnSuccess, (io_user_reference_t *)values, count,
	    kIOUserNotifyOptionCanDrop);
}

static uint64_t sweet_fb_logged;        /* selectors already logged (< 64) */

IOReturn
IOMobileFramebufferUserClient::externalMethod(uint32_t selector, IOExternalMethodArguments *args,
    __unused IOExternalMethodDispatch *dispatch, __unused OSObject *target,
    __unused void *reference)
{
	IOReturn ret = kIOReturnSuccess;
	uint64_t out[2] = { 0, 0 };
	uint32_t outCount = 0;

	switch (selector) {
	case kSweetFBGetLayerDefaultSurface:
		out[0] = sweet_surface_port_in_task(fFB->fSurface, fTask);
		outCount = 1;
		break;
	case kSweetFBSwapBegin:
		IOLockLock(fFB->fLock);
		out[0] = ++fFB->fSwapID;
		IOLockUnlock(fFB->fLock);
		outCount = 1;
		break;
	case kSweetFBSwapEnd: {
		uint8_t rec[SWEET_FB_SWAP_REC_SIZE];
		if (args->structureInputDescriptor != NULL ||
		    args->structureInputSize < SWEET_FB_SWAP_REC_SIZE) {
			ret = kIOReturnBadArgument;
			break;
		}
		memcpy(rec, args->structureInput, sizeof(rec));
		fFB->swap(rec);
		break;
	}
	case kSweetFBGetID:
		out[0] = 0;
		outCount = 1;
		break;
	case kSweetFBGetDisplaySize:
		out[0] = fFB->fLogicalW;
		out[1] = fFB->fLogicalH;
		outCount = 2;
		break;
	case kSweetFBIsMainDisplay:
		out[0] = 1;
		outCount = 1;
		break;
	case kSweetFBGetDotPitch:
		out[0] = SWEET_FB_DOT_PITCH;
		outCount = 1;
		break;
	case kSweetFBGetBlock:
		/*
		 * Block 0x40, the current mode: its colour and timing identifiers (publishModes). Zeros
		 * here left CoreAnimation without a mode, and its frame clock without the rate.
		 */
		if (args->structureOutput != NULL && args->structureOutputDescriptor == NULL) {
			bzero(args->structureOutput, args->structureOutputSize);
			if (args->scalarInputCount >= 1 && args->scalarInput[0] == 0x40 && args->structureOutputSize >= 12) {
				uint32_t ids[2] = { SWEET_FB_COLOR_ID, SWEET_FB_TIMING_ID };
				memcpy((uint8_t *)args->structureOutput + 4, ids, sizeof(ids));
			}
			return kIOReturnSuccess;
		}
		break;
	case kSweetFBNotifications:
		if (args->scalarInputCount >= 3 && args->scalarInput[2] < SWEET_FB_NOTIFY_TYPES) {
			uint32_t type = (uint32_t)args->scalarInput[2];
			IOLockLock(fFB->fLock);
			fFunc[type] = args->scalarInput[0];
			fRefcon[type] = args->scalarInput[1];
			IOLockUnlock(fFB->fLock);
			char name[32];
			proc_selfname(name, sizeof(name));
			IOLog("SWEET FB: %s notifications type %u %s\n", name, type,
			    args->scalarInput[0] ? "on" : "off");
		}
		break;
	case kSweetFBRequestPowerChange:
		if (args->scalarInputCount >= 1) {
			bool off = args->scalarInput[0] == 0;
			/*
			 * topaz: sweetfb_stayon=1 ignores display-off requests (auto-lock)
			 * while there are no buttons or touch to wake the panel again.
			 */
			int stayon = 0;
			if (off && PE_parse_boot_argn("sweetfb_stayon", &stayon, sizeof(stayon)) && stayon) {
				IOLog("SWEET FB: display off ignored (sweetfb_stayon)\n");
				break;
			}
			IOLockLock(fFB->fLock);
			bool changed = off != fFB->fPowerOff;
			if (changed) {
				fFB->fPowerOff = off;
				fFB->fFadeStep = 0;
				fFB->fDirty = true;     /* on: redraw the latest frame */
			}
			uint32_t level = fFB->fBlLevel;
			IOLockUnlock(fFB->fLock);
			if (changed && fFB->fDsi != NULL) {
				if (off) {
					fFB->setPanelOn(false);
				} else {
					fFB->setPanelOn(true);
					fFB->fBlSent = level;
					fFB->setPanelLevel(level);
				}
			}
			IOLog("SWEET FB: display %s\n", off ? "off" : "on");
		}
		break;
	case kSweetFBSwapWait:
	default:
		break;
	}

	for (uint32_t i = 0; i < args->scalarOutputCount; i++) {
		args->scalarOutput[i] = i < outCount ? out[i] : 0;
	}
	if (args->structureOutputDescriptor == NULL && args->structureOutput != NULL) {
		bzero(args->structureOutput, args->structureOutputSize);
	}
	if (selector >= 64 || !(sweet_fb_logged & (1ULL << selector))) {
		if (selector < 64) {
			sweet_fb_logged |= 1ULL << selector;
		}
		char name[32];
		proc_selfname(name, sizeof(name));
		IOLog("SWEET FB: %s sel %u in %u/%u out %u/%u -> 0x%x\n", name, selector,
		    args->scalarInputCount, args->structureInputSize,
		    args->scalarOutputCount, args->structureOutputSize, ret);
	}
	return ret;
}
#endif /* SWEET */
