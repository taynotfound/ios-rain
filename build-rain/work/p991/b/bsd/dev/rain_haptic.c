/*
 * rain: the vibration motor (DT vibrator_gpio: qcom,qpnp-vibrator-ldo switched by TLMM GPIO 36,
 * EAST tile of pinctrl-khaje.c) behind /dev/redwood-haptic, where the RedwoodAudio HAL plug-in
 * writes iOS's haptic waveforms (16-bit mono samples, userland/audio/audio_plugin.c
 * actuator_write()). The motor is on/off only: on while the waveform has any strength, off
 * again once that much audio time has passed.
 */
#if defined(ARM64_BOARD_CONFIG_SWEET)
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/uio.h>
#include <sys/errno.h>
#include <kern/thread_call.h>
#include <kern/clock.h>
#include <machine/machine_routines.h>
#include <miscfs/devfs/devfs.h>
#include <pexpert/pexpert.h>

extern vm_offset_t ml_io_map(vm_offset_t, vm_size_t);
void rain_haptic_init(void);

#define HAPTIC_GPIO_PA   0x00d24000u     /* TLMM 0x400000 + EAST 0x900000 + 36 * 0x1000 */
#define HAPTIC_RATE      48000u          /* the plug-in's sample rate */
#define HAPTIC_LEVEL     1500            /* |sample| above this switches the motor on */

static vm_offset_t hg;
static thread_call_t haptic_off_call;

static void
haptic_motor(bool on)
{
	*(volatile uint32_t *)(hg + 4) = on ? 2u : 0u;     /* GPIO_IN_OUT: bit 1 = output level */
	__asm__ volatile("dsb sy" ::: "memory");
}

static void
haptic_off(thread_call_param_t p0 __unused, thread_call_param_t p1 __unused)
{
	haptic_motor(false);
}

static int
haptic_open(dev_t dev __unused, int flags __unused, int type __unused, struct proc *p __unused)
{
	return hg ? 0 : ENXIO;
}

static int
haptic_close(dev_t dev __unused, int flags __unused, int type __unused, struct proc *p __unused)
{
	return 0;
}

static int
haptic_write(dev_t dev __unused, struct uio *uio, int flags __unused)
{
	int16_t buf[512];
	uint32_t frames = 0;
	int peak = 0, err = 0;

	while (uio_resid(uio) > 0 && err == 0) {
		user_ssize_t n = MIN(uio_resid(uio), (user_ssize_t)sizeof(buf));
		err = uiomove((caddr_t)buf, (int)n, uio);
		for (int i = 0; err == 0 && i < (int)(n / 2); i++) {
			int v = buf[i] < 0 ? -buf[i] : buf[i];
			peak = v > peak ? v : peak;
		}
		frames += (uint32_t)(n / 2);
	}
	if (err == 0 && peak > HAPTIC_LEVEL) {
		uint64_t deadline;
		uint32_t ms = frames * 1000u / HAPTIC_RATE + 15u;
		haptic_motor(true);
		clock_interval_to_deadline(ms, NSEC_PER_MSEC, &deadline);
		thread_call_enter_delayed(haptic_off_call, deadline);       /* a later write moves it on */
	}
	return err;
}

static const struct cdevsw haptic_cdevsw = {
	.d_open = haptic_open, .d_close = haptic_close, .d_read = eno_rdwrt, .d_write = haptic_write,
	.d_ioctl = eno_ioctl, .d_stop = eno_stop, .d_reset = eno_reset, .d_ttys = NULL, .d_select = eno_select,
	.d_mmap = eno_mmap, .d_strategy = eno_strat, .d_reserved_1 = eno_getc, .d_reserved_2 = eno_putc, .d_type = 0,
};

void
rain_haptic_init(void)
{
	int on = 1;
	(void)PE_parse_boot_argn("rainhaptic", &on, sizeof(on));
	if (!on || hg) {
		return;
	}
	hg = ml_io_map(HAPTIC_GPIO_PA, 0x1000);
	haptic_off_call = thread_call_allocate(haptic_off, NULL);
	if (!hg || !haptic_off_call) {
		printf("RAIN HAPTIC: no mapping\n");
		hg = 0;
		return;
	}
	*(volatile uint32_t *)(hg + 4) = 0;                /* low first, then output enable */
	*(volatile uint32_t *)hg = (*(volatile uint32_t *)hg & ~0x3ffu) | (1u << 9);   /* GPIO function, OE, 2 mA */
	__asm__ volatile("dsb sy" ::: "memory");
	int major = cdevsw_add(-1, &haptic_cdevsw);
	void *node = major < 0 ? NULL : devfs_make_node(makedev(major, 0), DEVFS_CHAR, UID_ROOT, GID_WHEEL, 0666, "redwood-haptic");
	printf("RAIN HAPTIC: /dev/redwood-haptic %s (gpio36 ctl %#x)\n", node ? "ready" : "failed", *(volatile uint32_t *)hg);
	uint32_t buzz = 0;                                /* boot-arg rainbuzz=1: one 300 ms buzz, to test */
	if (PE_parse_boot_argn("rainbuzz", &buzz, sizeof(buzz)) && buzz && node) {
		uint64_t deadline;
		haptic_motor(true);
		clock_interval_to_deadline(300, NSEC_PER_MSEC, &deadline);
		thread_call_enter_delayed(haptic_off_call, deadline);
	}
}
#endif /* ARM64_BOARD_CONFIG_SWEET */
