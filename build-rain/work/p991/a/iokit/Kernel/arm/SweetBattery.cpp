/*
 * Built-in iOS battery bring-up service. Default: 100%, AC, not charging.
 * sweetbattery=1: read-only PM6150 polling every 30 seconds; voltage-derived
 * SOC is an UNCALIBRATED estimate, not a replacement for the Linux QG algo.
 * See report.md for binary evidence, register provenance and integration.
 */
#if defined(ARM64_BOARD_CONFIG_SWEET)
#include <device/device_types.h> /* io_buf_ptr_t for private OSKext headers */
#include <IOKit/IOLib.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/pwr_mgt/IOPMPowerSource.h>
#include <libkern/c++/OSCollectionIterator.h>
#include <pexpert/pexpert.h>
/* bsd/dev/sweet_spmi.c */
extern "C" int sweet_spmi_init(void);
extern "C" int sweet_spmi_read(unsigned sid, unsigned addr, uint8_t *out);
/* bsd/dev/sweet_i2c1.c: topaz battery bus (QUP0 SE1) */
extern "C" int sweet_i2c1_init(void);
extern "C" int sweet_cpufreq_max(void);    /* bsd/dev/sweet_cpufreq.c */
extern "C" int sweet_i2c1_read(uint8_t addr, uint8_t reg, uint8_t *buf, unsigned len);
extern "C" int sweet_i2c1_write(uint8_t addr, uint8_t reg, uint8_t val);

class SweetBattery : public IOPMPowerSource {
	OSDeclareDefaultStructors(SweetBattery);
public:
	bool init(OSDictionary *dictionary = NULL) APPLE_KEXT_OVERRIDE;
	bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	void stop(IOService *provider) APPLE_KEXT_OVERRIDE;
	void free(void) APPLE_KEXT_OVERRIDE;
private:
	IOWorkLoop *fLoop;
	IOTimerEventSource *fTimer;
	bool fTimerAdded, fRunning, fReal;
	int fLastMode;
	void putNumber(const char *key, UInt32 value);
	void putBool(const char *key, bool value);
	void publish(unsigned percent, unsigned mv, int ma, bool external,
	    bool capable, bool charging, bool full, bool measured, bool currentValid);
	void poll(void);
	bool pollTopaz(void);
	static void timerFired(OSObject *owner, IOTimerEventSource *sender);
	static IOReturn stopPolling(OSObject *owner, void *, void *, void *, void *);
	static bool readWord(unsigned address, uint16_t *value);
	static unsigned estimatePercent(unsigned mv);
};
OSDefineMetaClassAndStructors(SweetBattery, IOPMPowerSource);

bool
SweetBattery::init(OSDictionary *dictionary)
{
	/* IOCatalogue calls init(dictionary). This XNU's IOPMPowerSource only
	 * overrides init(void): explicitly run it to allocate its PS dictionary
	 * and symbols, then copy the matching personality into the registry. */
	if (!IOPMPowerSource::init()) {
		return false;
	}
	fLoop = NULL;
	fTimer = NULL;
	fTimerAdded = fRunning = fReal = false;
	if (dictionary) {
		OSCollectionIterator *it = OSCollectionIterator::withCollection(dictionary);
		if (!it) {
			return false;
		}
		OSObject *object;
		bool ok = true;
		while ((object = it->getNextObject())) {
			const OSSymbol *key = OSDynamicCast(OSSymbol, object);
			if (!key || !setProperty(key, dictionary->getObject(key))) {
				ok = false;
				break;
			}
		}
		it->release();
		if (!ok) {
			return false;
		}
	}
	fLastMode = -1;
	return true;
}

void
SweetBattery::putNumber(const char *key, UInt32 value)
{
	const OSSymbol *symbol = OSSymbol::withCString(key);
	OSNumber *number = OSNumber::withNumber(value, 32);
	if (symbol && number) {
		setPSProperty(symbol, number);
	}
	OSSafeReleaseNULL(number);
	OSSafeReleaseNULL(symbol);
}

void
SweetBattery::putBool(const char *key, bool value)
{
	const OSSymbol *symbol = OSSymbol::withCString(key);
	if (symbol) {
		setPSProperty(symbol, value ? kOSBooleanTrue : kOSBooleanFalse);
		symbol->release();
	}
}

void
SweetBattery::publish(unsigned percent, unsigned mv, int ma, bool external,
    bool capable, bool charging, bool full, bool measured, bool currentValid)
{
	/* Use consistent mAh units (DT nominal=5020). powerd computes C*100/M. */
	const unsigned capacity = (5020u * percent + 50u) / 100u;
	setBatteryInstalled(true);
	setCurrentCapacity(capacity);
	setMaxCapacity(5020);
	setExternalConnected(external);
	setExternalChargeCapable(capable);
	setIsCharging(charging);
	setAtWarnLevel(false);
	/* An uncalibrated voltage curve must not assert a critical shutdown. */
	setAtCriticalLevel(false);
	setVoltage(mv);
	setAmperage(ma);
	setTimeRemaining(-1); /* Unknown; never publish zero minutes on error. */
	setCycleCount(0);     /* Placeholder: cycle learning is not implemented. */
	putBool("FullyCharged", full);
	putBool("AppleRawExternalConnected", external);
	putNumber("AppleRawCurrentCapacity", capacity);
	putNumber("AppleRawMaxCapacity", 5020);
	putNumber("DesignCapacity", 5020);
	putNumber("InstantAmperage", (UInt32)ma);
	/* iOS battery convention: centidegrees C; fixed 25 C, NOT measured. */
	putNumber("Temperature", 2500);
	putBool("SweetBatteryHardwareSampleValid", measured);
	putBool("SweetBatteryCurrentValid", currentValid);
	putBool("SweetBatterySOCEstimated", measured);
	putBool("SweetBatteryTemperatureEstimated", true);
	putBool("SweetBatteryFallback", !measured);
	putNumber("SweetBatteryPercent", percent);

	/* IOPMCopyBatteryInfo's per-source compatibility dictionary. */
	OSDictionary *legacy = OSDictionary::withCapacity(7);
	if (legacy) {
		const char *keys[] = {kIOBatteryCurrentChargeKey, kIOBatteryCapacityKey,
		    kIOBatteryVoltageKey, kIOBatteryAmperageKey, kIOBatteryCycleCountKey,
		    kIOBatteryFlagsKey};
		UInt32 values[] = {capacity, 5020, mv, (UInt32)ma, 0,
		    (UInt32)(kIOBatteryInstalled | (external ? kIOBatteryChargerConnect : 0) |
		    (charging ? kIOBatteryCharge : 0))};
		for (unsigned i = 0; i < sizeof(values) / sizeof(values[0]); ++i) {
			OSNumber *number = OSNumber::withNumber(values[i], 32);
			if (number) {
				legacy->setObject(keys[i], number);
				number->release();
			}
		}
		setLegacyIOBatteryInfo(legacy);
		legacy->release();
	}
	updateStatus(); /* Flush PS properties and send 0xe0024100 on change. */
	int mode = measured ? 1 : 0;
	if (mode != fLastMode) {
		IOLog("SWEET BATTERY: %s, %u%% %umV external=%d charging=%d\n",
		    measured ? "PM6150 / estimated SOC" : "static fallback",
		    percent, mv, external, charging);
		fLastMode = mode;
	}
}

/* Keep the proven single-byte observer command. Read two matching whole
 * words to reject most torn ADC samples without writing the QG HOLD bit.
 * This is a bounded consistency check, not a hardware snapshot guarantee. */
bool
SweetBattery::readWord(unsigned address, uint16_t *value)
{
	uint16_t previous = 0;
	for (unsigned attempt = 0; attempt < 4; ++attempt) {
		uint8_t lo, hi;
		if (sweet_spmi_read(0, address, &lo) ||
		    sweet_spmi_read(0, address + 1, &hi)) {
			return false;
		}
		uint16_t next = (uint16_t)((unsigned)lo | ((unsigned)hi << 8));
		if (attempt && next == previous) {
			*value = next;
			return true;
		}
		previous = next;
	}
	return false;
}

unsigned
SweetBattery::estimatePercent(unsigned mv)
{
	/* Generic single-cell curve, intentionally NOT advertised as QG SOC.
	 * Load sag, temperature, aging and charging introduce large errors. */
	static const unsigned volts[] = {3400, 3500, 3600, 3700, 3800, 3900,
	    4000, 4100, 4200, 4350};
	static const unsigned soc[] = {1, 5, 10, 20, 40, 60, 75, 85, 93, 100};
	if (mv <= volts[0]) {
		return soc[0];
	}
	for (unsigned i = 1; i < sizeof(volts) / sizeof(volts[0]); ++i) {
		if (mv < volts[i]) {
			return soc[i - 1] + (mv - volts[i - 1]) * (soc[i] - soc[i - 1]) /
			       (volts[i] - volts[i - 1]);
		}
	}
	return 100;
}

/*
 * topaz charger control (MakrSas/topaz-woa TopazBattery gauge.c ChargerLimits/ChargerPolicy):
 * left alone the bq2589x-class charger at 0x6a runs on its power-on defaults (slow, and its
 * watchdog puts them back), so with an adapter in: input limit from BC1.2 (CDP 1.5 A,
 * DCP/HVDCP 2 A), charge current 2 A and 4.40 V with the topaz DTB software JEITA zones on
 * the pack's NTC (SM5602 TEMP_EX, DTB thermal table), charging on, OTG off, watchdog off.
 * No PD contract, no charge pumps (the fast path stays Windows-only for now). sweetchg=0: hands off.
 */
static const uint16_t topaz_ntc[101] = {        /* -20..80 degC, signed 16-bit */
	0x506e, 0x4f35, 0x4e07, 0x4cc0, 0x4b76, 0x4a25, 0x48bd, 0x475d, 0x45f5, 0x4477,
	0x4304, 0x4175, 0x3fde, 0x3e4a, 0x3ca2, 0x3af6, 0x3945, 0x378a, 0x35cb, 0x33fc,
	0x322b, 0x3059, 0x2e82, 0x2ca0, 0x2ac6, 0x28c6, 0x26e0, 0x24ea, 0x22e8, 0x20ef,
	0x1ee7, 0x1cee, 0x1ae8, 0x18d0, 0x16d6, 0x14ce, 0x12f3, 0x10e4, 0x0ed6, 0x0cc4,
	0x0abc, 0x08bc, 0x06a8, 0x04ad, 0x02a1, 0x009e, 0xf669, 0xf471, 0xf278, 0xf081,
	0xee7b, 0xec8c, 0xeaa1, 0xe8be, 0xe6e1, 0xe505, 0xe332, 0xe162, 0xdf9b, 0xddd9,
	0xdc1f, 0xda67, 0xd8b4, 0xd70b, 0xd565, 0xd3c7, 0xd232, 0xd0a1, 0xcf18, 0xcd94,
	0xcc18, 0xcaa5, 0xc936, 0xc7d3, 0xc670, 0xc519, 0xc3c6, 0xc27a, 0xc136, 0xbff7,
	0xbec1, 0xbd92, 0xbc6e, 0xbb4d, 0xba36, 0xb922, 0xb816, 0xb712, 0xb611, 0xb517,
	0xb422, 0xb334, 0xb249, 0xb164, 0xb085, 0xafad, 0xaed8, 0xae0a, 0xad3e, 0xac78,
	0xabb7,
};

static int
topaz_ntc_tenths(uint16_t raw)
{
	int16_t v = (int16_t)raw;
	if (raw >= 0x8001 && raw <= 0x823B) {
		v = 0;
	}
	if (v >= (int16_t)topaz_ntc[0]) {
		return -200;
	}
	for (int i = 1; i < 101; i++) {
		if (v >= (int16_t)topaz_ntc[i]) {
			int hi = (int16_t)topaz_ntc[i - 1], lo = (int16_t)topaz_ntc[i];
			return (i - 1) * 10 - 200 + (hi - v) * 10 / (hi - lo);
		}
	}
	return 800;
}

static const struct { int min_tenths; unsigned ichg_ma, vreg_mv; } topaz_jeita[] = {
	{ -1000, 0, 0 }, { -100, 490, 4400 }, { 0, 950, 4400 }, { 50, 2000, 4400 },
	{ 480, 1024, 4100 }, { 600, 0, 0 },
};
#define TOPAZ_JEITA_ZONES (sizeof(topaz_jeita) / sizeof(topaz_jeita[0]))

static unsigned
topaz_jeita_zone(int tenths)
{
	unsigned z = 0;
	while (z + 1 < TOPAZ_JEITA_ZONES && tenths >= topaz_jeita[z + 1].min_tenths) {
		z++;
	}
	return z;
}

static void
topaz_charger(uint8_t r0b, int tenths)
{
	static int enabled = -1;
	static unsigned zone = TOPAZ_JEITA_ZONES;
	unsigned vbus = (r0b >> 5) & 7u;
	uint8_t r00, r03, r04, r06, r07;

	if (enabled < 0) {
		enabled = 1;
		(void)PE_parse_boot_argn("sweetchg", &enabled, sizeof(enabled));
	}
	if (!enabled || vbus < 1 || vbus > 6) {
		return;
	}
	/* to a zone with less current at once, to one with more only 1.5 degC inside it */
	unsigned z = topaz_jeita_zone(tenths);
	if (zone < TOPAZ_JEITA_ZONES && z != zone && topaz_jeita[z].ichg_ma >= topaz_jeita[zone].ichg_ma &&
	    (topaz_jeita_zone(tenths - 15) != z || topaz_jeita_zone(tenths + 15) != z)) {
		z = zone;
	}
	if (sweet_i2c1_read(0x6a, 0x00, &r00, 1) || sweet_i2c1_read(0x6a, 0x03, &r03, 1) ||
	    sweet_i2c1_read(0x6a, 0x04, &r04, 1) || sweet_i2c1_read(0x6a, 0x06, &r06, 1) ||
	    sweet_i2c1_read(0x6a, 0x07, &r07, 1)) {
		return;
	}
	unsigned iin = vbus == 2 ? 1500 : (vbus == 3 || vbus == 4) ? 2000 : 0;
	uint8_t w00 = iin ? (uint8_t)((r00 & 0xC0) | ((iin - 100) / 50)) : r00;
	uint8_t w04 = r04, w06 = r06;
	bool allow = topaz_jeita[z].ichg_ma != 0;
	if (allow) {
		w04 = (uint8_t)((r04 & 0x80) | (topaz_jeita[z].ichg_ma / 64));
		w06 = (uint8_t)((r06 & 0x03) | (((topaz_jeita[z].vreg_mv - 3840) / 16) << 2));
	}
	uint8_t w03 = (uint8_t)(((r03 & ~0x10u) | (allow ? 0x10u : 0)) & ~0x20u);   /* CHG_CONFIG, OTG off */
	uint8_t w07 = (uint8_t)(r07 & ~0x30u);                                    /* watchdog off */
	if (w06 != r06) sweet_i2c1_write(0x6a, 0x06, w06);
	if (w04 != r04) sweet_i2c1_write(0x6a, 0x04, w04);
	if (w00 != r00) sweet_i2c1_write(0x6a, 0x00, w00);
	if (w07 != r07) sweet_i2c1_write(0x6a, 0x07, w07);
	if (w03 != r03) sweet_i2c1_write(0x6a, 0x03, w03);
	if (z != zone || w00 != r00 || w04 != r04 || w03 != r03) {
		IOLog("SWEET CHARGER: vbus_stat %u, %d.%d C zone %u: iinlim %u mA, ichg %u mA, vreg %u mV, %s (r00 %02x r03 %02x r04 %02x r06 %02x r07 %02x)\n",
		    vbus, tenths / 10, (tenths < 0 ? -tenths : tenths) % 10, z, 100 + 50 * (w00 & 0x3F), 64 * (w04 & 0x7F),
		    3840 + 16 * (w06 >> 2), allow ? "charging" : "NOT charging (temperature)", w00, w03, w04, w06, w07);
	}
	zone = z;
}

/*
 * topaz (MakrSas/topaz-woa TopazBattery gauge.c): SM5602 fuel gauge at 0x71
 * (16-bit little-endian words) and the bq25890-map charger at 0x6a, REG0B.
 * The gauge is only read; the charger gets its limits from topaz_charger() above.
 */
bool
SweetBattery::pollTopaz(void)
{
	uint8_t w[2], st = 0;
	uint16_t soc, volt, cur, full;
	if (sweet_i2c1_read(0x71, 0x05, w, 2)) {
		return false;
	}
	soc = (uint16_t)(w[0] | w[1] << 8);
	if (sweet_i2c1_read(0x71, 0x07, w, 2)) {
		return false;
	}
	volt = (uint16_t)(w[0] | w[1] << 8);
	if (sweet_i2c1_read(0x71, 0x08, w, 2)) {
		return false;
	}
	cur = (uint16_t)(w[0] | w[1] << 8);
	bool fullOk = sweet_i2c1_read(0x71, 0x62, w, 2) == 0;
	full = fullOk ? (uint16_t)(w[0] | w[1] << 8) : 0;
	bool chgOk = sweet_i2c1_read(0x6a, 0x0b, &st, 1) == 0;
	bool texOk = sweet_i2c1_read(0x71, 0x0a, w, 2) == 0;
	int tenths = texOk ? topaz_ntc_tenths((uint16_t)(w[0] | w[1] << 8)) : 250;
	if (chgOk && texOk) {
		topaz_charger(st, tenths);
	}

	unsigned percent = (soc & 0x8000u) ? 0 : ((soc >> 8) & 0x7fu);
	unsigned mv = 1800u * (volt & 0x7fffu) / 19622u + 2700u;
	int ma = (int)((cur & 0x7fffu) * 1000u / 4088u * 2u);
	if (cur & 0x8000u) {
		ma = -ma;               /* bit 15: discharging */
	}
	unsigned fullMah = (full & 0x7fffu) * 1000u / 2048u;
	if (percent > 100 || mv < 2800 || mv > 4600) {
		return false;
	}
	unsigned vbus = (st >> 5) & 7u, chrg = (st >> 3) & 3u;
	bool external = chgOk && vbus >= 1 && vbus <= 6 && (st & 4u);
	bool charging = external && (chrg == 1 || chrg == 2);
	bool done = external && chrg == 3;
	publish(done ? 100 : percent, mv, ma, external, external, charging, done, true, true);
	static unsigned logs;
	if (logs++ % 20 == 0) {
		IOLog("SWEET BATTERY: topaz %u%% %umV %dmA full=%umAh chg=%#x external=%d charging=%d\n",
		    percent, mv, ma, fullMah, st, external, charging);
	}
	return true;
}

void
SweetBattery::poll(void)
{
	if (fReal && pollTopaz()) {
		return;
	}
	if (fReal) {
		publish(100, 4200, 0, true, true, false, true, false, false);
		return;
	}
	uint8_t type, present, usb, path, charger, error, temperature;
	uint16_t rawVoltage;
	bool valid = fReal &&
	    !sweet_spmi_read(0, 0x4804, &type) && type == 0x0d &&
	    !sweet_spmi_read(0, 0x4808, &present) && (present & 1u) &&
	    readWord(0x48c0, &rawVoltage) && rawVoltage != 0x8000 &&
	    !sweet_spmi_read(0, 0x1310, &usb) &&
	    !sweet_spmi_read(0, 0x110b, &path) &&
	    !sweet_spmi_read(0, 0x1006, &charger) &&
	    !sweet_spmi_read(0, 0x1007, &error) &&
	    !sweet_spmi_read(0, 0x100d, &temperature);
	unsigned mv = valid ? (unsigned)(((uint64_t)rawVoltage * 194637u) / 1000000u) : 0;
	/* Zero, reset, floating, or implausible data cannot become 0% battery. */
	valid = valid && mv >= 2800 && mv <= 4600;
	if (!valid) {
		publish(100, 4200, 0, true, true, false, true, false, false);
		return;
	}
	bool external = (usb & 0x10u) != 0;
	bool capable = external && (path & 0x10u) && (path & 1u) && !(path & 0x40u);
	unsigned state = charger & 7u;
	bool charging = capable && state >= 1 && state <= 4 &&
	    !(error & 2u) && !(temperature & 0x0cu);
	/* Inhibit (0) does not by itself establish full capacity. */
	bool full = external && state == 5 && !(error & 2u);
	int ma = 0;
	uint8_t subtype;
	uint16_t rawCurrent;
	bool currentValid = !sweet_spmi_read(0, 0x4805, &subtype) &&
	    (subtype == 3 || subtype == 4) && readWord(0x48c2, &rawCurrent) &&
	    rawCurrent != 0x8000;
	if (currentValid) {
		int signedRaw = (rawCurrent & 0x8000u) ? (int)rawCurrent - 65536 : (int)rawCurrent;
		/* Linux: negative=charging; IOPMPowerSource: positive=charging, mA. */
		ma = (int)(-((int64_t)signedRaw * (subtype == 3 ? 152588 : 305176)) / 1000000);
	}
	publish(full ? 100 : estimatePercent(mv), mv, ma, external, capable,
	    charging, full, true, currentValid);
}

void
SweetBattery::timerFired(OSObject *owner, IOTimerEventSource *sender)
{
	SweetBattery *self = OSDynamicCast(SweetBattery, owner);
	if (self && self->fRunning) {
		self->poll();
		sender->setTimeoutMS(10000);
	}
}

bool
SweetBattery::start(IOService *provider)
{
	if (!IOPMPowerSource::start(provider)) {
		return false;
	}
	(void)sweet_cpufreq_max();
	int real = 1;   /* topaz: real gauge by default; sweetbattery=0 for the static fallback */
	(void)PE_parse_boot_argn("sweetbattery", &real, sizeof(real));
	fReal = real == 1 && sweet_i2c1_init() == 0;
	/* Publish a complete source BEFORE registerService wakes powerd. */
	poll();
	if (fReal) {
		fLoop = IOWorkLoop::workLoop();
		fTimer = IOTimerEventSource::timerEventSource(this, &SweetBattery::timerFired);
		if (fLoop && fTimer && fLoop->addEventSource(fTimer) == kIOReturnSuccess) {
			fTimerAdded = true;
			fRunning = true;
			fTimer->setTimeoutMS(10000);
		} else {
			fReal = false;
			poll();
			IOLog("SWEET BATTERY: timer unavailable; static fallback\n");
		}
	}
	registerService();
	return true;
}

void
SweetBattery::stop(IOService *provider)
{
	if (fLoop && fTimerAdded) {
		/* Serialize against a running timer callback, including rearming. */
		(void)fLoop->runAction(&SweetBattery::stopPolling, this);
		fLoop->removeEventSource(fTimer);
		fTimerAdded = false;
	}
	IOPMPowerSource::stop(provider);
}

IOReturn
SweetBattery::stopPolling(OSObject *owner, void *, void *, void *, void *)
{
	SweetBattery *self = OSDynamicCast(SweetBattery, owner);
	if (!self) {
		return kIOReturnBadArgument;
	}
	self->fRunning = false;
	self->fTimer->cancelTimeout();
	self->fTimer->disable();
	return kIOReturnSuccess;
}

void
SweetBattery::free(void)
{
	OSSafeReleaseNULL(fTimer);
	OSSafeReleaseNULL(fLoop);
	IOPMPowerSource::free();
}
#endif /* ARM64_BOARD_CONFIG_SWEET */
