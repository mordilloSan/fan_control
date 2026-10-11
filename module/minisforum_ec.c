// SPDX-License-Identifier: GPL-2.0
/*
 * hwmon driver for the ITE IT5571 embedded controller of the Minisforum
 * UM760 Slim (board F7BSI, EC firmware 0.5).
 *
 * The register map, admission rules and I2EC access sequence come from the
 * "f7bsi" profile of FanControl.MinisforumUM (github.com/zuyan9), commit
 * d61e46c. File:line references point into that repository.
 *
 * Each fan (pwm1 CPU, pwm2 system) runs in one of three modes: a fixed value
 * written by userspace, this driver's temperature curve (module parameters
 * cpu_curve and system_curve), or the EC firmware's own policy.
 *
 * EC memory writes are limited to what the plugin writes: the system-fan
 * handoff (override 0x088b to 0x00/0xff, target 0x0885 to 0..51) and the
 * seven bases and slopes of the BIOS-selected CPU table, never its bands or
 * critical row. Every write checks its preconditions first and is read back.
 *
 * The driver moves a fan one code at a time, because a jump makes the EC's
 * speed loop hunt; a stopped fan restarts at SPINUP_CODE and steps down. It
 * gives the fan back to the firmware at a curve's firmware point, in mode 3,
 * when its EC temperature reaches its limit or reads implausibly, when it
 * hunts, when the EC is not in the state it last wrote, when a write or the
 * telemetry fails, on unload, sleep and shutdown. A fixed value that is not
 * renewed for WATCHDOG_SECONDS falls back to the curve.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/acpi.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dmi.h>
#include <linux/hwmon.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/stddef.h>
#include <linux/string.h>
#include <linux/stringify.h>
#include <linux/workqueue.h>

#define DRVNAME			"minisforum_ec"

/* Super-I/O index/data ports: PawnIO LpcIO slot 0 (LpcIO.p:136-152). */
#define SIO_INDEX		0x2e
#define SIO_DATA		0x2f

/* Super-I/O configuration registers (PawnIoNative.cs:227-274). */
#define SIO_REG_CHIP_ID		0x20	/* 0x20..0x22: ID high, ID low, revision */
#define SIO_REG_D2_INDEX	0x2e	/* selects a register in the EC's D2 bank */
#define SIO_REG_D2_DATA		0x2f

/* D2 registers for indirect (I2EC) access to EC memory. */
#define D2_I2EC_ADDR_LOW	0x10
#define D2_I2EC_ADDR_HIGH	0x11
#define D2_I2EC_DATA		0x12

/* EC memory, I2EC addresses (F7bsdProfile.cs:28-88). */
#define EC_TEMP_SYS		0x0305
#define EC_TEMP_CPU		0x0309
#define EC_CPU_ROWS		0x0310	/* 8 x (base, upper, lower); row 7 is critical */
#define EC_CPU_SELECTOR		0x032f
#define EC_SYS_POLICY_ROWS	0x0330	/* 3 x (code, upper, lower) */
#define EC_SYS_TARGET		0x0885
#define EC_SYS_EFFECTIVE	0x0889
#define EC_CPU_TEMP_OVERRIDE	0x088a
#define EC_SYS_OVERRIDE		0x088b
#define EC_CPU_SLOPES		0x08b0	/* 8 slopes; slope 7 belongs to the critical row */
#define EC_SYS_POLICY_TAIL	0x08c0	/* 3 bytes, all zero */
#define EC_DUTY_CPU		0x1803	/* PWM duty, diagnostics only (see below) */
#define EC_DUTY_SYS		0x1804
#define EC_TACH_CPU		0x181e	/* counter low byte; high byte at +1 */
#define EC_TACH_SYS		0x1820

#define CPU_ROWS		7	/* normal rows; the critical row follows */
#define CODE_MAX		51	/* F7bsdProfile.cs:28 */
#define PWM_PER_CODE		5	/* hwmon pwmN 0..255 is CODE_MAX * 5 */
#define SYS_SENTINEL		0xff	/* F7bsdProfile.cs:29 */
#define STABLE_ATTEMPTS		4	/* PawnIoF7bsdBackend.cs:5 */
#define TACH_ATTEMPTS		3	/* PawnIoF7bsdBackend.cs:603 */
#define TACH_DIVIDEND		2156250	/* PawnIoF7bsdBackend.cs:655 */
#define CACHE_JIFFIES		HZ
#define SYS_RELEASE_STEPS	4	/* PawnIoF7bsdBackend.cs:6 */
#define SYS_HANDOFF_POLLS	16	/* PawnIoF7bsdBackend.cs:7 */
#define SYS_HANDOFF_POLL_MS	100	/* PawnIoF7bsdBackend.cs:8-9 */
#define SYS_POLICY_BYTES	12

/* Behaviour and safety limits of this driver; not from the plugin. */
#define WATCHDOG_SECONDS	30	/* a fixed value must be renewed within this */
#define SYS_TEMP_LIMIT		55	/* EC system temperature handing pwm2 to firmware */
#define CPU_TEMP_LIMIT		80	/* EC CPU temperature handing pwm1 to firmware */
#define LIMIT_HYST		5	/* ... until it falls this far below the limit */
#define CURVE_HYST		3	/* degrees below a curve point before leaving it */
#define SYS_MIN_CODE		8	/* codes 1..7 made the system fan hunt (measured) */
#define CPU_MIN_CODE		6	/* code 5 made the CPU fan hunt (measured) */
#define SPINUP_CODE		20	/* a stopped fan restarts here and steps down */
#define MONITOR_JIFFIES		(2 * HZ)
#define STEP_JIFFIES		(4 * HZ)	/* at most one code per step */
#define HUNT_WINDOW		6	/* monitor samples looked at for hunting */
#define HUNT_SPREAD		600	/* RPM swing over that window that counts as hunting */
#define HUNT_BACKOFF		(60 * HZ)
#define MAX_HUNTS		3
#define HUNT_FORGET		(30 * 60 * HZ)
#define SETTLE_JIFFIES		(10 * HZ)	/* after a write, before hunting is judged */
#define MAX_READ_FAILURES	3	/* consecutive telemetry failures before letting go */
#define MAX_ENGAGE_FAILURES	3	/* consecutive failed engages before mode 3 */
#define RELEASE_RETRY_SHIFT	7U	/* failed releases retry after 2 s, 4 s, ... 256 s */
#define RESUME_SETTLE_JIFFIES	(5 * HZ)	/* the EC reads zeros for a moment after waking */

/* Controller signature, compared with the PNP bytes (F7bsdProfile.cs:34-35). */
static const u16 ec_ident_addrs[] = { 0x2000, 0x2001, 0x2002, 0x200d, 0x180c, 0x1841 };

/*
 * Every EC address this driver may read: the plugin's read allowlist
 * (F7bsdProfile.cs:90-97) plus the two PWM duty registers. Those two are read
 * for diagnostics only, to observe the system-fan behaviour of upstream
 * issue #14 (technical.md:121-124; 0x1803/0x1804 are pwm1/pwm2 in the
 * author's diagnostic log).
 */
static const struct {
	u16 first;
	u16 last;
} xram_readable_ranges[] = {
	{ 0x0305, 0x0305 },
	{ 0x0309, 0x0309 },
	{ 0x0310, 0x0327 },
	{ 0x032f, 0x032f },
	{ 0x0330, 0x0338 },
	{ 0x0885, 0x0885 },
	{ 0x0889, 0x088b },
	{ 0x08b0, 0x08b7 },
	{ 0x08c0, 0x08c2 },
	{ 0x1803, 0x1804 },
	{ 0x180c, 0x180c },
	{ 0x181e, 0x1821 },
	{ 0x1841, 0x1841 },
	{ 0x2000, 0x2002 },
	{ 0x200d, 0x200d },
};

struct cpu_table {
	u8 selector;
	u8 temp_override;
	u8 rows[CPU_ROWS + 1][3];	/* base, upper, lower */
	u8 slopes[CPU_ROWS + 1];
};

struct sys_policy {
	u8 rows[3][3];			/* code, upper, lower */
	u8 tail[3];
};

struct cpu_profile {
	u8 selector;
	const char *name;
	u8 bands[CPU_ROWS][2];		/* upper, lower */
	u8 bases[CPU_ROWS];
	u8 slopes[CPU_ROWS];
};

#define STANDARD_BANDS \
	{ { 25, 0 }, { 45, 25 }, { 54, 45 }, { 66, 54 }, { 76, 66 }, { 88, 76 }, { 93, 88 } }
#define PERFORMANCE_BANDS \
	{ { 25, 0 }, { 45, 25 }, { 54, 45 }, { 66, 54 }, { 80, 66 }, { 88, 80 }, { 93, 88 } }

/*
 * Canonical BIOS-selected CPU tables (PlatformProfiles.cs:293-313, 370-378);
 * names from MinisforumUMSeriesPlugin.cs:239-241.
 */
static const struct cpu_profile f7bsi_cpu_profiles[] = {
	{ 0x00, "Default", STANDARD_BANDS,
	  { 0, 16, 18, 22, 25, 27, 29 }, { 0, 10, 44, 25, 20, 16, 75 } },
	{ 0xb1, "Balance", STANDARD_BANDS,
	  { 0, 16, 18, 22, 25, 27, 29 }, { 0, 10, 44, 25, 20, 16, 75 } },
	{ 0xb2, "Performance", PERFORMANCE_BANDS,
	  { 0, 18, 22, 29, 31, 34, 37 }, { 0, 20, 77, 16, 25, 37, 125 } },
};

/* Critical row (base, upper, lower) and its slope (F7bsdProfile.cs:88). */
static const u8 cpu_critical_row[3] = { 51, 100, 93 };
#define CPU_CRITICAL_SLOPE	0

/* PlatformProfiles.cs:323-329 */
static const struct sys_policy f7bsi_sys_policy = {
	.rows = { { 0, 25, 0 }, { 15, 83, 25 }, { 40, 100, 83 } },
};

/* Host identity (PlatformProfiles.cs:439-446). */
static const struct {
	int field;
	const char *name;
	const char *value;
} f7bsi_host[] = {
	{ DMI_PRODUCT_NAME, "product_name", "EliteMini Series" },
	{ DMI_PRODUCT_VERSION, "product_version", "1.0" },
	{ DMI_PRODUCT_FAMILY, "product_family", "EliteMini" },
	{ DMI_PRODUCT_SKU, "product_sku", "MGF7BSI" },
	{ DMI_BOARD_NAME, "board_name", "F7BSI" },
	{ DMI_BOARD_VERSION, "board_version", "1.0" },
};
#define F7BSI_EC_MAJOR		0
#define F7BSI_EC_MINOR		5

enum sys_state {
	SYS_FIRMWARE,
	SYS_FIXED,
	SYS_RELEASING,
	SYS_UNSUPPORTED,
};

static const char * const sys_state_names[] = {
	[SYS_FIRMWARE] = "firmware",
	[SYS_FIXED] = "fixed-target",
	[SYS_RELEASING] = "releasing",
	[SYS_UNSUPPORTED] = "unsupported",
};

struct telemetry {
	int rpm[2];			/* CPU, system */
	u8 temp[2];			/* CPU, system; raw degrees C */
	u8 sys_effective;
	u8 sys_override;
	u8 sys_target;
	u8 duty[2];			/* CPU, system */
};

struct sys_regs {
	u8 effective;
	u8 override;
	u8 target;
};

struct ec_byte {
	u16 addr;
	u8 val;
};

static struct platform_device *mec_pdev;

/* Port primitives. Callers hold mec_data.lock and the I/O region. */

static void sio_write(u8 reg, u8 val)
{
	outb(reg, SIO_INDEX);
	outb(val, SIO_DATA);
}

static u8 sio_read(u8 reg)
{
	outb(reg, SIO_INDEX);
	return inb(SIO_DATA);
}

/* PawnIoNative.cs:274 */
static void sio_park_pnp(void)
{
	outb(SIO_REG_CHIP_ID, SIO_INDEX);
}

/* PawnIoNative.cs:266-272 */
static void sio_park(void)
{
	sio_write(SIO_REG_D2_INDEX, D2_I2EC_ADDR_LOW);
	sio_park_pnp();
}

/* PawnIoNative.cs:227-233 */
static u8 pnp_read(u8 reg)
{
	u8 val = sio_read(reg);

	sio_park_pnp();
	return val;
}

static void d2_write(u8 reg, u8 val)
{
	sio_write(SIO_REG_D2_INDEX, reg);
	sio_write(SIO_REG_D2_DATA, val);
}

/* PawnIoNative.cs:235-243, 258-264 */
static u8 i2ec_read(u16 addr)
{
	u8 val;

	d2_write(D2_I2EC_ADDR_HIGH, addr >> 8);
	d2_write(D2_I2EC_ADDR_LOW, addr & 0xff);
	sio_write(SIO_REG_D2_INDEX, D2_I2EC_DATA);
	val = sio_read(SIO_REG_D2_DATA);
	sio_park();
	return val;
}

/* PawnIoNative.cs:245-254 */
static void i2ec_write(u16 addr, u8 val)
{
	d2_write(D2_I2EC_ADDR_HIGH, addr >> 8);
	d2_write(D2_I2EC_ADDR_LOW, addr & 0xff);
	sio_write(SIO_REG_D2_INDEX, D2_I2EC_DATA);
	sio_write(SIO_REG_D2_DATA, val);
	sio_park();
}

static bool xram_readable(u16 addr)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(xram_readable_ranges); i++)
		if (addr >= xram_readable_ranges[i].first && addr <= xram_readable_ranges[i].last)
			return true;
	return false;
}

static int xram_read(u16 addr, u8 *val)
{
	if (WARN_ONCE(!xram_readable(addr),
		      DRVNAME ": EC read 0x%04x is not allowed\n", addr))
		return -EPERM;
	*val = i2ec_read(addr);
	return 0;
}

static int xram_read_range(u16 first, u8 *out, size_t n)
{
	size_t i;
	int ret;

	for (i = 0; i < n; i++) {
		ret = xram_read(first + i, &out[i]);
		if (ret)
			return ret;
	}
	return 0;
}

/*
 * Configuration snapshots are accepted only when two consecutive reads agree
 * (PawnIoF7bsdBackend.cs:546-562).
 */
static int read_stable(int (*read)(void *buf), void *out, void *tmp, size_t size)
{
	int i, ret;

	ret = read(out);
	if (ret)
		return ret;
	for (i = 1; i < STABLE_ATTEMPTS; i++) {
		ret = read(tmp);
		if (ret)
			return ret;
		if (!memcmp(tmp, out, size))
			return 0;
		memcpy(out, tmp, size);
	}
	return -EIO;
}

static int read_cpu_table(void *buf)
{
	struct cpu_table *t = buf;
	int ret;

	ret = xram_read(EC_CPU_SELECTOR, &t->selector);
	if (ret)
		return ret;
	ret = xram_read(EC_CPU_TEMP_OVERRIDE, &t->temp_override);
	if (ret)
		return ret;
	ret = xram_read_range(EC_CPU_ROWS, &t->rows[0][0], sizeof(t->rows));
	if (ret)
		return ret;
	return xram_read_range(EC_CPU_SLOPES, t->slopes, sizeof(t->slopes));
}

static int read_sys_policy(void *buf)
{
	struct sys_policy *p = buf;
	int ret;

	ret = xram_read_range(EC_SYS_POLICY_ROWS, &p->rows[0][0], sizeof(p->rows));
	if (ret)
		return ret;
	return xram_read_range(EC_SYS_POLICY_TAIL, p->tail, sizeof(p->tail));
}

/*
 * Tachometer: low, high, low again; the two low bytes must agree
 * (PawnIoF7bsdBackend.cs:597-615, 638-657).
 */
static int read_tach(u16 low, int *rpm)
{
	u8 lo1, hi, lo2;
	u16 counter;
	int i, ret;

	for (i = 0; i < TACH_ATTEMPTS; i++) {
		ret = xram_read(low, &lo1);
		if (!ret)
			ret = xram_read(low + 1, &hi);
		if (!ret)
			ret = xram_read(low, &lo2);
		if (ret)
			return ret;
		if (lo1 != lo2)
			continue;
		counter = lo1 | hi << 8;
		*rpm = counter ? TACH_DIVIDEND / counter : 0;
		return 0;
	}
	return -EIO;
}

/* F7bsdProfile.cs:304 */
static bool plausible_temp(u8 val)
{
	return val >= 1 && val <= 120;
}

/* F7bsdProfile.cs:223-250 */
static enum sys_state classify_sys(u8 effective, u8 override, u8 target)
{
	if (target > CODE_MAX)
		return SYS_UNSUPPORTED;
	if (override == 0) {
		if (plausible_temp(effective))
			return SYS_FIRMWARE;
		return effective == SYS_SENTINEL ? SYS_RELEASING : SYS_UNSUPPORTED;
	}
	if (override == SYS_SENTINEL &&
	    (effective == SYS_SENTINEL || plausible_temp(effective)))
		return SYS_FIXED;
	return SYS_UNSUPPORTED;
}

static int read_telemetry(struct telemetry *t)
{
	int ret;

	ret = read_tach(EC_TACH_CPU, &t->rpm[0]);
	if (ret)
		return ret;
	ret = read_tach(EC_TACH_SYS, &t->rpm[1]);
	if (ret)
		return ret;
	ret = xram_read(EC_TEMP_CPU, &t->temp[0]);
	if (ret)
		return ret;
	ret = xram_read(EC_TEMP_SYS, &t->temp[1]);
	if (ret)
		return ret;
	ret = xram_read(EC_SYS_EFFECTIVE, &t->sys_effective);
	if (ret)
		return ret;
	ret = xram_read(EC_SYS_OVERRIDE, &t->sys_override);
	if (ret)
		return ret;
	ret = xram_read(EC_SYS_TARGET, &t->sys_target);
	if (ret)
		return ret;
	ret = xram_read(EC_DUTY_CPU, &t->duty[0]);
	if (ret)
		return ret;
	return xram_read(EC_DUTY_SYS, &t->duty[1]);
}

static enum sys_state telemetry_sys_state(const struct telemetry *t)
{
	return classify_sys(t->sys_effective, t->sys_override, t->sys_target);
}

static const struct cpu_profile *find_cpu_profile(u8 selector)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(f7bsi_cpu_profiles); i++)
		if (f7bsi_cpu_profiles[i].selector == selector)
			return &f7bsi_cpu_profiles[i];
	return NULL;
}

/* Fan control: one state machine for both fans */

enum fan_id { FAN_CPU, FAN_SYS, NR_FANS };	/* also the hwmon channel */

enum fan_mode {
	MODE_MANUAL = 1,	/* fixed code written to pwmN, renewed within WATCHDOG_SECONDS */
	MODE_CURVE = 2,		/* this driver's curve */
	MODE_FIRMWARE = 3,	/* the EC firmware's own policy */
};

static const char * const mode_names[] = {
	[MODE_MANUAL] = "manual",
	[MODE_CURVE] = "curve",
	[MODE_FIRMWARE] = "firmware",
};

#define CURVE_MAX_POINTS	8
#define CURVE_FIRMWARE		(-1)

/*
 * A curve is a list of temp:code points in ascending temperature. The point
 * in force is the last one whose temperature has been reached (the first
 * point below that); going down, a point is left once the temperature is more
 * than CURVE_HYST degrees below it. CURVE_FIRMWARE hands the fan to the
 * firmware.
 */
struct curve {
	int n;
	u8 temp[CURVE_MAX_POINTS];
	s8 code[CURVE_MAX_POINTS];
};

struct curve_param {
	const char *fan;
	int min_code;		/* codes 1..min_code-1 are refused */
	struct curve curve;
};

static struct curve_param curve_params[NR_FANS] = {
	[FAN_CPU] = {
		.fan = "cpu",
		.min_code = CPU_MIN_CODE,
		.curve = {
			.n = 5,
			.temp = { 0, 25, 45, 54, 66 },
			.code = { 0, 10, 14, 18, CURVE_FIRMWARE },
		},
	},
	[FAN_SYS] = {
		.fan = "system",
		.min_code = SYS_MIN_CODE,
		.curve = {
			.n = 4,
			.temp = { 0, 25, 40, 50 },
			.code = { 0, 10, 16, CURVE_FIRMWARE },
		},
	},
};
static DEFINE_MUTEX(curve_lock);	/* protects curve_params[].curve */

static int curve_parse(const char *val, const struct curve_param *cp, struct curve *out)
{
	char buf[256], *s, *tok, *colon, *v;
	unsigned int temp, rpm;
	int code;

	if (strscpy(buf, val, sizeof(buf)) < 0)
		return -EINVAL;
	memset(out, 0, sizeof(*out));
	s = strim(buf);
	if (!*s)
		return 0;

	while ((tok = strsep(&s, ",")) != NULL) {
		colon = strchr(tok, ':');
		if (!colon || out->n == CURVE_MAX_POINTS)
			return -EINVAL;
		*colon = '\0';
		if (kstrtouint(strim(tok), 10, &temp) || temp > 120)
			return -EINVAL;
		if (out->n && temp <= out->temp[out->n - 1])
			return -EINVAL;
		v = strim(colon + 1);
		if (!strcmp(v, "firmware") || !strcmp(v, "fw")) {
			code = CURVE_FIRMWARE;
		} else {
			if (kstrtouint(v, 10, &rpm) || rpm > CODE_MAX * 100)
				return -EINVAL;
			code = DIV_ROUND_CLOSEST(rpm, 100);
			/* 1..49 would round to a stop: refuse it rather than stop the fan. */
			if (rpm && !code)
				return -EINVAL;
			if (code > 0 && code < cp->min_code)
				return -EINVAL;
		}
		out->temp[out->n] = temp;
		out->code[out->n] = code;
		out->n++;
	}
	return 0;
}

static int curve_set(const char *val, const struct kernel_param *kp)
{
	struct curve_param *cp = kp->arg;
	struct curve c;
	int ret;

	ret = curve_parse(val, cp, &c);
	if (ret) {
		pr_err("invalid %s_curve; expected temp:rpm or temp:firmware, comma separated, temperatures ascending, at most %d points, rpm 0 or %d..%d\n",
		       cp->fan, CURVE_MAX_POINTS, cp->min_code * 100, CODE_MAX * 100);
		return ret;
	}
	scoped_guard(mutex, &curve_lock)
		cp->curve = c;
	return 0;
}

static int curve_get(char *buffer, const struct kernel_param *kp)
{
	struct curve_param *cp = kp->arg;
	struct curve c;
	int i, len = 0;

	scoped_guard(mutex, &curve_lock)
		c = cp->curve;
	for (i = 0; i < c.n; i++) {
		if (c.code[i] == CURVE_FIRMWARE)
			len += scnprintf(buffer + len, PAGE_SIZE - len, "%s%u:firmware",
					 i ? "," : "", c.temp[i]);
		else
			len += scnprintf(buffer + len, PAGE_SIZE - len, "%s%u:%d",
					 i ? "," : "", c.temp[i], c.code[i] * 100);
	}
	len += scnprintf(buffer + len, PAGE_SIZE - len, "\n");
	return len;
}

static const struct kernel_param_ops curve_ops = {
	.set = curve_set,
	.get = curve_get,
};

module_param_cb(cpu_curve, &curve_ops, &curve_params[FAN_CPU], 0644);
MODULE_PARM_DESC(cpu_curve,
		 "CPU fan curve on the EC CPU temperature, default \"0:0,25:1000,45:1400,54:1800,66:firmware\"; empty leaves the fan to the firmware; rpm 0 or 600..5100");
module_param_cb(system_curve, &curve_ops, &curve_params[FAN_SYS], 0644);
MODULE_PARM_DESC(system_curve,
		 "System fan curve on the EC system temperature, default \"0:0,25:1000,40:1600,50:firmware\"; empty leaves the fan to the firmware; rpm 0 or 800..5100");

struct mec_data;
struct fan;

struct fan_ops {
	/*
	 * Take the fan from the firmware; *start is the code then in force.
	 * *issued is set once a write may have reached the EC, even on failure.
	 */
	int (*engage)(struct mec_data *d, struct fan *f, int rpm, int *start, bool *issued);
	int (*set)(struct mec_data *d, struct fan *f, u8 code);
	/* Give the fan back to the firmware and verify that it has it. */
	int (*release)(struct mec_data *d, struct fan *f);
	/* Is the EC still in exactly the state this driver last wrote? */
	bool (*held)(struct mec_data *d, struct fan *f, const struct telemetry *t);
};

struct fan {
	enum fan_id id;
	const char *name;
	const struct fan_ops *ops;
	int min_code;
	u8 temp_limit;

	enum fan_mode mode;
	int manual_code;
	unsigned long deadline;		/* manual-mode watchdog */

	bool owned;			/* the EC may be following this driver */
	bool release_pending;		/* a release must complete before anything else */
	int cur;			/* last verified code, -1 if none */
	int level;			/* curve point in force */
	bool hot;			/* at or over temp_limit */
	unsigned long next_step;
	unsigned long settle_until;	/* hunting is not judged before this */
	bool holding;			/* stay with the firmware until hold_until */
	unsigned long hold_until;
	int hunts;
	unsigned long last_hunt;
	int rpm_win[HUNT_WINDOW];
	int win_n;
	int engage_failures;		/* consecutive failed engages */
	unsigned int release_failures;	/* consecutive failed releases */
	unsigned long next_release;	/* a pending release is retried from here */
};

struct mec_data {
	struct device *dev;
	struct mutex lock;		/* serialises every Super-I/O sequence and the state below */
	const struct cpu_profile *cpu_profile;
	bool valid;
	bool sys_state_warned;
	unsigned long updated;
	struct telemetry t;
	struct fan fans[NR_FANS];
	bool stopped;			/* unload or shutdown in progress */
	bool suspended;			/* system sleep */
	int read_failures;		/* consecutive telemetry failures in the monitor */
	struct delayed_work monitor;
};

static int mec_update(struct mec_data *d)
{
	struct telemetry t;
	enum sys_state state;
	int ret;

	if (d->valid && time_before(jiffies, d->updated + CACHE_JIFFIES))
		return 0;

	ret = read_telemetry(&t);
	if (ret)
		return ret;

	state = telemetry_sys_state(&t);
	if (!d->fans[FAN_SYS].owned && state != SYS_FIRMWARE && !d->sys_state_warned) {
		dev_warn(d->dev,
			 "system fan left firmware control without this driver: effective 0x%02x, override 0x%02x, target %u (%s)\n",
			 t.sys_effective, t.sys_override, t.sys_target,
			 sys_state_names[state]);
		d->sys_state_warned = true;
	}

	d->t = t;
	d->updated = jiffies;
	d->valid = true;
	return 0;
}

/* CPU table checks (F7bsdProfile.cs:120-221) */

/* Selector, temperature override, bands and critical row: never written here. */
static const char *check_cpu_static(const struct cpu_table *t, const struct cpu_profile *p)
{
	int r;

	if (!p || t->selector != p->selector)
		return "CPU table selector is unknown or changed";
	if (t->temp_override)
		return "CPU temperature override is not zero";
	for (r = 0; r < CPU_ROWS; r++)
		if (t->rows[r][1] != p->bands[r][0] || t->rows[r][2] != p->bands[r][1])
			return "CPU temperature bands do not match the selected table";
	if (memcmp(t->rows[CPU_ROWS], cpu_critical_row, sizeof(cpu_critical_row)) ||
	    t->slopes[CPU_ROWS] != CPU_CRITICAL_SLOPE)
		return "CPU critical row is not (51, 100, 93, 0)";
	return NULL;
}

static bool cpu_is_canonical(const struct cpu_table *t, const struct cpu_profile *p)
{
	int r;

	for (r = 0; r < CPU_ROWS; r++)
		if (t->rows[r][0] != p->bases[r] || t->slopes[r] != p->slopes[r])
			return false;
	return true;
}

/* All seven bases at one code and the slopes at zero: what this driver writes. */
static bool cpu_is_fixed(const struct cpu_table *t, int code)
{
	int r;

	for (r = 0; r < CPU_ROWS; r++)
		if (t->rows[r][0] != code || t->slopes[r])
			return false;
	return true;
}

/*
 * States an interrupted enter, update or restore can leave behind
 * (F7bsdProfile.cs:388-506).
 */
static bool slope_restore_prefix(const u8 *obs, const u8 *canon)
{
	int start, end, i;

	for (start = 0; start <= CPU_ROWS; start++) {
		for (end = start; end <= CPU_ROWS; end++) {
			for (i = 0; i < CPU_ROWS; i++)
				if (obs[i] != (i >= start && i < end ? canon[i] : 0))
					break;
			if (i == CPU_ROWS)
				return true;
		}
	}
	return false;
}

static bool enter_or_restore_prefix(const u8 *obs, const u8 *canon)
{
	int restored, written, i;

	for (restored = 0; restored < CPU_ROWS; restored++) {
		if (memcmp(obs, canon, restored))
			continue;
		for (written = restored + 1; written <= CPU_ROWS; written++) {
			for (i = restored; i < written && obs[i] == obs[restored]; i++)
				;
			if (i == written &&
			    !memcmp(obs + written, canon + written, CPU_ROWS - written))
				return true;
		}
	}
	return false;
}

static bool update_or_restore_prefix(const u8 *obs, const u8 *canon)
{
	int restored, t, i;

	for (restored = 0; restored < CPU_ROWS; restored++) {
		if (memcmp(obs, canon, restored))
			continue;
		for (t = restored; t < CPU_ROWS && obs[t] == obs[restored]; t++)
			;
		if (t == CPU_ROWS)
			return true;
		for (i = t; i < CPU_ROWS && obs[i] == obs[t]; i++)
			;
		if (i == CPU_ROWS)
			return true;
	}
	return false;
}

static bool cpu_recoverable(const struct cpu_table *t, const struct cpu_profile *p)
{
	u8 bases[CPU_ROWS], slopes[CPU_ROWS];
	int r;

	for (r = 0; r < CPU_ROWS; r++) {
		bases[r] = t->rows[r][0];
		slopes[r] = t->slopes[r];
	}
	if (!memcmp(bases, p->bases, CPU_ROWS))
		return slope_restore_prefix(slopes, p->slopes);
	for (r = 0; r < CPU_ROWS; r++)
		if (slopes[r] || bases[r] > CODE_MAX)
			return false;
	return enter_or_restore_prefix(bases, p->bases) ||
	       update_or_restore_prefix(bases, p->bases);
}

/*
 * PNP and controller signatures (PawnIoNative.cs:62-88). The plugin checks
 * them once when it opens the port; this driver also checks before every write.
 */
static int mec_check_controller(struct device *dev)
{
	u8 pnp[3], ident[ARRAY_SIZE(ec_ident_addrs)];
	u8 expected[ARRAY_SIZE(ec_ident_addrs)] = { 0x55, 0x71, 0x00, 0x43, 0x14, 0x7f };
	size_t i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(pnp); i++)
		pnp[i] = pnp_read(SIO_REG_CHIP_ID + i);
	if (pnp[0] != 0x55 || pnp[1] != 0x71) {
		dev_err(dev, "Super-I/O chip ID %*ph is not an IT5571; no EC memory access attempted\n",
			(int)sizeof(pnp), pnp);
		return -ENODEV;
	}

	/* Only the silicon revision may vary, and it must match the PNP byte. */
	expected[2] = pnp[2];
	for (i = 0; i < ARRAY_SIZE(ident); i++) {
		ret = xram_read(ec_ident_addrs[i], &ident[i]);
		if (ret)
			return ret;
	}
	if (memcmp(ident, expected, sizeof(ident))) {
		dev_err(dev, "controller signature %*ph does not match %*ph\n",
			(int)sizeof(ident), ident, (int)sizeof(expected), expected);
		return -ENODEV;
	}
	return 0;
}

/* System-fan control */

static int read_sys_regs(struct sys_regs *r)
{
	int ret;

	ret = xram_read(EC_SYS_EFFECTIVE, &r->effective);
	if (ret)
		return ret;
	ret = xram_read(EC_SYS_OVERRIDE, &r->override);
	if (ret)
		return ret;
	return xram_read(EC_SYS_TARGET, &r->target);
}

static enum sys_state sys_regs_state(const struct sys_regs *r)
{
	return classify_sys(r->effective, r->override, r->target);
}

static int check_sys_policy(struct device *dev)
{
	struct sys_policy pol, tmp;
	int ret;

	ret = read_stable(read_sys_policy, &pol, &tmp, sizeof(pol));
	if (ret) {
		dev_err(dev, "system fan policy is not stable\n");
		return ret;
	}
	if (memcmp(&pol, &f7bsi_sys_policy, sizeof(pol))) {
		dev_err(dev, "system fan policy %*ph does not match f7bsi\n",
			(int)sizeof(pol), &pol);
		return -EIO;
	}
	return 0;
}

/* Expected policy bytes as write preconditions (F7bsdProfile.cs:297-302). */
static size_t sys_policy_expectations(struct ec_byte *out)
{
	const u8 *rows = &f7bsi_sys_policy.rows[0][0];
	size_t i, n = 0;

	for (i = 0; i < sizeof(f7bsi_sys_policy.rows); i++)
		out[n++] = (struct ec_byte){ EC_SYS_POLICY_ROWS + i, rows[i] };
	for (i = 0; i < sizeof(f7bsi_sys_policy.tail); i++)
		out[n++] = (struct ec_byte){ EC_SYS_POLICY_TAIL + i, f7bsi_sys_policy.tail[i] };
	return n;
}

/*
 * Write allowlist (F7bsdProfile.cs:346-386): the system override (0 or 0xff),
 * the system target and the seven CPU bases (0..51), and the seven CPU slopes
 * (0 or their canonical value).
 */
static bool xram_writable(const struct mec_data *d, u16 addr, u8 val)
{
	int r;

	if (addr == EC_SYS_TARGET)
		return val <= CODE_MAX;
	if (addr == EC_SYS_OVERRIDE)
		return val == 0 || val == SYS_SENTINEL;
	for (r = 0; r < CPU_ROWS; r++) {
		if (addr == EC_CPU_ROWS + 3 * r)
			return val <= CODE_MAX;
		if (addr == EC_CPU_SLOPES + r)
			return val == 0 || (d->cpu_profile && val == d->cpu_profile->slopes[r]);
	}
	return false;
}

/*
 * Check the controller signature and every precondition, then write each
 * byte and read it back (PawnIoNative.cs:125-157). *issued, if given, is set
 * just before the first write, like the plugin's beforeWrites callback: from
 * there on a failure may leave the EC following this driver.
 */
static int xram_write_verified(struct mec_data *d,
			       const struct ec_byte *before, size_t n_before,
			       const struct ec_byte *writes, size_t n_writes,
			       bool *issued)
{
	struct device *dev = d->dev;
	size_t i;
	u8 val;
	int ret;

	for (i = 0; i < n_writes; i++)
		if (WARN_ONCE(!xram_writable(d, writes[i].addr, writes[i].val),
			      DRVNAME ": EC write 0x%04x=0x%02x is not allowed\n",
			      writes[i].addr, writes[i].val))
			return -EPERM;

	ret = mec_check_controller(dev);
	if (ret)
		return ret;

	for (i = 0; i < n_before; i++) {
		ret = xram_read(before[i].addr, &val);
		if (ret)
			return ret;
		if (val != before[i].val) {
			dev_err(dev, "EC precondition failed at 0x%04x: expected 0x%02x, read 0x%02x\n",
				before[i].addr, before[i].val, val);
			return -EIO;
		}
	}

	if (issued)
		*issued = true;
	for (i = 0; i < n_writes; i++) {
		i2ec_write(writes[i].addr, writes[i].val);
		ret = xram_read(writes[i].addr, &val);
		if (ret)
			return ret;
		if (val != writes[i].val) {
			dev_err(dev, "EC verification failed at 0x%04x: wrote 0x%02x, read 0x%02x\n",
				writes[i].addr, writes[i].val, val);
			return -EIO;
		}
	}
	return 0;
}

/* System fan: the 0xff fixed-target handoff */

/* PawnIoF7bsdBackend.cs:520-544 */
static int sys_wait_effective(struct device *dev, bool owned)
{
	u8 last = 0;
	int i, ret;

	for (i = 0; i < SYS_HANDOFF_POLLS; i++) {
		ret = xram_read(EC_SYS_EFFECTIVE, &last);
		if (ret)
			return ret;
		if (owned ? last == SYS_SENTINEL : plausible_temp(last))
			return 0;
		if (i + 1 < SYS_HANDOFF_POLLS)
			msleep(SYS_HANDOFF_POLL_MS);
	}
	dev_err(dev, "system fan did not %s; effective byte ended at 0x%02x\n",
		owned ? "enter fixed-target mode" : "return to firmware mode", last);
	return -EIO;
}

/*
 * Seed the maximum target, clear the override and wait until the firmware
 * owns the fan again (PawnIoF7bsdBackend.cs:427-518).
 */
static int sys_release_core(struct mec_data *d)
{
	struct device *dev = d->dev;
	struct ec_byte before[2], write;
	struct sys_regs r, failed = { };
	bool write_failed = false;
	int step, ret;

	for (step = 0; step < SYS_RELEASE_STEPS; step++) {
		ret = read_sys_regs(&r);
		if (ret)
			return ret;
		if (write_failed && r.override == failed.override && r.target == failed.target)
			return -EIO;
		write_failed = false;

		switch (sys_regs_state(&r)) {
		case SYS_FIRMWARE:
			return 0;
		case SYS_RELEASING:
			ret = sys_wait_effective(dev, false);
			if (ret)
				return ret;
			ret = read_sys_regs(&r);
			if (ret)
				return ret;
			if (sys_regs_state(&r) != SYS_FIRMWARE) {
				dev_err(dev, "firmware did not resume system fan control\n");
				return -EIO;
			}
			return 0;
		case SYS_FIXED:
			before[0] = (struct ec_byte){ EC_SYS_OVERRIDE, SYS_SENTINEL };
			before[1] = (struct ec_byte){ EC_SYS_TARGET, r.target };
			if (r.target == CODE_MAX)
				write = (struct ec_byte){ EC_SYS_OVERRIDE, 0 };
			else
				write = (struct ec_byte){ EC_SYS_TARGET, CODE_MAX };
			if (xram_write_verified(d, before, ARRAY_SIZE(before), &write, 1, NULL)) {
				write_failed = true;
				failed = r;
			}
			break;
		default:
			dev_err(dev, "system fan state is unsupported (effective 0x%02x, override 0x%02x, target %u); no recovery write attempted\n",
				r.effective, r.override, r.target);
			return -EIO;
		}
	}
	dev_err(dev, "system fan release did not reach firmware state\n");
	return -EIO;
}

/*
 * Set the override, keeping the firmware's target (PawnIoF7bsdBackend.cs:392-415).
 * In firmware mode the effective byte follows a temperature and the target
 * follows the firmware's bands, so a precondition that fails before any write
 * is tried once more. The code in force is read back once the firmware has let
 * go of the target.
 */
static int sys_engage(struct mec_data *d, struct fan *f, int rpm, int *start, bool *issued)
{
	struct device *dev = d->dev;
	struct ec_byte before[SYS_POLICY_BYTES + 3];
	struct ec_byte write = { EC_SYS_OVERRIDE, SYS_SENTINEL };
	struct sys_regs r;
	int attempt, ret;
	size_t n;
	u8 target;

	ret = check_sys_policy(dev);
	if (ret)
		return ret;

	for (attempt = 0; attempt < 2; attempt++) {
		ret = read_sys_regs(&r);
		if (ret)
			return ret;
		if (sys_regs_state(&r) != SYS_FIRMWARE) {
			dev_err(dev, "system fan is not under firmware control: effective 0x%02x, override 0x%02x, target %u\n",
				r.effective, r.override, r.target);
			return -EIO;
		}

		n = sys_policy_expectations(before);
		before[n++] = (struct ec_byte){ EC_SYS_EFFECTIVE, r.effective };
		before[n++] = (struct ec_byte){ EC_SYS_OVERRIDE, r.override };
		before[n++] = (struct ec_byte){ EC_SYS_TARGET, r.target };

		ret = xram_write_verified(d, before, n, &write, 1, issued);
		if (!ret || *issued)
			break;
	}
	if (ret)
		return ret;

	ret = sys_wait_effective(dev, true);
	if (ret)
		return ret;
	ret = xram_read(EC_SYS_TARGET, &target);
	if (ret)
		return ret;
	if (target > CODE_MAX) {
		dev_err(dev, "system fan target %u after the handoff\n", target);
		return -EIO;
	}
	*start = target;
	return 0;
}

/* PawnIoF7bsdBackend.cs:316-390 */
static int sys_set(struct mec_data *d, struct fan *f, u8 code)
{
	struct device *dev = d->dev;
	struct ec_byte before[SYS_POLICY_BYTES + 3];
	struct ec_byte write = { EC_SYS_TARGET, code };
	struct sys_regs r;
	size_t n;
	int ret;

	ret = check_sys_policy(dev);
	if (ret)
		return ret;
	n = sys_policy_expectations(before);
	before[n++] = (struct ec_byte){ EC_SYS_EFFECTIVE, SYS_SENTINEL };
	before[n++] = (struct ec_byte){ EC_SYS_OVERRIDE, SYS_SENTINEL };
	before[n++] = (struct ec_byte){ EC_SYS_TARGET, f->cur };
	ret = xram_write_verified(d, before, n, &write, 1, NULL);
	if (ret)
		return ret;
	ret = read_sys_regs(&r);
	if (ret)
		return ret;
	if (r.effective != SYS_SENTINEL || r.override != SYS_SENTINEL || r.target != code) {
		dev_err(dev, "fixed-target state not confirmed after writing %u\n", code);
		return -EIO;
	}
	return 0;
}

static int sys_release(struct mec_data *d, struct fan *f)
{
	return sys_release_core(d);
}

static bool sys_held(struct mec_data *d, struct fan *f, const struct telemetry *t)
{
	return t->sys_effective == SYS_SENTINEL && t->sys_override == SYS_SENTINEL &&
	       t->sys_target == f->cur;
}

static const struct fan_ops sys_fan_ops = {
	.engage = sys_engage,
	.set = sys_set,
	.release = sys_release,
	.held = sys_held,
};

/* CPU fan: the same code in the seven normal rows of the BIOS-selected table */

#define CPU_SNAPSHOT_BYTES	(2 + sizeof_field(struct cpu_table, rows) + \
				 sizeof_field(struct cpu_table, slopes))

/* The whole snapshot as write preconditions (PawnIoF7bsdBackend.cs:227-232). */
static size_t cpu_expectations(const struct cpu_table *t, struct ec_byte *out)
{
	const u8 *rows = &t->rows[0][0];
	size_t i, n = 0;

	out[n++] = (struct ec_byte){ EC_CPU_SELECTOR, t->selector };
	out[n++] = (struct ec_byte){ EC_CPU_TEMP_OVERRIDE, t->temp_override };
	for (i = 0; i < sizeof(t->rows); i++)
		out[n++] = (struct ec_byte){ EC_CPU_ROWS + i, rows[i] };
	for (i = 0; i < sizeof(t->slopes); i++)
		out[n++] = (struct ec_byte){ EC_CPU_SLOPES + i, t->slopes[i] };
	return n;
}

static int read_cpu_stable(struct device *dev, struct cpu_table *t)
{
	struct cpu_table tmp;
	int ret;

	ret = read_stable(read_cpu_table, t, &tmp, sizeof(*t));
	if (ret)
		dev_err(dev, "CPU fan table is not stable\n");
	return ret;
}

/* Slopes to zero, then every base to the code (F7bsdProfile.cs:306-317). */
static int cpu_engage(struct mec_data *d, struct fan *f, int rpm, int *start, bool *issued)
{
	struct ec_byte before[CPU_SNAPSHOT_BYTES], writes[2 * CPU_ROWS];
	struct cpu_table t;
	const char *problem;
	int code, r, n = 0, ret;

	code = clamp(DIV_ROUND_CLOSEST(rpm, 100), 0, CODE_MAX);
	if (code > 0 && code < f->min_code)
		code = f->min_code;

	ret = read_cpu_stable(d->dev, &t);
	if (ret)
		return ret;
	problem = check_cpu_static(&t, d->cpu_profile);
	if (!problem && !cpu_is_canonical(&t, d->cpu_profile))
		problem = "CPU table is not the canonical firmware table";
	if (problem) {
		dev_err(d->dev, "%s; CPU fan left to the firmware\n", problem);
		return -EIO;
	}

	for (r = 0; r < CPU_ROWS; r++)
		writes[n++] = (struct ec_byte){ EC_CPU_SLOPES + r, 0 };
	for (r = 0; r < CPU_ROWS; r++)
		writes[n++] = (struct ec_byte){ EC_CPU_ROWS + 3 * r, code };
	ret = xram_write_verified(d, before, cpu_expectations(&t, before), writes, n, issued);
	if (!ret)
		*start = code;
	return ret;
}

/* PawnIoF7bsdBackend.cs:203-252 */
static int cpu_set(struct mec_data *d, struct fan *f, u8 code)
{
	struct ec_byte before[CPU_SNAPSHOT_BYTES], writes[CPU_ROWS];
	struct cpu_table t;
	const char *problem;
	int r, ret;

	ret = read_cpu_stable(d->dev, &t);
	if (ret)
		return ret;
	problem = check_cpu_static(&t, d->cpu_profile);
	if (!problem && !cpu_is_fixed(&t, f->cur))
		problem = "CPU table is not the state last written by this driver";
	if (problem) {
		dev_err(d->dev, "%s\n", problem);
		return -EIO;
	}

	for (r = 0; r < CPU_ROWS; r++)
		writes[r] = (struct ec_byte){ EC_CPU_ROWS + 3 * r, code };
	return xram_write_verified(d, before, cpu_expectations(&t, before), writes, CPU_ROWS, NULL);
}

/*
 * Slopes to zero, canonical bases, canonical slopes, then verify the whole
 * table (F7bsdProfile.cs:319-332, PawnIoF7bsdBackend.cs:264-314).
 */
static int cpu_release(struct mec_data *d, struct fan *f)
{
	const struct cpu_profile *p = d->cpu_profile;
	struct ec_byte before[CPU_SNAPSHOT_BYTES], writes[3 * CPU_ROWS];
	struct cpu_table t;
	const char *problem;
	int r, n = 0, ret;

	ret = read_cpu_stable(d->dev, &t);
	if (ret)
		return ret;
	problem = check_cpu_static(&t, p);
	if (problem) {
		dev_err(d->dev, "%s; no CPU restore write attempted\n", problem);
		return -EIO;
	}
	if (cpu_is_canonical(&t, p))
		return 0;
	if (!cpu_recoverable(&t, p)) {
		dev_err(d->dev, "CPU table is in a state this driver cannot produce; no restore write attempted\n");
		return -EIO;
	}

	for (r = 0; r < CPU_ROWS; r++)
		writes[n++] = (struct ec_byte){ EC_CPU_SLOPES + r, 0 };
	for (r = 0; r < CPU_ROWS; r++)
		writes[n++] = (struct ec_byte){ EC_CPU_ROWS + 3 * r, p->bases[r] };
	for (r = 0; r < CPU_ROWS; r++)
		writes[n++] = (struct ec_byte){ EC_CPU_SLOPES + r, p->slopes[r] };
	xram_write_verified(d, before, cpu_expectations(&t, before), writes, n, NULL);

	ret = read_cpu_stable(d->dev, &t);
	if (ret)
		return ret;
	if (check_cpu_static(&t, p) || !cpu_is_canonical(&t, p)) {
		dev_err(d->dev, "the canonical CPU table was not restored\n");
		return -EIO;
	}
	return 0;
}

/* A mismatch is read once more before it counts: one bad snapshot is not trusted. */
static bool cpu_held(struct mec_data *d, struct fan *f, const struct telemetry *unused)
{
	struct cpu_table t;
	int attempt;

	for (attempt = 0; attempt < 2; attempt++)
		if (!read_cpu_table(&t) && !check_cpu_static(&t, d->cpu_profile) &&
		    cpu_is_fixed(&t, f->cur))
			return true;
	return false;
}

static const struct fan_ops cpu_fan_ops = {
	.engage = cpu_engage,
	.set = cpu_set,
	.release = cpu_release,
	.held = cpu_held,
};

/* Generic fan logic */

static int fan_release(struct mec_data *d, struct fan *f, const char *reason)
{
	unsigned long delay;
	int ret;

	if (!f->owned && !f->release_pending)
		return 0;
	f->release_pending = true;
	d->valid = false;
	ret = f->ops->release(d, f);
	if (ret) {
		/* Back off, so that an EC state that cannot be released does not flood the log. */
		delay = MONITOR_JIFFIES << min(f->release_failures, RELEASE_RETRY_SHIFT);
		f->release_failures++;
		f->next_release = jiffies + delay;
		dev_err(d->dev, "%s fan: returning it to firmware failed %u times (%d, %s); next retry in %lu s\n",
			f->name, f->release_failures, ret, reason, delay / HZ);
		return ret;
	}
	f->release_failures = 0;
	f->owned = false;
	f->release_pending = false;
	f->cur = -1;
	f->win_n = 0;
	dev_info(d->dev, "%s fan returned to firmware: %s\n", f->name, reason);
	return 0;
}

static void fan_hold(struct fan *f, unsigned long duration)
{
	f->holding = true;
	f->hold_until = jiffies + duration;
}

static int fan_engage(struct mec_data *d, struct fan *f, int rpm)
{
	bool issued = false;
	int start = 0, ret;

	d->valid = false;
	ret = f->ops->engage(d, f, rpm, &start, &issued);
	if (ret) {
		if (issued) {
			/* A write may have reached the EC: release until that succeeds. */
			f->owned = true;
			fan_release(d, f, "taking control failed");
		} else if (!f->ops->release(d, f)) {
			/*
			 * Nothing was written, but a state another EC writer left
			 * (a repairable CPU table, a fixed-target handoff) is handed
			 * back now. If that fails it is not this driver's to retry:
			 * the next engage tries again after the hold.
			 */
			dev_info(d->dev, "%s fan left with the firmware: taking control failed\n",
				 f->name);
		}
		return ret;
	}
	f->owned = true;
	f->cur = start;
	f->next_step = jiffies + STEP_JIFFIES;
	f->settle_until = jiffies + SETTLE_JIFFIES;
	f->win_n = 0;
	dev_info(d->dev, "%s fan under driver control (%s), starting at %d RPM\n",
		 f->name, mode_names[f->mode], start * 100);
	return 0;
}

static int fan_set(struct mec_data *d, struct fan *f, int code)
{
	int ret;

	d->valid = false;
	ret = f->ops->set(d, f, code);
	if (ret) {
		fan_release(d, f, "target write failed");
		return ret;
	}
	f->cur = code;
	f->settle_until = jiffies + SETTLE_JIFFIES;
	f->win_n = 0;
	return 0;
}

/*
 * One code at a time towards the goal: a jump makes the EC's speed loop hunt
 * (docs/fase2.md). Codes 1..min_code-1 are skipped; a stopped fan restarts at
 * SPINUP_CODE and steps from there towards the goal.
 */
static int fan_next_code(const struct fan *f, int goal)
{
	int next;

	if (f->cur == 0)
		return SPINUP_CODE;
	next = f->cur + (goal > f->cur ? 1 : -1);
	if (next > 0 && next < f->min_code)
		next = goal > f->cur ? f->min_code : 0;
	return next;
}

static int curve_goal(struct mec_data *d, struct fan *f, const struct curve *c, u8 temp)
{
	int level = clamp(f->level, 0, c->n - 1);

	while (level + 1 < c->n && temp >= c->temp[level + 1])
		level++;
	while (level > 0 && temp + CURVE_HYST < c->temp[level])
		level--;
	if (level != f->level) {
		if (c->code[level] == CURVE_FIRMWARE)
			dev_info(d->dev, "%s fan: %u C, curve point %u C: firmware\n",
				 f->name, temp, c->temp[level]);
		else
			dev_info(d->dev, "%s fan: %u C, curve point %u C: %d RPM\n",
				 f->name, temp, c->temp[level], c->code[level] * 100);
		f->level = level;
	}
	return c->code[level];
}

/* A steady target whose RPM still swings by HUNT_SPREAD means the EC loop is hunting. */
static void fan_check_hunting(struct mec_data *d, struct fan *f, int rpm)
{
	int i, lo = INT_MAX, hi = 0;

	if (f->hunts && time_after(jiffies, f->last_hunt + HUNT_FORGET))
		f->hunts = 0;
	if (time_before(jiffies, f->settle_until))
		return;

	f->rpm_win[f->win_n % HUNT_WINDOW] = rpm;
	if (++f->win_n < HUNT_WINDOW)
		return;
	for (i = 0; i < HUNT_WINDOW; i++) {
		lo = min(lo, f->rpm_win[i]);
		hi = max(hi, f->rpm_win[i]);
	}
	if (hi - lo <= HUNT_SPREAD)
		return;

	f->hunts++;
	f->last_hunt = jiffies;
	dev_warn(d->dev, "%s fan hunting at %d RPM (measured %d..%d)\n",
		 f->name, f->cur * 100, lo, hi);
	fan_release(d, f, "hunting");
	if (f->hunts >= MAX_HUNTS) {
		f->mode = MODE_FIRMWARE;
		f->hunts = 0;
		dev_warn(d->dev, "%s fan hunted %d times; left to the firmware until pwm%d_enable is set again\n",
			 f->name, MAX_HUNTS, f->id + 1);
	} else {
		fan_hold(f, HUNT_BACKOFF);
	}
}

static void fan_tick(struct mec_data *d, struct fan *f, const struct curve *c,
		     const struct telemetry *t)
{
	u8 temp = t->temp[f->id];
	int rpm = t->rpm[f->id];
	const char *why;
	int goal;

	if (f->release_pending) {
		if (!time_before(jiffies, f->next_release))
			fan_release(d, f, "retry");
		return;
	}

	if (!f->hot && (temp >= f->temp_limit || !plausible_temp(temp))) {
		f->hot = true;
		dev_warn(d->dev, "%s fan: EC temperature %u C, limit %u C\n",
			 f->name, temp, f->temp_limit);
	} else if (f->hot && plausible_temp(temp) && temp + LIMIT_HYST < f->temp_limit) {
		f->hot = false;
		dev_info(d->dev, "%s fan: EC temperature back to %u C\n", f->name, temp);
	}

	if (f->mode == MODE_MANUAL && time_after(jiffies, f->deadline)) {
		f->mode = c->n ? MODE_CURVE : MODE_FIRMWARE;
		dev_info(d->dev, "%s fan: no pwm%d write for %d s, back to %s\n",
			 f->name, f->id + 1, WATCHDOG_SECONDS, mode_names[f->mode]);
	}
	if (f->mode == MODE_CURVE && !c->n)
		f->mode = MODE_FIRMWARE;
	if (f->holding && !time_before(jiffies, f->hold_until))
		f->holding = false;

	if (f->hot)
		why = "temperature limit";
	else if (f->mode == MODE_FIRMWARE)
		why = "firmware mode";
	else if (f->holding)
		why = "backing off";
	else
		why = NULL;

	if (why)
		goal = CURVE_FIRMWARE;
	else if (f->mode == MODE_MANUAL)
		goal = f->manual_code;
	else
		goal = curve_goal(d, f, c, temp);

	if (goal == CURVE_FIRMWARE) {
		if (f->owned)
			fan_release(d, f, why ?: "curve");
		return;
	}

	if (!f->owned) {
		if (!fan_engage(d, f, rpm)) {
			f->engage_failures = 0;
		} else if (++f->engage_failures >= MAX_ENGAGE_FAILURES) {
			f->mode = MODE_FIRMWARE;
			f->engage_failures = 0;
			dev_warn(d->dev, "%s fan: taking control failed %d times; left to the firmware until pwm%d_enable is set again\n",
				 f->name, MAX_ENGAGE_FAILURES, f->id + 1);
		} else {
			fan_hold(f, HUNT_BACKOFF);
		}
		return;
	}

	if (!f->ops->held(d, f, t)) {
		dev_err(d->dev, "%s fan: EC state changed under driver control\n", f->name);
		fan_release(d, f, "unexpected state");
		fan_hold(f, HUNT_BACKOFF);
		return;
	}

	if (f->cur != goal) {
		if (time_before(jiffies, f->next_step))
			return;
		if (fan_set(d, f, fan_next_code(f, goal)))
			fan_hold(f, HUNT_BACKOFF);
		else
			f->next_step = jiffies + STEP_JIFFIES;
		return;
	}

	if (goal > 0)
		fan_check_hunting(d, f, rpm);
}

static void mec_monitor(struct work_struct *work)
{
	struct mec_data *d = container_of(to_delayed_work(work), struct mec_data, monitor);
	struct curve curves[NR_FANS];
	struct telemetry t;
	int i;

	scoped_guard(mutex, &curve_lock)
		for (i = 0; i < NR_FANS; i++)
			curves[i] = curve_params[i].curve;

	guard(mutex)(&d->lock);
	if (d->stopped || d->suspended)
		return;

	if (!read_telemetry(&t)) {
		d->read_failures = 0;
		d->t = t;
		d->updated = jiffies;
		d->valid = true;
		for (i = 0; i < NR_FANS; i++)
			fan_tick(d, &d->fans[i], &curves[i], &t);
	} else if (++d->read_failures >= MAX_READ_FAILURES) {
		/* Without temperatures and RPM nothing can be judged: let the firmware decide. */
		for (i = 0; i < NR_FANS; i++) {
			struct fan *f = &d->fans[i];

			if (f->release_pending && time_before(jiffies, f->next_release))
				continue;
			if (f->owned || f->release_pending) {
				fan_release(d, f, "EC telemetry unreadable");
				fan_hold(f, HUNT_BACKOFF);
			}
		}
	}
	queue_delayed_work(system_dfl_wq, &d->monitor, MONITOR_JIFFIES);
}

#define STOP_RELEASE_ATTEMPTS	3

static void mec_stop(struct mec_data *d, const char *reason)
{
	int i, attempt;

	scoped_guard(mutex, &d->lock)
		d->stopped = true;
	cancel_delayed_work_sync(&d->monitor);

	guard(mutex)(&d->lock);
	for (i = 0; i < NR_FANS; i++) {
		for (attempt = 0; attempt < STOP_RELEASE_ATTEMPTS; attempt++) {
			if (!fan_release(d, &d->fans[i], reason))
				break;
			msleep(200);
		}
		if (attempt == STOP_RELEASE_ATTEMPTS)
			dev_crit(d->dev, "%s fan may still be under driver control; power the machine off completely before using the EC again\n",
				 d->fans[i].name);
	}
}

/* hwmon interface */

static const char * const fan_labels[] = { "CPU", "System" };
static const char * const temp_labels[] = { "EC CPU", "EC System" };

static umode_t mec_is_visible(const void *drvdata, enum hwmon_sensor_types type,
			      u32 attr, int channel)
{
	if (type == hwmon_pwm)
		return 0644;
	return 0444;
}

static int mec_read(struct device *dev, enum hwmon_sensor_types type,
		    u32 attr, int channel, long *val)
{
	struct mec_data *d = dev_get_drvdata(dev);
	struct fan *f;
	int ret;

	guard(mutex)(&d->lock);
	ret = mec_update(d);
	if (ret)
		return ret;

	switch (type) {
	case hwmon_fan:
		*val = d->t.rpm[channel];
		return 0;
	case hwmon_temp:
		*val = d->t.temp[channel] * 1000;
		return 0;
	case hwmon_pwm:
		f = &d->fans[channel];
		switch (attr) {
		case hwmon_pwm_input:
			/* EC target code 0..51 (nominally code * 100 RPM), as hwmon's 0..255. */
			if (f->owned && f->cur >= 0)
				*val = f->cur * PWM_PER_CODE;
			else if (channel == FAN_SYS)
				*val = min_t(int, d->t.sys_target, CODE_MAX) * PWM_PER_CODE;
			else
				return -ENODATA;
			return 0;
		case hwmon_pwm_enable:
			*val = f->mode;
			return 0;
		default:
			break;
		}
		break;
	default:
		break;
	}
	return -EOPNOTSUPP;
}

static bool code_valid(const struct fan *f, long val)
{
	return val == 0 || (val >= f->min_code && val <= CODE_MAX);
}

/*
 * pwmN_enable: 1 fixed value (starting at the current speed), 2 curve,
 * 3 firmware. pwmN: the fixed value, 0..255 in steps of PWM_PER_CODE (one EC
 * code), accepted in mode 1 only; each write restarts the watchdog, after
 * which the fan returns to its curve.
 */
static int mec_write(struct device *dev, enum hwmon_sensor_types type,
		     u32 attr, int channel, long val)
{
	struct mec_data *d = dev_get_drvdata(dev);
	struct fan *f;
	int code, ret;
	bool has_curve;

	if (type != hwmon_pwm)
		return -EOPNOTSUPP;
	f = &d->fans[channel];

	scoped_guard(mutex, &curve_lock)
		has_curve = curve_params[channel].curve.n > 0;

	guard(mutex)(&d->lock);
	if (d->stopped)
		return -ENODEV;
	if (d->suspended)
		return -EBUSY;

	switch (attr) {
	case hwmon_pwm_input:
		if (f->mode != MODE_MANUAL)
			return -EBUSY;
		if (val < 0 || val > CODE_MAX * PWM_PER_CODE)
			return -EINVAL;
		code = DIV_ROUND_CLOSEST(val, PWM_PER_CODE);
		/* 1..2 would round to a stop: refuse it rather than stop the fan. */
		if ((val && !code) || !code_valid(f, code))
			return -EINVAL;
		f->manual_code = code;
		break;
	case hwmon_pwm_enable:
		if (val == MODE_MANUAL) {
			ret = mec_update(d);
			if (ret)
				return ret;
			if (f->mode != MODE_MANUAL) {
				if (f->owned && f->cur >= 0)
					code = f->cur;
				else if (channel == FAN_SYS)
					code = d->t.sys_target;
				else
					code = DIV_ROUND_CLOSEST(d->t.rpm[channel], 100);
				code = clamp(code, 0, CODE_MAX);
				if (!code_valid(f, code))
					code = f->min_code;
				f->manual_code = code;
			}
		} else if (val == MODE_CURVE) {
			if (!has_curve)
				return -EINVAL;
		} else if (val != MODE_FIRMWARE) {
			return -EINVAL;
		}
		if (val != f->mode)
			dev_info(d->dev, "%s fan: mode %s\n", f->name, mode_names[val]);
		f->mode = val;
		f->holding = false;
		f->hunts = 0;
		f->engage_failures = 0;
		f->next_release = jiffies;	/* a pending release is retried now */
		break;
	default:
		return -EOPNOTSUPP;
	}
	f->deadline = jiffies + WATCHDOG_SECONDS * HZ;
	mod_delayed_work(system_dfl_wq, &d->monitor, 0);
	return 0;
}

static int mec_read_string(struct device *dev, enum hwmon_sensor_types type,
			   u32 attr, int channel, const char **str)
{
	switch (type) {
	case hwmon_fan:
		*str = fan_labels[channel];
		return 0;
	case hwmon_temp:
		*str = temp_labels[channel];
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static const struct hwmon_ops mec_hwmon_ops = {
	.is_visible = mec_is_visible,
	.read = mec_read,
	.read_string = mec_read_string,
	.write = mec_write,
};

static const struct hwmon_channel_info * const mec_info[] = {
	HWMON_CHANNEL_INFO(fan,
			   HWMON_F_INPUT | HWMON_F_LABEL,
			   HWMON_F_INPUT | HWMON_F_LABEL),
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_LABEL),
	HWMON_CHANNEL_INFO(pwm,
			   HWMON_PWM_INPUT | HWMON_PWM_ENABLE,
			   HWMON_PWM_INPUT | HWMON_PWM_ENABLE),
	NULL
};

static const struct hwmon_chip_info mec_chip_info = {
	.ops = &mec_hwmon_ops,
	.info = mec_info,
};

/* debugfs: a fresh, uncached dump of everything the driver can read. */

static int state_show(struct seq_file *s, void *unused)
{
	struct mec_data *d = s->private;
	struct cpu_table cpu, cpu_tmp;
	struct sys_policy pol, pol_tmp;
	const struct cpu_profile *profile;
	const char *cpu_problem;
	struct telemetry t;
	u8 pnp[3], ident[ARRAY_SIZE(ec_ident_addrs)];
	struct fan *f;
	size_t i;
	int r, ret;

	guard(mutex)(&d->lock);

	for (i = 0; i < ARRAY_SIZE(pnp); i++)
		pnp[i] = pnp_read(SIO_REG_CHIP_ID + i);
	for (i = 0; i < ARRAY_SIZE(ident); i++) {
		ret = xram_read(ec_ident_addrs[i], &ident[i]);
		if (ret)
			return ret;
	}
	seq_printf(s, "pnp:          %*ph\n", (int)sizeof(pnp), pnp);
	seq_printf(s, "controller:   %*ph\n", (int)sizeof(ident), ident);

	ret = read_stable(read_cpu_table, &cpu, &cpu_tmp, sizeof(cpu));
	if (ret) {
		seq_puts(s, "cpu table:    no stable snapshot\n");
	} else {
		profile = find_cpu_profile(cpu.selector);
		cpu_problem = check_cpu_static(&cpu, profile);
		if (!cpu_problem && cpu_is_fixed(&cpu, d->fans[FAN_CPU].cur) && d->fans[FAN_CPU].owned)
			cpu_problem = "held by this driver";
		else if (!cpu_problem && !cpu_is_canonical(&cpu, profile))
			cpu_problem = "NOT canonical";
		seq_printf(s, "cpu table:    selector 0x%02x (%s), temp override 0x%02x, %s\n",
			   cpu.selector, profile ? profile->name : "?",
			   cpu.temp_override, cpu_problem ?: "canonical");
		seq_puts(s, "  row  base upper lower slope\n");
		for (r = 0; r <= CPU_ROWS; r++)
			seq_printf(s, "  %3d  %4u %5u %5u %5u%s\n", r,
				   cpu.rows[r][0], cpu.rows[r][1], cpu.rows[r][2],
				   cpu.slopes[r], r == CPU_ROWS ? "  critical" : "");
	}

	ret = read_stable(read_sys_policy, &pol, &pol_tmp, sizeof(pol));
	if (ret)
		seq_puts(s, "sys policy:   no stable snapshot\n");
	else
		seq_printf(s, "sys policy:   %*ph (%s)\n", (int)sizeof(pol), &pol,
			   memcmp(&pol, &f7bsi_sys_policy, sizeof(pol)) ?
			   "MISMATCH" : "matches f7bsi");

	ret = read_telemetry(&t);
	if (ret) {
		seq_printf(s, "telemetry:    read failed (%d)\n", ret);
		return 0;
	}
	seq_printf(s, "sys state:    effective 0x%02x, override 0x%02x, target %u -> %s\n",
		   t.sys_effective, t.sys_override, t.sys_target,
		   sys_state_names[telemetry_sys_state(&t)]);
	seq_printf(s, "fan rpm:      cpu %d, sys %d\n", t.rpm[0], t.rpm[1]);
	seq_printf(s, "pwm duty:     cpu (0x%04x) %u, sys (0x%04x) %u\n",
		   EC_DUTY_CPU, t.duty[0], EC_DUTY_SYS, t.duty[1]);
	seq_printf(s, "ec temp:      cpu %u C, sys %u C\n", t.temp[0], t.temp[1]);

	for (i = 0; i < NR_FANS; i++) {
		f = &d->fans[i];
		seq_printf(s, "%-6s fan:   mode %s, %s, code %d, curve point %d%s%s%s, hunts %d\n",
			   f->name, mode_names[f->mode],
			   f->release_pending ? "RELEASE PENDING" :
			   f->owned ? "driver control" : "firmware control",
			   f->cur, f->level, f->hot ? ", HOT" : "",
			   f->holding ? ", backing off" : "",
			   f->mode == MODE_MANUAL ? ", manual" : "", f->hunts);
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(state);

static void mec_debugfs_remove(void *dir)
{
	debugfs_remove_recursive(dir);
}

static void fan_init(struct fan *f, enum fan_id id, const char *name,
		     const struct fan_ops *ops, u8 temp_limit)
{
	f->id = id;
	f->name = name;
	f->ops = ops;
	f->min_code = curve_params[id].min_code;
	f->temp_limit = temp_limit;
	f->mode = MODE_FIRMWARE;
	f->cur = -1;
}

/*
 * Admission checks that touch the hardware: signatures, CPU table, exact
 * system policy, and both fans back with the firmware. A state left by an
 * earlier session that this driver can produce is repaired first
 * (PawnIoF7bsdBackend.cs:49-126); anything else refuses the load.
 */
static int mec_admit(struct mec_data *d, struct telemetry *t)
{
	struct device *dev = d->dev;
	struct cpu_table cpu, cpu_tmp;
	const struct cpu_profile *p;
	enum sys_state state;
	struct sys_regs r;
	const char *problem;
	bool cpu_repair;
	int ret;

	guard(mutex)(&d->lock);

	/* Every check first, no write until all of them pass. */
	ret = mec_check_controller(dev);
	if (ret)
		return ret;

	ret = read_stable(read_cpu_table, &cpu, &cpu_tmp, sizeof(cpu));
	if (ret)
		return dev_err_probe(dev, ret, "CPU fan table is not stable\n");
	p = find_cpu_profile(cpu.selector);
	problem = check_cpu_static(&cpu, p);
	if (problem)
		return dev_err_probe(dev, -ENODEV, "%s (selector 0x%02x)\n",
				     problem, cpu.selector);
	cpu_repair = !cpu_is_canonical(&cpu, p);
	if (cpu_repair && !cpu_recoverable(&cpu, p))
		return dev_err_probe(dev, -ENODEV,
				     "CPU table is neither canonical nor a state this driver can repair\n");
	d->cpu_profile = p;

	ret = check_sys_policy(dev);
	if (ret)
		return -ENODEV;

	/*
	 * Only the three state bytes (PawnIoF7bsdBackend.cs:74): a tachometer
	 * glitch must not keep a leftover from being repaired.
	 */
	ret = read_sys_regs(&r);
	if (ret)
		return ret;
	state = sys_regs_state(&r);
	if (state == SYS_UNSUPPORTED)
		return dev_err_probe(dev, -ENODEV,
				     "system fan state is unsupported: effective 0x%02x, override 0x%02x, target %u\n",
				     r.effective, r.override, r.target);

	/*
	 * Repair what an earlier session left (PawnIoF7bsdBackend.cs:84-99):
	 * every leftover is marked first, so that a failed repair still has the
	 * rest released on the way out, and the system fan goes first because
	 * the firmware has no temperature fallback while it is in the handoff.
	 */
	d->fans[FAN_SYS].owned = state != SYS_FIRMWARE;
	d->fans[FAN_CPU].owned = cpu_repair;
	if (state != SYS_FIRMWARE) {
		dev_warn(dev, "system fan found in %s state (effective 0x%02x, override 0x%02x, target %u)\n",
			 sys_state_names[state], r.effective, r.override, r.target);
		if (fan_release(d, &d->fans[FAN_SYS], "leftover from an earlier session"))
			return dev_err_probe(dev, -EIO,
					     "could not return the system fan to firmware; power the machine off completely\n");
		ret = read_sys_regs(&r);
		if (ret)
			return ret;
		if (sys_regs_state(&r) != SYS_FIRMWARE)
			return dev_err_probe(dev, -EIO,
					     "system fan did not return to firmware control\n");
	}
	if (cpu_repair) {
		dev_warn(dev, "CPU table left by an earlier session; restoring it\n");
		if (fan_release(d, &d->fans[FAN_CPU], "leftover from an earlier session"))
			return dev_err_probe(dev, -EIO,
					     "could not restore the CPU table; power the machine off completely\n");
	}
	ret = check_sys_policy(dev);
	if (ret)
		return -ENODEV;

	ret = read_telemetry(t);
	if (ret)
		return dev_err_probe(dev, ret, "telemetry read failed\n");
	return 0;
}

static void mec_remove_action(void *data)
{
	mec_stop(data, "driver unload");
}

static int mec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct telemetry t;
	struct mec_data *d;
	struct device *hwmon;
	struct dentry *dir;
	int i, ret;

	if (acpi_check_region(SIO_INDEX, 2, DRVNAME))
		return dev_err_probe(dev, -EBUSY,
				     "ACPI declares an operation region over ports 0x2e-0x2f\n");
	/* Exclusive (not muxed): other Super-I/O drivers get -EBUSY. */
	if (!devm_request_region(dev, SIO_INDEX, 2, DRVNAME))
		return dev_err_probe(dev, -EBUSY, "ports 0x2e-0x2f are already in use\n");

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	d->dev = dev;
	fan_init(&d->fans[FAN_CPU], FAN_CPU, "cpu", &cpu_fan_ops, CPU_TEMP_LIMIT);
	fan_init(&d->fans[FAN_SYS], FAN_SYS, "system", &sys_fan_ops, SYS_TEMP_LIMIT);
	INIT_DELAYED_WORK(&d->monitor, mec_monitor);
	ret = devm_mutex_init(dev, &d->lock);
	if (ret)
		return ret;

	/*
	 * Runs after hwmon is gone and before the ports are released; also on
	 * probe failure.
	 */
	ret = devm_add_action_or_reset(dev, mec_remove_action, d);
	if (ret)
		return ret;

	ret = mec_admit(d, &t);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, d);

	scoped_guard(mutex, &curve_lock)
		for (i = 0; i < NR_FANS; i++)
			if (curve_params[i].curve.n)
				d->fans[i].mode = MODE_CURVE;

	hwmon = devm_hwmon_device_register_with_info(dev, DRVNAME, d,
						     &mec_chip_info, NULL);
	if (IS_ERR(hwmon))
		return PTR_ERR(hwmon);

	dir = debugfs_create_dir(DRVNAME, NULL);
	debugfs_create_file("state", 0400, dir, d, &state_fops);
	ret = devm_add_action_or_reset(dev, mec_debugfs_remove, dir);
	if (ret)
		return ret;

	dev_info(dev, "CPU table %s (0x%02x); cpu fan %s, %d RPM; system fan %s, %d RPM\n",
		 d->cpu_profile->name, d->cpu_profile->selector,
		 mode_names[d->fans[FAN_CPU].mode], t.rpm[0],
		 mode_names[d->fans[FAN_SYS].mode], t.rpm[1]);
	queue_delayed_work(system_dfl_wq, &d->monitor, 0);
	return 0;
}

/*
 * System sleep (suspend, hibernate): hand both fans back first, and refuse to
 * sleep if that fails. The selected modes apply again RESUME_SETTLE_JIFFIES
 * after resume, once the EC has its state back.
 */
static int mec_suspend(struct device *dev)
{
	struct mec_data *d = dev_get_drvdata(dev);
	int i, ret = 0;

	scoped_guard(mutex, &d->lock) {
		d->suspended = true;
		for (i = 0; i < NR_FANS; i++)
			if (fan_release(d, &d->fans[i], "system sleep"))
				ret = -EIO;
		if (ret) {
			d->suspended = false;
			mod_delayed_work(system_dfl_wq, &d->monitor, 0);
		}
	}
	if (!ret)
		cancel_delayed_work_sync(&d->monitor);
	return ret;
}

static int mec_resume(struct device *dev)
{
	struct mec_data *d = dev_get_drvdata(dev);

	scoped_guard(mutex, &d->lock)
		d->suspended = false;
	mod_delayed_work(system_dfl_wq, &d->monitor, RESUME_SETTLE_JIFFIES);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(mec_pm_ops, mec_suspend, mec_resume);

/* Reboot and power-off: never leave a fan under driver control. */
static void mec_shutdown(struct platform_device *pdev)
{
	struct mec_data *d = platform_get_drvdata(pdev);

	if (d)
		mec_stop(d, "shutdown");
}

static struct platform_driver mec_driver = {
	.driver = {
		.name = DRVNAME,
		.pm = pm_sleep_ptr(&mec_pm_ops),
	},
	.shutdown = mec_shutdown,
};

struct ec_version {
	bool found;
	u8 major;
	u8 minor;
};

/* SMBIOS type 0, bytes 0x16/0x17: EC firmware major/minor release. */
static void ec_version_decode(const struct dmi_header *dh, void *priv)
{
	struct ec_version *v = priv;
	const u8 *raw = (const u8 *)dh;

	if (dh->type != 0 || v->found)
		return;
	v->found = true;
	if (dh->length < 0x18) {
		v->major = 0xff;
		v->minor = 0xff;
		return;
	}
	v->major = raw[0x16];
	v->minor = raw[0x17];
}

/* Module autoload; mec_check_host() then checks every field exactly. */
static const struct dmi_system_id mec_dmi_table[] __initconst = {
	{
		.matches = {
			DMI_EXACT_MATCH(DMI_PRODUCT_NAME, "EliteMini Series"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "F7BSI"),
			DMI_EXACT_MATCH(DMI_BOARD_VERSION, "1.0"),
		},
	},
	{ }
};
MODULE_DEVICE_TABLE(dmi, mec_dmi_table);

/* Exact host and EC version gate, before any port access (PlatformProfiles.cs:68-81). */
static int __init mec_check_host(void)
{
	struct ec_version ver = { };
	const char *s;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(f7bsi_host); i++) {
		s = dmi_get_system_info(f7bsi_host[i].field);
		if (!s || strcmp(s, f7bsi_host[i].value)) {
			pr_err("DMI %s is \"%s\", expected \"%s\"\n",
			       f7bsi_host[i].name, s ?: "", f7bsi_host[i].value);
			return -ENODEV;
		}
	}

	if (dmi_walk(ec_version_decode, &ver) || !ver.found) {
		pr_err("SMBIOS BIOS information (type 0) not found\n");
		return -ENODEV;
	}
	if (ver.major != F7BSI_EC_MAJOR || ver.minor != F7BSI_EC_MINOR) {
		pr_err("EC firmware %u.%u, expected %u.%u\n", ver.major, ver.minor,
		       F7BSI_EC_MAJOR, F7BSI_EC_MINOR);
		return -ENODEV;
	}
	return 0;
}

static int __init mec_init(void)
{
	int ret;

	ret = mec_check_host();
	if (ret)
		return ret;

	mec_pdev = platform_create_bundle(&mec_driver, mec_probe, NULL, 0, NULL, 0);
	return PTR_ERR_OR_ZERO(mec_pdev);
}

static void __exit mec_exit(void)
{
	platform_device_unregister(mec_pdev);
	platform_driver_unregister(&mec_driver);
}

module_init(mec_init);
module_exit(mec_exit);

MODULE_DESCRIPTION("Minisforum UM760 Slim (F7BSI) EC fan monitor and control");
MODULE_LICENSE("GPL");
