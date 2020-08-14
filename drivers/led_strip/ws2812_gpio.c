/*
 * Copyright (c) 2018 Intel Corporation
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT worldsemi_ws2812_gpio

#include <drivers/led_strip.h>

#include <string.h>

#define LOG_LEVEL CONFIG_LED_STRIP_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(ws2812_gpio);

#include <zephyr.h>
#include <soc.h>
#include <drivers/gpio.h>
#include <device.h>
#include <drivers/clock_control.h>

#ifdef CONFIG_CLOCK_CONTROL_NRF
#include <drivers/clock_control/nrf_clock_control.h>
#endif

#include <sys/util_internal.h>


/* STM32 Series delay sources
 *
 * Series	Core	Max(Mhz)	Delay option
 * H7		M7F	480		DWT (>96MHz) NOP (<= 256MHz)
 * F7		M7F	216		DWT (>96MHz) NOP (<= 256MHz)
 * F4		M4F	180		DWT-NOP
 * F3		M4F	72		NOP
 * F2		M3	120		DWT-NOP
 * F1		M3	72		NOP
 * F0		M0	48		NOP
 * G4		M4F	170		DWT-NOP
 * G0		M0+	64		NOP
 * L5		M33	110		DWT-NOP
 * L4+		M4F	80		NOP
 * L1		M3	32		NOP
 * L0		M0+	32		NOP
 */

#define CYC_T1H CONFIG_WS2812_STRIP_GPIO_CYCLES_T1H
#define CYC_T1L CONFIG_WS2812_STRIP_GPIO_CYCLES_T1L
#define CYC_T0L CONFIG_WS2812_STRIP_GPIO_CYCLES_T0L
#define CYC_T0H CONFIG_WS2812_STRIP_GPIO_CYCLES_T0H

#define WS2812_GPIO_CPU_FREQ CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC

/* DWT enabled devices */
/* Only T1H and T0L are really important */
#if WS2812_GPIO_CPU_FREQ >= 80000000 && (\
	defined(CONFIG_CPU_CORTEX_M7) || \
	defined(CONFIG_CPU_CORTEX_M4) || \
	defined(CONFIG_CPU_CORTEX_M3)\
	)

#define WS2812_DELAY_USE_DWT 1
#define WS2812_DELAY_USE_NOP 0
#define WS2812_DELAY_USE_CNT 0

/* Fallback to NOP or CNT SW delay */
/* Cortex M0 and M0+ are NOP/CNT only */
#elif WS2812_GPIO_CPU_FREQ < 80000000 || \
	defined(CONFIG_CPU_CORTEX_M0) || \
	defined(CONFIG_CPU_CORTEX_M0PLUS)

#define WS2812_DELAY_USE_DWT 0
#define WS2812_DELAY_USE_NOP 1
#define WS2812_DELAY_USE_CNT 0

#endif

#if WS2812_DELAY_USE_DWT
/* Debug Watchpoint Timer */

/* DWT counter enable bit */
#define DEMCR_TRCENA    0x01000000

/* Core Debug registers */
#define DEMCR           (*((volatile uint32_t *)0xE000EDFC))
/* DWT control register */
#define DWT_CTRL        (*(volatile uint32_t *)0xe0001000)
#define CYCCNTENA       (1<<0)
/* DWT cycle counter register */
#define DWT_CYCCNT      ((volatile uint32_t *)0xE0001004)
#define CPU_CYCLES      (*DWT_CYCCNT)

static inline void DWT_RESET(void)
{
	/* Enable DWT */
	DEMCR |= DEMCR_TRCENA;
	*DWT_CYCCNT = 0;
	/* Enable CPU cycle counter */
	DWT_CTRL |= CYCCNTENA;
}

#define DWT_DELAY_INLINE(time, delay_cyc) do { \
	(time) = CPU_CYCLES + (delay_cyc); \
	while (CPU_CYCLES < (time)); \
} while (0)

static inline void DWT_DELAY(uint32_t ticks)
{
	static uint32_t time;

	DWT_DELAY_INLINE(time, ticks);
}

/* Remove function overhead to delay cycles */
#define DELAY_T1H	DWT_DELAY_INLINE(time, CYC_T1H-5)
#define DELAY_T1L	DWT_DELAY_INLINE(time, CYC_T1L-5)
#define DELAY_T0H	DWT_DELAY_INLINE(time, CYC_T0H-5)
#define DELAY_T0L	DWT_DELAY_INLINE(time, CYC_T0L-5)

#endif	/* WS2812_DELAY_USE_DWT */

/* TODO properly test and configure CNT SW delay*/
#if WS2812_DELAY_USE_CNT
/* Software delay loops */
/* Macro for waiting N*3 cycles*/
#define WAIT_Nx3_CYC(n) do { \
volatile uint32_t cyc = n;\
__asm volatile( \
	".syntax unified; \n" \
	"0: \n" \
	"subs %[count], #1 \n" \
	"bne 0b	\n" \
	".syntax unified; \n" \
	::\
	[count] "l" (cyc)); \
} while (0)

/* set the ch with register of value N */
/* waits 3 cycles each time the branch is taken */
/* add 1 cycle per number of wait states per instruction */
/* uses 2 cycles if branch is not taken */
/* for ch==1, only 2 cycles are used */
/* we need a NOP after the branch to compensate for the 1 cyc branch */
#define DELAY_Nx3_HIGH_CYC_ASM \
		"0: \n" \
		"subs %[ch], #1 \n" \
		"bne 0b	\n" \

/* set the cl with register of value N */
#define DELAY_Nx3_LOW_CYC_ASM \
		"1: \n" \
		"subs %[cl], #1 \n" \
		"bne 1b	\n" \


#define WS2812_NOPS(delay_ns) \
	(((delay_ns)*(WS2812_GPIO_CPU_FREQ))/(NSEC_PER_SEC))

#define DELAY_T1H_ASM	DELAY_Nx3_HIGH_CYC_ASM
#define DELAY_T1L_ASM	DELAY_Nx3_LOW_CYC_ASM
#define DELAY_T0H_ASM	DELAY_Nx3_HIGH_CYC_ASM
#define DELAY_T0L_ASM	DELAY_Nx3_LOW_CYC_ASM

#endif /* WS2812_DELAY_USE_CNT */


#if WS2812_DELAY_USE_NOP
/* NOP inline delays */
/* UTIL_REPEAT can only generate 256 NOP maximum */

#define DELAY_NOP(i, _) "ADD SP, #0 \n"
#define DELAY_T1H_ASM UTIL_EVAL(UTIL_REPEAT(CYC_T1H, DELAY_NOP))
#define DELAY_T1L_ASM UTIL_EVAL(UTIL_REPEAT(CYC_T1L, DELAY_NOP))
#define DELAY_T0L_ASM UTIL_EVAL(UTIL_REPEAT(CYC_T0L, DELAY_NOP))
#define DELAY_T0H_ASM UTIL_EVAL(UTIL_REPEAT(CYC_T0H, DELAY_NOP))

#define DELAY_T1H __asm volatile(DELAY_T1H_ASM)
#define DELAY_T1L __asm volatile(DELAY_T1L_ASM)
#define DELAY_T0H __asm volatile(DELAY_T0H_ASM)
#define DELAY_T0L __asm volatile(DELAY_T0L_ASM)

#endif /* WS2812_DELAY_USE_NOP */


struct ws2812_gpio_data {
	struct device *gpio;
	/* Used for direct gpio manipulation */
	uint32_t gpio_base;
};

struct ws2812_gpio_cfg {
	uint8_t pin;
	bool has_white;
};

static struct ws2812_gpio_data *dev_data(struct device *dev)
{
	return dev->data;
}

static const struct ws2812_gpio_cfg *dev_cfg(struct device *dev)
{
	return dev->config;
}

/*
 * GPIO set/clear
 *
 * This uses BSRR low halfword to set HIGH
 * BSSR high halfword to reset to LOW
 *
 */

#if CONFIG_SOC_FAMILY_STM32
/* Write the BSRR low halfword to set the gpio */
#define SET_HIGH_ASM "strh %[ps], [%[rs], #0]\n"
/* Write BSRR high halfword to reset the gpio */
#define SET_LOW_ASM "strh %[ps], [%[rs], #2]\n"
#endif

/*
 * GPIO set/clear (these make assumptions about assembly details
 * below).
 *
 * This uses OUTCLR == OUTSET+4.
 *
 * We should be able to make this portable using the results of
 * https://github.com/zephyrproject-rtos/zephyr/issues/11917.
 *
 * We already have the GPIO device stashed in ws2812_gpio_data, so
 * this driver can be used as a test case for the optimized API.
 *
 * Per Arm docs, both Rd and Rn must be r0-r7, so we use the "l"
 * constraint in the below assembly.
 *
 */
#if CONFIG_SOC_FAMILY_NRF
/* OUTSET = BIT(LED_PIN) */
#define SET_HIGH_ASM "str %[ps], [%[rs], #0]\n"
/* OUTCLR = BIT(LED_PIN) */
#define SET_LOW_ASM "str %[ps], [%[rs], #4]\n"
#endif

#define SET_HIGH(base, pin) \
	__asm volatile(SET_HIGH_ASM :: [rs] "l" (base), [ps] "l" (pin))

#define SET_LOW(base, pin) \
	__asm volatile(SET_LOW_ASM :: [rs] "l" (base), [ps] "l" (pin))

#if WS2812_DELAY_USE_CNT
#define ONE_BIT_PAD(base, pin, c1h, c1l, padh) \
	__asm volatile( \
		".syntax unified; \n" \
		SET_HIGH_ASM \
		DELAY_Nx3_HIGH_CYC_ASM \
		DELAY_NOP_ASM(padh) \
		SET_LOW_ASM \
		DELAY_Nx3_LOW_CYC_ASM \
		".syntax divided;\n" \
		:: \
		[rs] "l" (base), \
		[ps] "l" (pin), \
		[ch] "l" (c1h), \
		[cl] "l" (c1l) \
	)

#define ZERO_BIT_PAD(base, pin, c0h, c0l, padh) \
	__asm volatile( \
		".syntax unified; \n" \
		SET_HIGH_ASM \
		DELAY_Nx3_HIGH_CYC_ASM \
		DELAY_NOP_ASM(padh) \
		SET_LOW_ASM \
		DELAY_Nx3_LOW_CYC_ASM \
		".syntax divided;\n" \
		:: \
		[rs] "l" (base), \
		[ps] "l" (pin), \
		[ch] "l" (c0h), \
		[cl] "l" (c0l) \
	)

#define ONE_BIT(base, pin, c1h, c1l) \
	__asm volatile( \
		".syntax unified; \n" \
		SET_HIGH_ASM \
		DELAY_T1H_ASM \
		SET_LOW_ASM \
		DELAY_T1L_ASM \
		".syntax divided;\n" \
		:: \
		[rs] "l" (base), \
		[ps] "l" (pin), \
		[ch] "l" (c1h), \
		[cl] "l" (c1l) \
	)


#define ZERO_BIT(base, pin, c0h, c0l) \
	__asm volatile( \
		".syntax unified; \n" \
		SET_HIGH_ASM \
		DELAY_T0H_ASM \
		SET_LOW_ASM \
		DELAY_T0L_ASM \
		".syntax divided;\n" \
		:: \
		[rs] "l" (base), \
		[ps] "l" (pin), \
		[ch] "l" (c0h), \
		[cl] "l" (c0l) \
	)
#endif


#if defined(CONFIG_SOC_FAMILY_STM32) && WS2812_DELAY_USE_CNT
static int get_wait_states_stm32(void)
{
	switch (LL_FLASH_GetLatency()) {
#ifdef LL_FLASH_LATENCY_0
	case LL_FLASH_LATENCY_0:
		return 0;
#endif
#ifdef LL_FLASH_LATENCY_1
	case LL_FLASH_LATENCY_1:
		return 1;
#endif
#ifdef LL_FLASH_LATENCY_2
	case LL_FLASH_LATENCY_2:
		return 2;
#endif
#ifdef LL_FLASH_LATENCY_3
	case LL_FLASH_LATENCY_3:
		return 3;
#endif
#ifdef LL_FLASH_LATENCY_4
	case LL_FLASH_LATENCY_4:
		return 4;
#endif
#ifdef LL_FLASH_LATENCY_5
	case LL_FLASH_LATENCY_5:
		return 5;
#endif
#ifdef LL_FLASH_LATENCY_6
	case LL_FLASH_LATENCY_6:
		return 6;
#endif
#ifdef LL_FLASH_LATENCY_7
	case LL_FLASH_LATENCY_7:
		return 7;
#endif
	default:
		return 0;
	}
}
#endif


#ifdef CONFIG_SOC_FAMILY_STM32
static int send_buf(struct device *dev, uint8_t *buf, size_t len)
{
	unsigned int key;
	/* Direct GPIO port manipulation */
	volatile GPIO_TypeDef *gpio = (GPIO_TypeDef *)dev_data(dev)->gpio_base;
	volatile uint32_t *base = &(gpio->BSRR);
	const uint16_t pin = BIT(dev_cfg(dev)->pin);

	/* PIN level inversion handling */
	const struct gpio_driver_data *const data =
		(const struct gpio_driver_data *)dev_data(dev)->gpio->data;

#if WS2812_DELAY_USE_DWT
	/* Cycle reference time, used for inlined delays */
	uint32_t time = 0;
	DWT_RESET();
#endif /* WS2812_DELAY_USE_DWT */

#if WS2812_DELAY_USE_CNT
	volatile uint32_t c1h;
	volatile uint32_t c1l;
	volatile uint32_t c0h;
	volatile uint32_t c0l;
	volatile uint32_t cxl;
	/* Prefetch is now disabled every instruction is read from flash */
	/* Wait states now account in the instruction fetch timings */
	int nws = get_wait_states_stm32();
	/* For each instruction fetched from flash, we need to account nws cycles */
	/* So each instruction takes nws cycles for each instruction fetching and */
	/* the instruction cycle duration */
	/* 1 instruction set GPIO */
	/* 2 instruction delay loop of 3 cycles */
	/* 1 instruction set GPIO */
	/* 2 instruction delay loop of 3 cycles*/

	uint32_t cycle_per_delayloop = 3 + 2*nws;

	c1h = CYC_T1H/cycle_per_delayloop;
	c1l = CYC_T1L/cycle_per_delayloop;
	c0h = CYC_T0H/cycle_per_delayloop;
	c0l = CYC_T0L/cycle_per_delayloop;

	/* Do full loop for multiple and pad with nop for the remaining */
	/* Only t1h and t0h are really important timings */
	/* So nop padding is only for those 2 */
	/* Number of necessary NOP to pad delays for r1h and r0h */
	uint32_t r1h = c1h%cycle_per_delayloop;
	uint32_t r0h = c0h%cycle_per_delayloop;
#endif

	/* prevent this thread from being preempted */
	k_sched_lock();
	/* block IRQ so we have all the CPU time */
	key = irq_lock();

	/* Branch before to keep determinism of pulses */
	/* ACTIVE_LOW, invert signal */
	if (data->invert & (gpio_port_pins_t) pin) {
		while (len--) {
			uint32_t b = *buf++;
			int32_t i;

			for (i = 7; i >= 0; i--) {
				if (b & BIT(i)) {
				#if WS2812_DELAY_USE_DWT || WS2812_DELAY_USE_NOP
					SET_LOW(base, pin);
					DELAY_T1H;
					SET_HIGH(base, pin);
					DELAY_T1L;
				#elif WS2812_DELAY_USE_CNT
					__asm volatile(
						".syntax unified; \n"
						SET_LOW_ASM
						DELAY_T1H_ASM
						SET_HIGH_ASM
						DELAY_T1L_ASM
						".syntax divided;\n"
						::
						[rs] "l" (base),
						[ps] "l" (pin),
						[ch] "l" (cdh),
						[cl] "l" (cdl)
					);
				#endif
				} else {
				#if WS2812_DELAY_USE_DWT || WS2812_DELAY_USE_NOP
					SET_LOW(base, pin);
					DELAY_T0H;
					SET_HIGH(base, pin);
					DELAY_T0L;
				#elif WS2812_DELAY_USE_CNT
					__asm volatile(
						".syntax unified; \n"
						SET_LOW_ASM
						DELAY_T0H_ASM
						SET_HIGH_ASM
						DELAY_T0L_ASM
						".syntax divided;\n"
						::
						[rs] "l" (base),
						[ps] "l" (pin),
						[ch] "l" (cdh),
						[cl] "l" (cdl)
					);
				#endif
				}
			}
		}
	}
	/* Normal GPIO active high level*/
	else {
		while (len--) {
			uint32_t b = *buf++;
			int32_t i;

			for (i = 7; i >= 0; i--) {
				if (b & BIT(i)) {
				#if WS2812_DELAY_USE_DWT || WS2812_DELAY_USE_NOP
					SET_HIGH(base, pin);
					DELAY_T1H;
					SET_LOW(base, pin);
					DELAY_T1L;
				#elif WS2812_DELAY_USE_CNT
					__asm volatile(
						".syntax unified; \n"
						SET_HIGH_ASM
						DELAY_T1H_ASM
						SET_LOW_ASM
						DELAY_T1L_ASM
						".syntax divided;\n"
						::
						[rs] "l" (base),
						[ps] "l" (pin),
						[ch] "l" (cdh),
						[cl] "l" (cdl)
					);
				#endif
				} else {
				#if WS2812_DELAY_USE_DWT || WS2812_DELAY_USE_NOP
					SET_HIGH(base, pin);
					DELAY_T0H;
					SET_LOW(base, pin);
					DELAY_T0L;
				#elif WS2812_DELAY_USE_CNT
					__asm volatile(
						".syntax unified; \n"
						SET_HIGH_ASM
						DELAY_T0H_ASM
						SET_LOW_ASM
						DELAY_T0L_ASM
						".syntax divided;\n"
						::
						[rs] "l" (base),
						[ps] "l" (pin),
						[ch] "l" (cdh),
						[cl] "l" (cdl)
					);
				#endif
				}
			}
		}
	}

	irq_unlock(key);

	k_sched_unlock();

	return 0;
}

#endif


#ifdef CONFIG_SOC_FAMILY_NRF
static int send_buf(struct device *dev, uint8_t *buf, size_t len)
{
	int rc = 0;
	/* Direct GPIO port manipulation */
	volatile NRF_GPIO_Type *gpio =
		(NRF_GPIO_Type *)dev_data(dev)->gpio_base;
	volatile uint32_t *base = &(gpio->OUTSET);
	const uint32_t pin = BIT(dev_cfg(dev)->pin);

	/* PIN level inversion handling */
	const struct gpio_driver_data *const data =
		(const struct gpio_driver_data *)dev_data(dev)->gpio->data;

	struct onoff_manager *mgr =
		z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	struct onoff_client cli;
	unsigned int key;


	sys_notify_init_spinwait(&cli.notify);
	rc = onoff_request(mgr, &cli);
	if (rc < 0) {
		return rc;
	}

	while (sys_notify_fetch_result(&cli.notify, &rc)) {
		/* pend until clock is up and running */
	}

	key = irq_lock();

	/* Branch before to keep determinism of pulses */
	/* ACTIVE_LOW, invert signal */
	if (data->invert & (gpio_port_pins_t) pin) {
		while (len--) {
			uint32_t b = *buf++;
			int32_t i;

			/*
			 * Generate signal out of the bits, MSbit first.
			 *
			 * Accumulator maintenance and branching mean the
			 * inter-bit time will be longer than TxL, but the
			 * wp.josh.com blog post says we have at least 5 usec
			 * of slack time between bits before we risk the
			 * signal getting latched, so this will be fine as
			 * long as the compiler does something minimally
			 * reasonable.
			 */
			for (i = 7; i >= 0; i--) {
				if (b & BIT(i)) {
					SET_HIGH(base, pin);
					DELAY_T1H;
					SET_LOW(base, pin);
					DELAY_T1L;
				} else {
					SET_HIGH(base, pin);
					DELAY_T0H;
					SET_LOW(base, pin);
					DELAY_T0L;
				}
			}
		}
	}
	/* Normal ACTIVE HIGH level */
	else {
		while (len--) {
			uint32_t b = *buf++;
			int32_t i;

			for (i = 7; i >= 0; i--) {
				if (b & BIT(i)) {
					SET_LOW(base, pin);
					DELAY_T1H;
					SET_HIGH(base, pin);
					DELAY_T1L;
				} else {
					SET_LOW(base, pin);
					DELAY_T0H;
					SET_HIGH(base, pin);
					DELAY_T0L;
				}
			}
		}
	}

	irq_unlock(key);

	rc = onoff_release(mgr);
	/* Returns non-negative value on success. Cap to 0 as API states. */

	rc = MIN(rc, 0);

	return rc;
}
#endif

static int ws2812_gpio_update_rgb(struct device *dev, struct led_rgb *pixels,
				  size_t num_pixels)
{
	const struct ws2812_gpio_cfg *config = dev->config;
	const bool has_white = config->has_white;
	uint8_t *ptr = (uint8_t *)pixels;
	size_t i;

	/* Convert from RGB to on-wire format (GRB or GRBW) */
	for (i = 0; i < num_pixels; i++) {
		uint8_t r = pixels[i].r;
		uint8_t g = pixels[i].g;
		uint8_t b = pixels[i].b;

		*ptr++ = g;
		*ptr++ = r;
		*ptr++ = b;
		if (has_white) {
			*ptr++ = 0; /* white channel is unused */
		}
	}

	return send_buf(dev, (uint8_t *)pixels, num_pixels * (has_white ? 4 : 3));
}

static int ws2812_gpio_update_channels(struct device *dev, uint8_t *channels,
				       size_t num_channels)
{
	LOG_ERR("update_channels not implemented");
	return -ENOTSUP;
}

static const struct led_strip_driver_api ws2812_gpio_api = {
	.update_rgb = ws2812_gpio_update_rgb,
	.update_channels = ws2812_gpio_update_channels,
};

#define WS2812_GPIO_LABEL(idx) \
	(DT_INST_LABEL(idx))
#define WS2812_GPIO_HAS_WHITE(idx) \
	(DT_INST_PROP(idx, has_white_channel) == 1)
#define WS2812_GPIO_DEV(idx) \
	(DT_INST_GPIO_LABEL(idx, in_gpios))
#define WS2812_GPIO_PIN(idx) \
	(DT_INST_GPIO_PIN(idx, in_gpios))
#define WS2812_GPIO_FLAGS(idx) \
	(DT_INST_GPIO_FLAGS(idx, in_gpios))
#define WS2812_GPIO_BASE(idx) \
	(DT_REG_ADDR(DT_INST_PHANDLE(idx, in_gpios)))

#ifdef CONFIG_SOC_FAMILY_NRF
/*
 * The inline assembly above is designed to work on nRF51 devices with
 * the 16 MHz clock enabled.
 *
 * TODO: try to make this portable, or at least port to more devices.
 */
#define WS2812_GPIO_CLK(idx) DT_LABEL(DT_INST(0, nordic_nrf_clock))
#endif

#define WS2812_GPIO_DEVICE(idx)					\
									\
	static int ws2812_gpio_##idx##_init(struct device *dev)	\
	{								\
		struct ws2812_gpio_data *data = dev_data(dev);		\
		data->gpio_base = WS2812_GPIO_BASE(idx); \
		data->gpio = device_get_binding(WS2812_GPIO_DEV(idx));	\
		if (!data->gpio) {					\
			LOG_ERR("Unable to find GPIO controller %s",	\
				WS2812_GPIO_DEV(idx));			\
			return -ENODEV;				\
		}							\
		LOG_INF("ws2812_gpio_reg name: %s", data->gpio->name); \
		LOG_INF("ws2812_gpio base %p", (uint32_t *) data->gpio_base); \
		return gpio_pin_configure(data->gpio,			\
					  WS2812_GPIO_PIN(idx),	\
					  WS2812_GPIO_FLAGS(idx) |	\
					  GPIO_OUTPUT);		\
	}								\
									\
	static struct ws2812_gpio_data ws2812_gpio_##idx##_data;	\
									\
	static const struct ws2812_gpio_cfg ws2812_gpio_##idx##_cfg = { \
		.pin = WS2812_GPIO_PIN(idx),				\
		.has_white = WS2812_GPIO_HAS_WHITE(idx),		\
	};								\
									\
	DEVICE_AND_API_INIT(ws2812_gpio_##idx, WS2812_GPIO_LABEL(idx),	\
			    ws2812_gpio_##idx##_init,			\
			    &ws2812_gpio_##idx##_data,			\
			    &ws2812_gpio_##idx##_cfg, POST_KERNEL,	\
			    CONFIG_LED_STRIP_INIT_PRIORITY,		\
			    &ws2812_gpio_api);

DT_INST_FOREACH_STATUS_OKAY(WS2812_GPIO_DEVICE)
