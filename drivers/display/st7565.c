/*
 * Copyright (c) 2020 LOCHE Jeremy <lochejeremy@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sitronix_st7565fb

#include <logging/log.h>
LOG_MODULE_REGISTER(st7565, CONFIG_DISPLAY_LOG_LEVEL);

#include <string.h>
#include <device.h>
#include <drivers/display.h>
#include <init.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <sys/byteorder.h>

#include "st7565_regs.h"

/**
 * ST7565 / ST7567 compatible LCD controller driver.
 * i.e. screen compatibles:
 * - Winstar WO12864
 * - PowerTip PE12864-004
 * - Adafruit ST7565
 */

#define ST7565_SPI_FREQ DT_INST_PROP(0, spi_max_frequency)
#define ST7565_BUS_NAME DT_INST_BUS_LABEL(0)

#define ST7565_CS_PIN DT_INST_SPI_DEV_CS_GPIOS_PIN(0)
#define ST7565_CS_FLAGS DT_INST_SPI_DEV_CS_GPIOS_FLAGS(0)
#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
#define ST7565_CS_CNTRL DT_INST_SPI_DEV_CS_GPIOS_LABEL(0)
#endif

#define ST7565_RS_PIN DT_INST_GPIO_PIN(0, rs_gpios)
#define ST7565_RS_CNTRL DT_INST_GPIO_LABEL(0, rs_gpios)
#define ST7565_RS_FLAGS DT_INST_GPIO_FLAGS(0, rs_gpios)

#define ST7565_RESET_PIN DT_INST_GPIO_PIN(0, reset_gpios)
#define ST7565_RESET_CNTRL DT_INST_GPIO_LABEL(0, reset_gpios)
#define ST7565_RESET_FLAGS DT_INST_GPIO_FLAGS(0, reset_gpios)

#define ST7565_BIAS_SELECT		(DT_INST_PROP(0, bias) ? \
					ST7565_BIAS_1_7 : ST7565_BIAS_1_9)

#define ST7565_REGULATOR_RATIO		DT_INST_PROP(0, regulator_ratio)
#define ST7565_EV_CONTRAST_RANGE	DT_INST_PROP(0, ev_contrast_range)

#define LCD_PANEL_WIDTH			DT_INST_PROP(0, width)
#define LCD_PANEL_HEIGHT		DT_INST_PROP(0, height)
#define LCD_PANEL_NUMOF_COLUMS		LCD_PANEL_WIDTH
#define LCD_PANEL_NUMOF_ROWS_PER_PAGE	8
#define LCD_PANEL_NUMOF_PAGES		(LCD_PANEL_HEIGHT / \
					 LCD_PANEL_NUMOF_ROWS_PER_PAGE)

/* From datasheet, give voltage corresponding to rr and ev setting */
#define ST7565_GET_VOLTAGE(rr, ev)	((rr)*2.1*(99+(ev))/162)

#define ST7565_PIXELS_PER_BYTE		8

struct st7565_data {
	const struct device *reset;
	const struct device *rs;
	const struct device *spi_dev;
	struct spi_config spi_config;
	struct spi_cs_control cs_ctrl;
	bool inverted_pixel_mode;
};

static inline void st7565_set_cmd_mode(struct st7565_data *driver)
{
	/* Select the command register */
	gpio_pin_set(driver->rs, ST7565_RS_PIN, 1);
}

static inline void st7565_set_data_mode(struct st7565_data *driver)
{
	/* Select the data register */
	gpio_pin_set(driver->rs, ST7565_RS_PIN, 0);
}

static inline int st7565_write_cmd_buf(struct st7565_data *driver,
				       uint8_t *cmd, size_t len)
{
	int err;
	struct spi_buf buf = {.buf = cmd, .len = len};
	struct spi_buf_set buf_set = {.buffers = &buf, .count = 1};

	/* Select the command register */
	st7565_set_cmd_mode(driver);
	err = spi_write(driver->spi_dev, &driver->spi_config, &buf_set);
	if (err < 0) {
		return err;
	}
	return 0;
}

static inline int st7565_write_cmd(struct st7565_data *driver, uint8_t cmd)
{
	return st7565_write_cmd_buf(driver, &cmd, 1);
}

static inline int st7565_write_data(struct st7565_data *driver,
				    uint8_t *data, size_t len)
{
	int err;
	struct spi_buf buf = {.buf = data, .len = len};
	struct spi_buf_set buf_set = {.buffers = &buf, .count = 1};

	if (data != NULL) {
		/* Select the data register */
		st7565_set_data_mode(driver);
		err = spi_write(driver->spi_dev, &driver->spi_config, &buf_set);
		if (err < 0) {
			return err;
		}
	}

	return 0;
}

static int st7565_blanking_off(const struct device *dev)
{
	return -ENOTSUP;
}

static int st7565_blanking_on(const struct device *dev)
{
	return -ENOTSUP;
}

static int st7565_write(const struct device *dev, const uint16_t x,
			 const uint16_t y,
			 const struct display_buffer_descriptor *desc,
			 const void *buf)
{
	struct st7565_data *driver = dev->data;
	int err;
	size_t buf_len;
	uint8_t page_start;
	uint8_t nb_pages;
	uint8_t cursor_cmd[3];

	LOG_DBG("x:%d y:%d w:%d h:%d", x, y, desc->width, desc->height);

	if (desc->pitch < desc->width) {
		LOG_ERR("Pitch is smaller than width");
		return -EINVAL;
	}

	buf_len = MIN(desc->buf_size, desc->height * desc->width / 8);
	if (buf == NULL || buf_len == 0U) {
		LOG_ERR("Display buffer is not available");
		return -EINVAL;
	}

	if (desc->pitch > desc->width) {
		LOG_ERR("Unsupported mode");
		return -ENOTSUP;
	}

	if ((y + desc->height) > LCD_PANEL_HEIGHT) {
		LOG_ERR("Buffer out of bounds (height)");
		return -EINVAL;
	}

	if ((x + desc->width) > LCD_PANEL_WIDTH) {
		LOG_ERR("Buffer out of bounds (width)");
		return -EINVAL;
	}

	/* We can only update starting from a page */
	/* and update a multiple of pages */
	/* Buffer needs to be n*NUMOF_ROWS_PER_PAGE */
	if ((desc->height % LCD_PANEL_NUMOF_ROWS_PER_PAGE) != 0U) {
		LOG_ERR("Buffer height not multiple of %d",
				LCD_PANEL_NUMOF_ROWS_PER_PAGE);
		return -EINVAL;
	}

	/* We need to start at the beginning of a page */
	if ((y % LCD_PANEL_NUMOF_ROWS_PER_PAGE) != 0U) {
		LOG_ERR("Y coordinate not multiple of %d",
				LCD_PANEL_NUMOF_ROWS_PER_PAGE);
		return -EINVAL;
	}

	/* x can be any value, y must be a multiple of NUMOF_ROWS_PER_PAGE */
	/* find the page corresponding to start position */
	page_start = y / LCD_PANEL_NUMOF_ROWS_PER_PAGE;
	/* find the last page to update */

	nb_pages = desc->height / LCD_PANEL_NUMOF_ROWS_PER_PAGE;

	/* Set x = column index need only to be done once */
	cursor_cmd[1] = ST7565_CMD_SET_COL_ADDR_LSB(x);
	cursor_cmd[2] = ST7565_CMD_SET_COL_ADDR_MSB(x);

	/* Write each page data */
	for (uint8_t page_off = 0; page_off < nb_pages; page_off++) {
		/* Update y = page index */
		cursor_cmd[0] = ST7565_CMD_SET_PAGE_ADDR(
					page_start + page_off);
		/* Write X and Y values */
		st7565_write_cmd_buf(driver, cursor_cmd, 3);

		/* Write each page data */
		err = st7565_write_data(driver,
					(uint8_t *) buf + page_off*desc->width,
					desc->width);
	}

	return 0;
}

static int st7565_read(const struct device *dev, const uint16_t x,
			const uint16_t y,
			const struct display_buffer_descriptor *desc,
			void *buf)
{
	LOG_ERR("not supported");
	return -ENOTSUP;
}

static void *st7565_get_framebuffer(const struct device *dev)
{
	LOG_ERR("not supported");
	return NULL;
}

static int st7565_set_brightness(const struct device *dev,
				  const uint8_t brightness)
{
	LOG_WRN("not supported");
	return -ENOTSUP;
}

static int st7565_set_contrast(const struct device *dev, uint8_t contrast)
{
	int err;
	struct st7565_data *driver = dev->data;

	const uint8_t ev_range[] = ST7565_EV_CONTRAST_RANGE;
	int ev_min = ev_range[0];
	int ev_max = ev_range[1];
	int ev = ev_min + (ev_max-ev_min)*contrast/255;

	/* Send contrast header command */
	err = st7565_write_cmd(driver, ST7565_CMD_EV_HEADER);

	if (err) {
		return err;
	}

	/* Send contrast value */
	err = st7565_write_cmd(driver, ST7565_CMD_EV_VALUE(ev));

	if (err) {
		return err;
	}

	return 0;
}

static void st7565_get_capabilities(const struct device *dev,
				    struct display_capabilities *caps)
{
	struct st7565_data *driver = dev->data;

	memset(caps, 0, sizeof(struct display_capabilities));
	caps->x_resolution = LCD_PANEL_WIDTH;
	caps->y_resolution = LCD_PANEL_HEIGHT;
	caps->supported_pixel_formats = PIXEL_FORMAT_MONO01 |
					PIXEL_FORMAT_MONO10;
	if (driver->inverted_pixel_mode) {
		caps->current_pixel_format = PIXEL_FORMAT_MONO10;
	} else {
		caps->current_pixel_format = PIXEL_FORMAT_MONO01;
	}
	caps->screen_info = SCREEN_INFO_MONO_VTILED;
}

static int st7565_set_orientation(const struct device *dev,
				   const enum display_orientation
				   orientation)
{
	struct st7565_data *driver = dev->data;

	switch (orientation) {
	case DISPLAY_ORIENTATION_NORMAL:
		/* Orientation Normal : COM_REVERSE, SEG_NORMAL */
		st7565_write_cmd(driver, ST7565_CMD_SEG_DIR_NORMAL);
		st7565_write_cmd(driver, ST7565_CMD_COM_DIR_REVERSE);
		return 0;
	case DISPLAY_ORIENTATION_ROTATED_180:
		/* Orientation Reverse : COM_NORMAL, SEG_REVERSE */
		st7565_write_cmd(driver, ST7565_CMD_COM_DIR_NORMAL);
		st7565_write_cmd(driver, ST7565_CMD_SEG_DIR_REVERSE);
		return 0;
	default:
		break;
	}
	LOG_ERR("not supported");
	return -ENOTSUP;
}

static int st7565_set_pixel_format(const struct device *dev,
				    const enum display_pixel_format pf)
{
	struct st7565_data *driver = dev->data;

	switch (pf) {
	case PIXEL_FORMAT_MONO01:
		/* Set to normal mode */
		/* treat off pixel as 0 and on pixel as 1 */
		driver->inverted_pixel_mode = 0;
		st7565_write_cmd(driver, ST7565_CMD_NORMAL_DISPLAY);
		return 0;
	case PIXEL_FORMAT_MONO10:
		/* Set to inverted mode */
		/* treat off pixel as 1 and on pixel as 0 */
		driver->inverted_pixel_mode = 1;
		st7565_write_cmd(driver, ST7565_CMD_INVERSE_DISPLAY);
		return 0;
	default:
		break;
	}

	LOG_ERR("not supported");
	return -ENOTSUP;
}

static int st7565_controller_init(const struct device *dev)
{
	int err;
	struct st7565_data *driver = dev->data;

	/* Hardware Reset of the LCD */
	gpio_pin_set(driver->reset, ST7565_RESET_PIN, 1);
	k_msleep(ST7565_RESET_DELAY);
	gpio_pin_set(driver->reset, ST7565_RESET_PIN, 0);
	k_msleep(ST7565_RESET_DELAY);

	/* Do a Soft Reset */
	/* After SRSTn start line, column, page, com output mode */
	/* Regulator ratio, electronic volume and static indicator are reset*/
	st7565_write_cmd(driver, ST7565_CMD_SW_RESET);

	/* Configure Bias to 1/9 */
	st7565_write_cmd(driver, ST7565_CMD_BIAS(ST7565_BIAS_SELECT));

	/* Configure SEG direction */
	/* Configure COM direction */
	/* This is done with orientation command */
	/* For normal up to down Y axis, we need Reverse mode */
	st7565_set_orientation(dev, DISPLAY_ORIENTATION_NORMAL);

	/* Select regulation ratio */ /* for ST7565 RR=6 */
	st7565_write_cmd(driver, ST7565_CMD_REG_RATIO(ST7565_REGULATOR_RATIO));

	/* Select Contrast to middle value */
	st7565_set_contrast(dev, 128);

	/* Select booster ratio : x4 by default */
	st7565_write_cmd(driver, ST7565_CMD_BOOSTER_RATIO_HEADER);
	st7565_write_cmd(driver, ST7565_CMD_BOOSTER_RATIO_VALUE(0));

	/* Select Power Control */ /* Enable all regulators */
	st7565_write_cmd(driver, ST7565_CMD_PWR_CTRL |
				 ST7565_CMD_PWR_VR_MASK |
				 ST7565_CMD_PWR_VB_MASK |
				 ST7565_CMD_PWR_VF_MASK);

	/* Set Pixel format */
	st7565_set_pixel_format(dev, PIXEL_FORMAT_MONO01);
	/* Initialize DDRAM */
	/* Don't do it to save on boot time. Upper layers will do it */
	/* Display ON */
	err = st7565_write_cmd(driver, ST7565_CMD_DISPLAY_ON);
	if (err) {
		return err;
	}


	LOG_DBG("ST7565 init complete");
	return 0;
}

static int st7565_init(const struct device *dev)
{
	struct st7565_data *driver = dev->data;

	LOG_DBG("Initializing ST7565");

	driver->spi_dev = device_get_binding(ST7565_BUS_NAME);
	if (driver->spi_dev == NULL) {
		LOG_ERR("Could not get SPI device for ST7565");
		return -EIO;
	}

	driver->spi_config.frequency = ST7565_SPI_FREQ;
	driver->spi_config.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8);
	driver->spi_config.slave = DT_INST_REG_ADDR(0);
	driver->spi_config.cs = NULL;

	driver->reset = device_get_binding(ST7565_RESET_CNTRL);
	if (driver->reset == NULL) {
		LOG_ERR("Could not get GPIO port for ST7565 reset");
		return -EIO;
	}

	gpio_pin_configure(driver->reset, ST7565_RESET_PIN,
			   GPIO_OUTPUT_INACTIVE | ST7565_RESET_FLAGS);

	driver->rs = device_get_binding(ST7565_RS_CNTRL);
	if (driver->rs == NULL) {
		LOG_ERR("Could not get GPIO port for ST7565 RS signal");
		return -EIO;
	}

	gpio_pin_configure(driver->rs, ST7565_RS_PIN,
			   GPIO_OUTPUT_INACTIVE | ST7565_RS_FLAGS);

#if defined(ST7565_CS_CNTRL)
	driver->cs_ctrl.gpio_dev = device_get_binding(ST7565_CS_CNTRL);
	if (!driver->cs_ctrl.gpio_dev) {
		LOG_ERR("Unable to get SPI GPIO CS device");
		return -EIO;
	}

	driver->cs_ctrl.gpio_pin = ST7565_CS_PIN;
	driver->cs_ctrl.gpio_dt_flags = ST7565_CS_FLAGS;
	driver->cs_ctrl.delay = 0U;
	driver->spi_config.cs = &driver->cs_ctrl;
#endif

	return st7565_controller_init(dev);
}

static struct st7565_data st7565_driver;

static struct display_driver_api st7565_driver_api = {
	.blanking_on = st7565_blanking_on,
	.blanking_off = st7565_blanking_off,
	.write = st7565_write,
	.read = st7565_read,
	.get_framebuffer = st7565_get_framebuffer,
	.set_brightness = st7565_set_brightness,
	.set_contrast = st7565_set_contrast,
	.get_capabilities = st7565_get_capabilities,
	.set_pixel_format = st7565_set_pixel_format,
	.set_orientation = st7565_set_orientation,
};


DEVICE_AND_API_INIT(st7565, DT_INST_LABEL(0), st7565_init,
		    &st7565_driver, NULL,
		    POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY,
		    &st7565_driver_api);
