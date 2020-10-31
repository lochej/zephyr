/* Registers definition for ST7565 / ST7567 compatible controller */

/*
 * Copyright (c) 2020 LOCHE Jeremy <lochejeremy@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ST7565_REGS_H__
#define __ST7565_REGS_H__

#define ST7565_CMD_SW_RESET			0xe2
#define ST7565_CMD_DISPLAY_ON			0xaf
#define ST7565_CMD_DISPLAY_OFF			0xae
#define ST7565_CMD_SET_START_LINE(line)		(0x40 | ((line)&0x3F))
#define ST7565_CMD_SET_PAGE_ADDR(addr)		(0xb0 | ((addr)&0x0F))
/* Provides lower and upper command byte from 8 bits collumn addr */
#define ST7565_CMD_SET_COL_ADDR_LSB(col)	(0x00 | ((col)&0x0F))
#define ST7565_CMD_SET_COL_ADDR_MSB(col)	(0x10 | (((col) >> 4)&0x0F))

#define ST7565_BIAS_1_7				0x01
#define ST7565_BIAS_1_9				0x00

#define ST7565_CMD_BIAS(bias)			(0xa2 | ((bias)&0x01))
/* Regulator ratio from 0 to 7 */
#define ST7565_CMD_REG_RATIO(num)		(0x20 | ((num)&0x07))
/* Power control */
#define ST7565_CMD_PWR_CTRL			0x28
#define ST7565_CMD_PWR_VR_MASK			0x02
#define ST7565_CMD_PWR_VB_MASK			0x04
#define ST7565_CMD_PWR_VF_MASK			0x01

#define ST7565_CMD_COM_DIR_NORMAL		0xc0
#define ST7565_CMD_COM_DIR_REVERSE		0xc8

#define ST7565_CMD_SEG_DIR_NORMAL		0xa0
#define ST7565_CMD_SEG_DIR_REVERSE		0xa1

#define ST7565_CMD_EV_HEADER			0x81
#define ST7565_CMD_EV_VALUE(val)		((val) & 0x3f)

#define ST7565_CMD_NORMAL_DISPLAY		0xA6
#define ST7565_CMD_INVERSE_DISPLAY		0xA7

#define ST7565_CMD_BOOSTER_RATIO_HEADER		0xF8
#define ST7565_CMD_BOOSTER_RATIO_VALUE(val)	((val)&0x3)

#define ST7565_CMD_ALL_POINTS_NORMAL		0xa4
#define ST7565_CMD_ALL_POINT_ON			0xa5

/* time constants in ms */
#define ST7565_RESET_DELAY			1
#define ST7565_BUSY_DELAY			1

#endif /* __SSD16XX_REGS_H__ */
