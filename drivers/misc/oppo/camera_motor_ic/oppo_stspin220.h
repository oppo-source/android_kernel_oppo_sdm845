/************************************************************************************
** Copyright (C), 2008-2017, OPPO Mobile Comm Corp., Ltd
** VENDOR_EDIT
** File: oppo_drv8834.h
**
** Description:
**      Definitions for m1120 motor driver ic drv8834.
**
** Version: 1.0
** Date created: 2018/01/14,20:27
**
** --------------------------- Revision History: -------------------------------------
* <version>		<date>		<author>		<desc>
**************************************************************************************/
#ifndef __OPPO_STSPIN220_H__
#define __OPPO_STSPIN220_H__

#define  RATIO_A 24 //2.4
#define  RATIO_B 1806 //18.06
#define  RATIO_B_FI_6 354 //3.54
#define  RATIO_C 20
#define  RATIO_D 32

enum {
	GPIO_MODE = 0,
	HIGH_IMPEDANCE_MODE
};

struct oppo_sts_chip {
	struct device	*dev;
	struct pwm_device *pwm_dev;
	struct pinctrl *pctrl;
	struct pinctrl_state *pwm_state;
	struct pinctrl_state *boost_state;
	struct pinctrl_state *enable_state;
	struct pinctrl_state *sleep_state;
	struct pinctrl_state *dir_state;
	struct pinctrl_state *dir_switch_state;
	unsigned int boost_gpio;
	unsigned int sleep_gpio;
	unsigned int sleep1_gpio;
	unsigned int dir_gpio;
	unsigned int enable_gpio;
	unsigned int step_gpio;
	unsigned int dir_switch_gpio;
	int dir_switch;
	int motor_type;
};

#endif // __OPPO_DRV8834_H__

