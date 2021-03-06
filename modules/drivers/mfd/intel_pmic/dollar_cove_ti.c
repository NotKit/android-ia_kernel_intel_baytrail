/*
 * Dollar Cove  TI --  Device access for Intel PMIC for CR
 *
 * Copyright (c) 2014, Intel Corporation.
 *
 * Author: Ramakrishna Pallala <ramakrishna.pallala@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/mfd/core.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/mfd/intel_mid_pmic.h>
#include <linux/acpi.h>
#include <asm/intel_vlv2.h>
#include <linux/version.h>
#include "./pmic.h"

#define IRQLVL1		0x01
#define MIRQLVL1	0x02

enum {
	PWRBTN = 0,
	DIETMPWARN,
	ADCCMPL,
	VBATLOW = 4,
	VBUSDET,
	CCEOCAL =7,
};

static struct resource power_button_resources[] = {
	{
		.start	= PWRBTN,
		.end	= PWRBTN,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource thermal_resources[] = {
	{
		.start = DIETMPWARN,
		.end   = DIETMPWARN,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource adc_resources[] = {
	{
		.start = ADCCMPL,
		.end   = ADCCMPL,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource pwrsrc_resources[] = {
	{
		.start = VBUSDET,
		.end   = VBUSDET,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource battery_resources[] = {
	{
		.start = VBATLOW,
		.end   = VBATLOW,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = CCEOCAL,
		.end   = CCEOCAL,
		.flags = IORESOURCE_IRQ,
	},
};

static struct mfd_cell dollar_cove_ti_dev[] = {
	{
		.name = "dollar_cove_ti_adc",
		.id = 0,
		.num_resources = ARRAY_SIZE(adc_resources),
		.resources = adc_resources,
	},
	{
		.name = "dollar_cove_ti_power_button",
		.id = 0,
		.num_resources = ARRAY_SIZE(power_button_resources),
		.resources = power_button_resources,
	},
	{
		.name = "dollar_cove_ti_pwrsrc",
		.id = 0,
		.num_resources = ARRAY_SIZE(pwrsrc_resources),
		.resources = pwrsrc_resources,
	},
	{
		.name = "dollar_cove_ti_cc",
		.id = 0,
		.num_resources = ARRAY_SIZE(battery_resources),
		.resources = battery_resources,
	},
	{
		.name = "intel_fuel_gauge",
		.id = 0,
	},
	{
		.name = "intel_fg_iface",
		.id = 0,
	},
	{NULL, },
};

#define DOLLAR_COVE_IRQREGMAP(irq) \
	[irq] = { \
		{MIRQLVL1, irq, 1, 0}, \
		{IRQLVL1, irq, 1, INTEL_PMIC_REG_W1C}, \
		{IRQLVL1, irq, 1, INTEL_PMIC_REG_W1C}, \
	}

struct intel_pmic_irqregmap dollar_cove_ti_irqregmap[] = {
	DOLLAR_COVE_IRQREGMAP(PWRBTN),
	DOLLAR_COVE_IRQREGMAP(DIETMPWARN),
	DOLLAR_COVE_IRQREGMAP(ADCCMPL),
	DOLLAR_COVE_IRQREGMAP(VBATLOW),
	DOLLAR_COVE_IRQREGMAP(VBUSDET),
	DOLLAR_COVE_IRQREGMAP(CCEOCAL),
};

static int dollar_cove_ti_init(void)
{
	int reg_val=0;

	pr_info("Dollar Cove(TI: IC_TYPE 0x%02X\n", intel_mid_pmic_readb(0x00));
	pr_info("Forcing Sleep Mode Vnn to 0.5V \n");

	/* Write 0x14 to Register Buck5 VSleep Control: 0x2B */
	reg_val = intel_mid_pmic_readb(0x2B);
	if (reg_val < 0) {
		pr_err("Error reading BUCK5 VSLEEP %x\n", reg_val);
	}

	else {
		reg_val &= ~0xFE;
		reg_val |= 0x14;

		reg_val = intel_mid_pmic_writeb(0x2B, reg_val);
		if (reg_val < 0) {
			pr_err("Error setting VSLEEP Vnn to 0.5V %x\n", reg_val);
		}
	}

	return 0;
}

struct intel_mid_pmic dollar_cove_ti_pmic = {
	.label		= "dollar cove_ti",
	.irq_flags	= IRQF_TRIGGER_RISING | IRQF_ONESHOT,
	.init		= dollar_cove_ti_init,
	.cell_dev 	= dollar_cove_ti_dev,
	.irq_regmap	= dollar_cove_ti_irqregmap,
	.irq_num	= 8,
};

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com");
