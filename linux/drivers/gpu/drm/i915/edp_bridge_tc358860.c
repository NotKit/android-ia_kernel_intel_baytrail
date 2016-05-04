/*
 * Copyright ? 2008 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * Authors:
 *    Lingyan Guo <lingyan.guo@intel.com>
 *
 */


#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/input/mt.h>
#include <linux/lnw_gpio.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>
#include <drm/drmP.h>
#include "intel_drv.h"
#include <drm/i915_drm.h>
#include "i915_drv.h"
#include "intel_drv.h"
#include "intel_dsi.h"

#define GPIOC_2 2
#define GPIOC_3 3  //power EN
//#define SIO_PWM0 94
#define GPIO_SC_8 138  //new reset pin
static int hw_missing = 0;
static int init_done = 0;

static u8 fake_edid1[256] = {
        0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x52, 0x62, 0x88, 0x88, 0x00, 0x88, 0x88, 0x88,
        0x1c, 0x15, 0x01, 0x03, 0x80, 0x00, 0x00, 0x78, 0x0A, 0x0D, 0xC9, 0xA0, 0x57, 0x47, 0x98, 0x27,
        0x12, 0x48, 0x4C, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
        0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x30, 0x5c, 0x00, 0x68, 0x61, 0x00, 0x18, 0x80, 0xb4, 0x30,
        0xe2, 0x00, 0x00, 0x68, 0x00, 0x00, 0x00, 0x18, 0x30, 0x5c, 0x00, 0x68, 0x61, 0x00, 0x18, 0x80,
        0xb4, 0x30, 0xe2, 0x00, 0x00, 0x68, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x54,
        0x6f, 0x73, 0x68, 0x69, 0x62, 0x61, 0x2d, 0x55, 0x48, 0x32, 0x44, 0x0a, 0x00, 0x00, 0x00, 0xfd,
        0x00, 0x17, 0x3d, 0x0f, 0x8c, 0x17, 0x00, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0xa6,

        0x02, 0x03, 0x1a, 0x74, 0x47, 0x04, 0x13, 0x03, 0x02, 0x07, 0x06, 0x01, 0x23, 0x09, 0x07, 0x01,
        0x83, 0x01, 0x00, 0x00, 0x65, 0x03, 0x0c, 0x00, 0x10, 0x00, 0x30, 0x5c, 0x00, 0x68, 0x61, 0x00,
        0x18, 0x80, 0xb4, 0x30, 0xe2, 0x00, 0x00, 0x68, 0x00, 0x00, 0x00, 0x18, 0x30, 0x5c, 0x00, 0x68,
        0x61, 0x00, 0x18, 0x80, 0xb4, 0x30, 0xe2, 0x00, 0x00, 0x68, 0x00, 0x00, 0x00, 0x18, 0x30, 0x5c,
        0x00, 0x68, 0x61, 0x00, 0x18, 0x80, 0xb4, 0x30, 0xe2, 0x00, 0x00, 0x68, 0x00, 0x00, 0x00, 0x18,
        0x30, 0x5c, 0x00, 0x68, 0x61, 0x00, 0x18, 0x80, 0xb4, 0x30, 0xe2, 0x00, 0x00, 0x68, 0x00, 0x00,
        0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF4,
};

static struct i2c_client *ktd2151_client = NULL;
static struct i2c_client *tc358860_client = NULL;

static struct delayed_work tc358860_work;
/* Assume BIOS / bootloader has already enabled the converter at boot */
static int tc358860_enabled = 1;

static struct mutex tc358860_lock;

int tc358860_has_hw(void)
{
	if (!init_done) {
		pr_warning("%s: warning, hw not detected yet!", __func__);
		WARN_ON(1);
		/* Favor new display in case of error and assume HW exists */
		return 1;
	}
	return !hw_missing;
}

struct edid *tc358860_get_edid(void)
{
        u8 *block;

        DRM_DEBUG_KMS("use hardcode EDID\n");

        if ((block = kmalloc(256, GFP_KERNEL)) == NULL)
                return NULL;

        memcpy(block, fake_edid1, 256);

        return (struct edid *)block;
}

static int ktd2151_regr32(struct i2c_client *client, u16 reg, u32 *value)
{
	int r;
	u8 tx_data[] = {
		(reg >> 8) & 0xff,
		reg & 0xff,
	};

	u8 rx_data[4];

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = rx_data,
			.len = ARRAY_SIZE(rx_data),
		 },
	};

	r = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
		DRM_DEBUG_KMS("reg 0x%04x error %d\n", reg, r);
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		DRM_DEBUG_KMS("reg 0x%04x msgs %d\n", reg, r);
		return -EAGAIN;
	}

	*value = rx_data[3] << 24 | rx_data[2] << 16 |
		rx_data[1] << 8 | rx_data[0];

	//DRM_DEBUG_KMS("reg 0x%04x value 0x%08x\n", reg, *value);

	return 0;
}

static int ktd2151_regr16(struct i2c_client *client, u16 reg, u32 *value)
{
	int r;
	u8 tx_data[] = {
		(reg >> 8) & 0xff,
		reg & 0xff,
	};

	u8 rx_data[2];

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = rx_data,
			.len = ARRAY_SIZE(rx_data),
		 },
	};

	r = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
		DRM_DEBUG_KMS("reg 0x%04x error %d\n", reg, r);
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		DRM_DEBUG_KMS("reg 0x%04x msgs %d\n", reg, r);
		return -EAGAIN;
	}

	*value = rx_data[1] << 8 | rx_data[0];

	//DRM_DEBUG_KMS("reg 0x%04x value 0x%08x\n", reg, *value);

	return 0;
}
static int ktd2151_regr8(struct i2c_client *client, u16 reg, u32 *value)
{
	int r;
	u8 tx_data[] = {
		(reg >> 8) & 0xff,
		reg & 0xff,
	};

	u8 rx_data[1];

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = rx_data,
			.len = ARRAY_SIZE(rx_data),
		 },
	};

	r = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
		DRM_DEBUG_KMS("reg 0x%04x error %d\n", reg, r);
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		DRM_DEBUG_KMS("reg 0x%04x msgs %d\n", reg, r);
		return -EAGAIN;
	}

	*value = rx_data[0];

	//DRM_DEBUG_KMS("reg 0x%04x value 0x%08x\n", reg, *value);

	return 0;
}

#if 0
static int ktd2151_regw(struct i2c_client *client, u8 reg, u8 val)
{
        int ret;

        ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		DRM_DEBUG_KMS("reg 0x%04x val 0x%08x error %d\n", reg, val, ret);
		return ret;
	}

        return 0;
}
#endif

static int tc358860_regw8(struct i2c_client *client, u16 reg, u8 value)
{
	int r;

        u8 tx_data[] = {
                (reg >> 8) & 0xff,
                reg & 0xff,
                value & 0xff,
        };

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
	};

	r = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
		dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x error %d\n",
			__func__, reg, value, r);
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x msgs %d\n",
			__func__, reg, value, r);
		return -EAGAIN;
	}

	return 0;
}
#if 0
static int tc358860_regw16(struct i2c_client *client, u16 reg, u16 value)
{
	int r;

        u8 tx_data[] = {
                (reg >> 8) & 0xff,
                reg & 0xff,
                value & 0xff,
                (value >> 8) & 0xff,
        };

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
	};

	r = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
		dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x error %d\n",
			__func__, reg, value, r);
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x msgs %d\n",
			__func__, reg, value, r);
		return -EAGAIN;
	}

	return 0;
}
#endif
static int tc358860_regw32(struct i2c_client *client, u16 reg, u32 value)
{
	int r;

        u8 tx_data[] = {
                (reg >> 8) & 0xff,
                reg & 0xff,
                value & 0xff,
                (value >> 8) & 0xff,
                (value >> 16) & 0xff,
                (value >> 24) & 0xff,
        };

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
	};

	r = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
		dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x error %d\n",
			__func__, reg, value, r);
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x msgs %d\n",
			__func__, reg, value, r);
		return -EAGAIN;
	}

	return 0;
}

void tc358860_bridge_disable(struct drm_device *dev)
{
        struct drm_i915_private *dev_priv = dev->dev_private;

        DRM_DEBUG_KMS("\n");
        if(hw_missing)
                return;

        if (!tc358860_enabled)
                return;

        mutex_lock(&tc358860_lock);
        /* backlight off command 0x53 parameter 00h */
        tc358860_regw32(tc358860_client, 0x42fc, 0x80005315);
        /* display off command 0x28 */
        tc358860_regw32(tc358860_client, 0x42fc, 0x80002805);
        /* 100ms sleep. */
        msleep(100);
        /* sleep in command 0x10 */
        tc358860_regw32(tc358860_client, 0x42fc, 0x80001005);
        /* 150ms sleep. */
        msleep(150);

        /* DSI video transfer stop */
        vlv_gpio_nc_write(dev_priv, GPIO_NC_10_PCONF0, 0x2000CC00);
        vlv_gpio_nc_write(dev_priv, GPIO_NC_10_PAD, 0x00000004);

        /* lcd reset low */
        gpio_set_value(GPIO_SC_8, 0);
        /* 10 ms sleep */
        usleep_range(10000, 10001);

        /* VSN OFF */
        vlv_gpio_nc_write(dev_priv, GPIO_NC_19_PCONF0, 0x2000CC00);
        vlv_gpio_nc_write(dev_priv, GPIO_NC_19_PAD, 0x00000004);

        /* 10 ms sleep */
        usleep_range(10000, 10001);

        /* VSP OFF */
        vlv_gpio_nc_write(dev_priv, GPIO_NC_20_PCONF0, 0x2000CC00);
        vlv_gpio_nc_write(dev_priv, GPIO_NC_20_PAD, 0x00000004);
        usleep_range(10000, 10001);

        /* disable IOVCC */
        gpio_set_value(GPIOC_3, 0);

        /* edp to mipi chip in reset */
        vlv_gpio_nc_write(dev_priv, GPIO_NC_16_PCONF0, 0x2000CC00);
        vlv_gpio_nc_write(dev_priv, GPIO_NC_16_PAD, 0x00000004);

        tc358860_enabled = 0;
        mutex_unlock(&tc358860_lock);
}

void tc358860_bridge_enable(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
        u32 data = 0;

        DRM_DEBUG_KMS("\n");
	if(hw_missing)
		return;

	if (tc358860_enabled)
		return;

	mutex_lock(&tc358860_lock);

        /* lcd reset low */
        gpio_set_value(GPIO_SC_8, 0);

	/* IOVCC on */
	gpio_set_value(GPIOC_3, 1);
	/* sleep 10ms */
        usleep_range(10000, 10001);

	/* VSP ON */
        vlv_gpio_nc_write(dev_priv, GPIO_NC_20_PCONF0, 0x2000CC00);
        vlv_gpio_nc_write(dev_priv, GPIO_NC_20_PAD, 0x00000005);

	/* sleep 10ms */
        usleep_range(10000, 10001);

	/* VSN ON */
        vlv_gpio_nc_write(dev_priv, GPIO_NC_19_PCONF0, 0x2000CC00);
        vlv_gpio_nc_write(dev_priv, GPIO_NC_19_PAD, 0x00000005);
	/* should be 20ms sleep */
        usleep_range(20000, 20001);

	/* reset to L has been done earlier, reset to H now */
	gpio_set_value(GPIO_SC_8, 1);
	/* sleep after reset L-> H 10 ms */
        usleep_range(10000, 10001);
	/* should be reset H -> L */
	gpio_set_value(GPIO_SC_8, 0);
	/* 10 ms wait  */
        usleep_range(10000, 10001);
	/* reset release L -> H missing */
	gpio_set_value(GPIO_SC_8, 1);
	/* spec states there should be a 30ms delay here. However this causes
	   visual artefacts (grey screen) which dissappear when removed.
	   Also the delays involved getting the EDP to MIPI chip
	   out of reset are likely to cover the delay.
	 */

	/* EDP to MIPI chip out of reset */
        vlv_gpio_nc_write(dev_priv, GPIO_NC_16_PCONF0, 0x2000CC00);
        vlv_gpio_nc_write(dev_priv, GPIO_NC_16_PAD, 0x00000005);
        usleep_range(10000, 10001);

	/* DSI video mode transfer start? */
        ktd2151_regr32(tc358860_client, 0x0180, &data);
        DRM_DEBUG_KMS("data: 0x%x\n", data);

	tc358860_enabled = 1;
	mutex_unlock(&tc358860_lock);

	/* rest of screen enable is handled in cmd1, cmd2 and cmd3
	   * video on happens in cmd1
	   * sleep out and display on in cmd2
	   * display pass-through in cmd3
	*/

}

#if 0
static int tc358860_bridge_probe(struct i2c_client *client,
                const struct i2c_device_id *id)
{
        DRM_DEBUG_KMS("\n");
        tc358860_client = client;
        return 0;
}

static int tc358860_bridge_remove(struct i2c_client *client)
{
        DRM_DEBUG_KMS("\n");
        tc358860_client = NULL;

        return 0;
}
#endif

int tc358860_read_reg(int addr, u32 compare_value, int bit, int value_len)
{
        u32 data = 0;
        int i = 0;

        for (i = 0; i < 5; i++) {
                if (value_len == 8)
                        ktd2151_regr8(tc358860_client, addr, &data);
                else if (value_len == 16)
                        ktd2151_regr16(tc358860_client, addr, &data);
                else
                        ktd2151_regr32(tc358860_client, addr, &data);

                if (((data & bit) != compare_value) && (compare_value != 0xff)) {
                        usleep_range(1000, 2000);
                        continue;
                } else
                        break;
        }

        if (i >= 5) {
                DRM_DEBUG_KMS("read reg: 0x%x error: 0x%x\n", addr, data & bit);
                return -1;
        }
        DRM_DEBUG_KMS("read reg: 0x%x value: 0x%x\n", addr, data);

        return 0;
}

int tc358860_read_reg1(int addr, u32 compare_val1,
                u32 compare_val2, int bit, int value_len)
{
        u32 data = 0;
        int i = 0;

        for (i = 0; i < 5; i++) {
                if (value_len == 8)
                        ktd2151_regr8(tc358860_client, addr, &data);
                else
                        ktd2151_regr32(tc358860_client, addr, &data);

                if (((data & bit) != compare_val1) &&
                        ((data&bit) != compare_val2)) {
                        usleep_range(1000, 2000);
                        continue;
                } else
                        break;
        }

        if (i >= 5) {
                DRM_DEBUG_KMS("read reg: 0x%x error: 0x%x\n", addr, data);
                return -1;
        }
        DRM_DEBUG_KMS("0x%x: 0x%x\n", addr, data);

        return 0;
}
void tc358860_send_init_cmd1(struct intel_dp *intel_dp)
{

        if (!i2c_check_functionality(tc358860_client->adapter, I2C_FUNC_I2C)) {
                DRM_DEBUG_KMS("i2c_check_functionality() failed\n");
                return;
        } else {
                DRM_DEBUG_KMS("i2c_check_functionality() ok\n");
        }

	if(hw_missing || !init_done)
		return;

        //io voltage setting
        tc358860_regw32(tc358860_client, 0x0800, 0x0001);
        //boot settings
        tc358860_regw32(tc358860_client, 0x1000, 0x6978);
        tc358860_regw32(tc358860_client, 0x1004, 0x4090d);
        tc358860_regw32(tc358860_client, 0x1008, 0x3b4000e);
        tc358860_regw32(tc358860_client, 0x100c, 0x300030b);
        tc358860_regw32(tc358860_client, 0x1010, 0xd90020);
        tc358860_regw32(tc358860_client, 0x1014, 0x0001);

        usleep_range(1000, 1001);
        tc358860_read_reg(0x1018, 0x02, 7, 32);

        tc358860_regw8(tc358860_client, 0xb005, 0x16);
        tc358860_regw8(tc358860_client, 0xb006, 0x00);
        tc358860_regw8(tc358860_client, 0xb007, 0x11);
        tc358860_regw8(tc358860_client, 0xb008, 0x00);
        tc358860_regw8(tc358860_client, 0xb009, 0x21);
        tc358860_regw8(tc358860_client, 0xb00a, 0x0c);

        tc358860_regw32(tc358860_client, 0x41b0, 0x42293);
        tc358860_regw32(tc358860_client, 0x41bc, 0xa01);
        tc358860_regw32(tc358860_client, 0x41c0, 0x30);
        tc358860_regw32(tc358860_client, 0x41a8, 0x6590);
        tc358860_regw32(tc358860_client, 0x1014, 0x0003);

        usleep_range(1000, 1001);
        tc358860_read_reg(0x1018, 0x06, 7, 32);

        //Additional setting for eDP
        tc358860_regw8(tc358860_client, 0x8003, 0x41);
        tc358860_regw8(tc358860_client, 0xb400, 0x0d);

        //DPRX CAD Register Setting
        tc358860_regw8(tc358860_client, 0xb88e, 0xff);
        tc358860_regw8(tc358860_client, 0xb88f, 0xff);
        tc358860_regw8(tc358860_client, 0xb89a, 0xff);
        tc358860_regw8(tc358860_client, 0xb89b, 0xff);
        tc358860_regw8(tc358860_client, 0xb800, 0x0e);
        tc358860_regw8(tc358860_client, 0xbb26, 0x02);
        tc358860_regw8(tc358860_client, 0xbb01, 0x20);
        tc358860_regw8(tc358860_client, 0xb8c0, 0xf1);
        tc358860_regw8(tc358860_client, 0xb8c1, 0xf1);
        tc358860_regw8(tc358860_client, 0xb8c2, 0xf0);
        tc358860_regw8(tc358860_client, 0xb8c3, 0xf0);
        tc358860_regw8(tc358860_client, 0xb8c4, 0xf0);
        tc358860_regw8(tc358860_client, 0xb8c5, 0xf0);
        tc358860_regw8(tc358860_client, 0xb8c6, 0xf0);
        tc358860_regw8(tc358860_client, 0xb8c7, 0xf0);
        tc358860_regw8(tc358860_client, 0xb80b, 0x00);
        tc358860_regw8(tc358860_client, 0xb833, 0x00);
        tc358860_regw8(tc358860_client, 0xb85b, 0x00);
        tc358860_regw8(tc358860_client, 0xb810, 0x00);
        tc358860_regw8(tc358860_client, 0xb838, 0x00);
        tc358860_regw8(tc358860_client, 0xb860, 0x00);
        tc358860_regw8(tc358860_client, 0xb815, 0x00);
        tc358860_regw8(tc358860_client, 0xb83d, 0x00);
        tc358860_regw8(tc358860_client, 0xb865, 0x00);
        tc358860_regw8(tc358860_client, 0xb81a, 0x00);
        tc358860_regw8(tc358860_client, 0xb842, 0x00);
        tc358860_regw8(tc358860_client, 0xb86a, 0x00);
        tc358860_regw8(tc358860_client, 0xb81f, 0x00);
        tc358860_regw8(tc358860_client, 0xb847, 0x00);
        tc358860_regw8(tc358860_client, 0xb86f, 0x00);
        tc358860_regw8(tc358860_client, 0xb824, 0x00);
        tc358860_regw8(tc358860_client, 0xb84c, 0x00);
        tc358860_regw8(tc358860_client, 0xb874, 0x00);
        tc358860_regw8(tc358860_client, 0xb829, 0x00);
        tc358860_regw8(tc358860_client, 0xb851, 0x00);
        tc358860_regw8(tc358860_client, 0xb879, 0x00);
        tc358860_regw8(tc358860_client, 0xb82e, 0x00);
        tc358860_regw8(tc358860_client, 0xb856, 0x00);
        tc358860_regw8(tc358860_client, 0xb87e, 0x00);
        tc358860_regw8(tc358860_client, 0xbb90, 0x10);
        tc358860_regw8(tc358860_client, 0xbb91, 0x0f);
        tc358860_regw8(tc358860_client, 0xbb92, 0xf6);
        tc358860_regw8(tc358860_client, 0xbb93, 0x10);
        tc358860_regw8(tc358860_client, 0xbb94, 0x0f);
        tc358860_regw8(tc358860_client, 0xbb95, 0xf6);
        tc358860_regw8(tc358860_client, 0xbb96, 0x10);
        tc358860_regw8(tc358860_client, 0xbb97, 0x0f);
        tc358860_regw8(tc358860_client, 0xbb98, 0xf6);
        tc358860_regw8(tc358860_client, 0xbb99, 0x10);
        tc358860_regw8(tc358860_client, 0xbb9a, 0x0f);
        tc358860_regw8(tc358860_client, 0xbb9b, 0xf6);
        tc358860_regw8(tc358860_client, 0xb88a, 0x03);
        tc358860_regw8(tc358860_client, 0xb896, 0x03);
        tc358860_regw8(tc358860_client, 0xbbd1, 0x07);
        tc358860_regw8(tc358860_client, 0xbbb0, 0x07);
        tc358860_regw8(tc358860_client, 0xb88b, 0x04);
        tc358860_regw8(tc358860_client, 0xb88c, 0x45);
        tc358860_regw8(tc358860_client, 0xb88d, 0x05);
        tc358860_regw8(tc358860_client, 0xb897, 0x04);
        tc358860_regw8(tc358860_client, 0xb898, 0xe0);
        tc358860_regw8(tc358860_client, 0xb899, 0x2e);
        tc358860_regw8(tc358860_client, 0x800e, 0x00);
        tc358860_regw32(tc358860_client, 0x1014, 0x07);

        usleep_range(1000, 1001);
        tc358860_read_reg(0x1018, 0x07, 7, 32);

        usleep_range(1000, 1001);
        //eDP Setting for Link Training
        tc358860_read_reg(0xb631, 0x01, 3, 8);

        tc358860_regw8(tc358860_client, 0x8001, 0x0a);
        tc358860_regw8(tc358860_client, 0x8002, 0x04);
        tc358860_regw8(tc358860_client, 0xb608, 0x0b);
        tc358860_regw8(tc358860_client, 0xb800, 0x1e);
        tc358860_regw8(tc358860_client, 0x0700, 0x00);

        usleep_range(1000, 1001);
}

void tc358860_send_init_cmd2(struct intel_dp *intel_dp)
{
        DRM_DEBUG_KMS("\n");

	if(hw_missing || !init_done)
		return;

#if 0
        //Check Link Training Status
        tc358860_read_reg(0x8100, 0x0a, 0xff, 8);
        tc358860_read_reg(0x8202, 0x77, 0xff, 8);
        tc358860_read_reg(0x8203, 0x77, 0xff, 8);
        tc358860_read_reg1(0x8204, 0x81, 0x01, 0xff, 8);
	tc358860_read_reg(0x8101, 0x84, 0xff, 8);
        tc358860_read_reg(0x8206, 0xff, 0xff, 8);
        tc358860_read_reg(0x8207, 0xff, 0xff, 8);
	tc358860_read_reg(0x8103, 0xff, 0xff, 8);
	tc358860_read_reg(0x8104, 0xff, 0xff, 8);
	tc358860_read_reg(0x8105, 0xff, 0xff, 8);
	tc358860_read_reg(0x8106, 0xff, 0xff, 8);
#endif

        //dsi start
        tc358860_regw32(tc358860_client, 0x407c, 0x81);
        tc358860_regw32(tc358860_client, 0x4050, 0x00);
        tc358860_regw32(tc358860_client, 0x401c, 0x01);

        usleep_range(1000, 1001);
        tc358860_read_reg(0x4060, 0x03, 7, 32);

	/* send sleep out command 0x11 */
        tc358860_regw32(tc358860_client, 0x42fc, 0x80001105);
	/* wait for a frame */
	usleep_range(1000, 1001);
        tc358860_read_reg(0x4200, 0x01, 7, 32);

	/* display on command 0x29 */
        tc358860_regw32(tc358860_client, 0x42fc, 0x80002905);
        usleep_range(1000, 1001);
        tc358860_read_reg(0x4200, 0x01, 7, 32);

        tc358860_regw32(tc358860_client, 0x2a10, 0x80040010);
        tc358860_regw32(tc358860_client, 0x3a10, 0x80040010);
        tc358860_regw32(tc358860_client, 0x2a04, 0x01);
        tc358860_regw32(tc358860_client, 0x3a04, 0x01);
}

void tc358860_cmd3_work(struct work_struct *work)
{
        DRM_DEBUG_KMS("\n");

	if(hw_missing || !init_done)
		return;
        tc358860_regw32(tc358860_client, 0x0154, 0x01);
#if 0
        tc358860_read_reg(0xb228, 0x0f50, 0xffff, 16);
        tc358860_read_reg(0xb22a, 0x0880, 0xffff, 16);
        tc358860_read_reg(0xb22c, 0x8020, 0xffff, 16);
        tc358860_read_reg(0xb22e, 0x0048, 0xffff, 16);
        tc358860_read_reg(0xb230, 0x0008, 0xffff, 16);
        tc358860_read_reg(0xb232, 0x8002, 0xffff, 16);
        tc358860_read_reg(0xb234, 0x0f00, 0xffff, 16);
        tc358860_read_reg(0xb236, 0x0870, 0xffff, 16);
        tc358860_read_reg(0x8202, 0x77, 0xff, 8);
        tc358860_read_reg(0x8203, 0x77, 0xff, 8);
        tc358860_read_reg1(0x8204, 0x81, 0x01, 0xff, 8);
        tc358860_read_reg(0x810a, 0xff, 0xff, 8);
        tc358860_read_reg(0x8206, 0xff, 0xff, 8);
        tc358860_read_reg(0x8207, 0xff, 0xff, 8);
        tc358860_read_reg(0x8103, 0xff, 0xff, 8);
        tc358860_read_reg(0x8104, 0xff, 0xff, 8);
        tc358860_read_reg(0x8105, 0xff, 0xff, 8);
        tc358860_read_reg(0x8106, 0xff, 0xff, 8);
	tc358860_read_reg(0x8101, 0x84, 0xff, 8);
#endif
}

void tc358860_send_init_cmd3(void)
{
	schedule_delayed_work(&tc358860_work, 0);
}

static struct i2c_client *register_i2c_device(int bus, int addr, char *name)
{
        struct i2c_adapter *adapter;
        struct i2c_client *client;
        struct i2c_board_info info;

        memset(&info, 0, sizeof(info));
        strlcpy(info.type, name, I2C_NAME_SIZE);
        adapter = i2c_get_adapter(bus);
        if (!adapter) {
                printk(KERN_ERR "Err: invalid i2c adapter %d\n", bus);
                return NULL;
        } else {
                info.addr = addr;
                client = i2c_new_device(adapter, &info);
                if (!client) {
                        printk(KERN_ERR "Fail to add i2c device in tc %d:%d\n",
                                        bus, addr);
                        return NULL;
                }
        }


        return client;
}

static const struct i2c_device_id ktd2151_id[] = {
	{ "i2c_ktd2151", 0 },
	{ }
};

struct i2c_driver ktd2151_voltage_i2c_driver = {
	.driver = {
		.name = "i2c_ktd2151",
	},
	.id_table = ktd2151_id,
};

static const struct i2c_device_id tc358860_bridge_id[] = {
	{ "i2c_disp_brig", 0 },
	{ }
};
struct i2c_driver tc358860_bridge_i2c_driver = {
	.driver = {
		.name = "i2c_disp_brig",
	},
	.id_table = tc358860_bridge_id,
};


int tc358860_init(struct drm_device *dev)
{
	int ret = 0;
	int err = 0;

	//struct drm_i915_private *dev_priv = dev->dev_private;

	DRM_DEBUG_KMS("\n");

       // err = gpio_request(GPIOC_2, "tc358860_vdd1v1");
       // if (err < 0) {
        //        printk(KERN_ERR "%s:failed to set gpio reset.\n",
       //                         __func__);
        //        return -1;
       // }
      //  gpio_direction_output(GPIOC_2, 0);

        err = gpio_request(GPIOC_3, "tc358860_pwren");
        if (err < 0) {
                printk(KERN_ERR "%s:failed to set gpio pwren.\n",
                                __func__);
                return -1;
        }
        gpio_direction_output(GPIOC_3, 1);

        err = gpio_request(GPIO_SC_8, "tc358860_panelreset");
        if (err < 0) {
                printk(KERN_ERR "%s:failed to set gpio panelreset.\n",
                                __func__);
                return -1;
        }
        gpio_direction_output(GPIO_SC_8, 1);

#if 0
        err = gpio_request(SIO_PWM0, "tc358860_vdd1v8");
        if (err < 0) {
                printk(KERN_ERR "%s:failed to set gpio reset.\n",
                                __func__);
                return -1;
        }
        gpio_direction_output(SIO_PWM0, 0);
#endif

	mutex_init(&tc358860_lock);

        ktd2151_client = register_i2c_device(3, 0x3e, "i2c_ktd2151");
        if (ktd2151_client == NULL) {
                printk(KERN_ERR "Fail get i2c device\n");
		hw_missing = 1;
                return -1;
        }

	printk(KERN_ERR "ktd2151_client = %p\n", (void *)ktd2151_client);

	ret = i2c_add_driver(&ktd2151_voltage_i2c_driver);
	if (ret) {
		DRM_ERROR("add bridge I2C driver faild\n");
		return -EINVAL;
	}

        tc358860_client = register_i2c_device(4, 0x68, "i2c_disp_brig");
        if (tc358860_client == NULL) {
                printk(KERN_ERR "Fail get i2c device\n");
                return -1;
        }

	ret = i2c_add_driver(&tc358860_bridge_i2c_driver);
	if (ret) {
		DRM_ERROR("add bridge I2C driver faild\n");
		return -EINVAL;
	}

        if(tc358860_regw32(tc358860_client, 0x0800, 0x0001) != 0)
	{
		hw_missing = 1;
		printk(KERN_ERR "Setting hw_missing to 1\n");
	}

	init_done = 1;
        INIT_DELAYED_WORK(&tc358860_work, tc358860_cmd3_work);

	return 0;
}
