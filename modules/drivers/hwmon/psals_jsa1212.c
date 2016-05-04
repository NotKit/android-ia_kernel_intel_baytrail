/*
 * JSA01212 Sensor Driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/sysfs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/acpi_gpio.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <asm/div64.h>

/*i2c registers*/
#define REG_CONFIGURE		0x01
#define REG_INTERRUPT		0x02
#define REG_PROX_LT		0x03
#define REG_PROX_HT		0x04
#define REG_ALSIR_TH1		0x05
#define REG_ALSIR_TH2		0x06
#define REG_ALSIR_TH3		0x07
#define REG_PROX_DATA		0x08
#define REG_ALSIR_DT1		0x09
#define REG_ALSIR_DT2		0x0a
#define REG_ALS_RNG		0x0b
#define REG_UNKN1		0x0e
#define REG_UNKN2		0x0f

/* JSA1212 ALS RNG REG bits */
#define JSA1212_ALS_RNG_0_2048          0x00
#define JSA1212_ALS_RNG_0_1024          0x01
#define JSA1212_ALS_RNG_0_512           0x02
#define JSA1212_ALS_RNG_0_256           0x03
#define JSA1212_ALS_RNG_0_128           0x04
#define JSA1212_ALS_RNG_COUNT           0x05

/* JSA1212 INT REG bits */
#define JSA1212_INT_CTRL_EITHER         0x00
#define JSA1212_INT_CTRL_BOTH           0x01
#define JSA1212_INT_CTRL_MASK           0x01

#define JSA1212_INT_ALS_PRST_1CONV      0x00
#define JSA1212_INT_ALS_PRST_4CONV      0x02
#define JSA1212_INT_ALS_PRST_8CONV      0x04
#define JSA1212_INT_ALS_PRST_16CONV     0x06
#define JSA1212_INT_ALS_PRST_MASK       0x06

#define JSA1212_INT_ALS_FLAG            0x08

#define JSA1212_INT_PXS_UNKN1           0x10

#define JSA1212_INT_PXS_PRST_1CONV      0x00
#define JSA1212_INT_PXS_PRST_4CONV      0x20
#define JSA1212_INT_PXS_PRST_8CONV      0x40
#define JSA1212_INT_PXS_PRST_16CONV     0x60
#define JSA1212_INT_PXS_PRST_MASK       0x60

#define JSA1212_INT_PXS_FLAG            0x80

/* JSA1212 CONF REG bits */
#define JSA1212_CONF_UNKN1              0x01
#define JSA1212_CONF_UNKN2              0x02

#define JSA1212_CONF_ALS_DISABLE        0x00
#define JSA1212_CONF_ALS_ENABLE         0x04
#define JSA1212_CONF_ALS_MASK           0x04

#define JSA1212_CONF_IRDR_200MA         0x08
#define JSA1212_CONF_IRDR_100MA         0x00
#define JSA1212_CONF_IRDR_MASK          0x08

#define JSA1212_CONF_PXS_SLP_800MS      0x00
#define JSA1212_CONF_PXS_SLP_400MS      0x10
#define JSA1212_CONF_PXS_SLP_200MS      0x20
#define JSA1212_CONF_PXS_SLP_100MS      0x30
#define JSA1212_CONF_PXS_SLP_75MS       0x40
#define JSA1212_CONF_PXS_SLP_50MS       0x50
#define JSA1212_CONF_PXS_SLP_12MS       0x60
#define JSA1212_CONF_PXS_SLP_0MS        0x70
#define JSA1212_CONF_PXS_SLP_MASK       0x70

#define JSA1212_CONF_PXS_DISABLE        0x00
#define JSA1212_CONF_PXS_ENABLE         0x80
#define JSA1212_CONF_PXS_MASK           0x80

/*INT_GPIO*/
#define	INT_GPIO 		136

#define DRIVER_NAME 		"JSA01212:00"
#define PS_INPUT_NAME		"jsa1212_ps"
#define ALS_INPUT_NAME		"jsa1212_als"

#define CONFIG_JSA1212_DEBUG
#ifdef CONFIG_JSA1212_DEBUG
static unsigned int debug_level = 0;
#define DBG_LEVEL1		1
#define DBG_LEVEL2		2
#define DBG_LEVEL3		3
#define DBG_LEVEL4		4
#define SENSOR_DBG(level, fmt, ...)			\
do {							\
	if (level <= debug_level)			\
		printk(KERN_DEBUG "<jsa1212>[%d]%s  "	\
			 fmt "\n",			\
			__LINE__, __func__,		\
			##__VA_ARGS__);			\
} while (0)

#else
#define SENSOR_DBG(level, ...)
#endif

/*Private data for each sensor*/
struct sensor_data {
	struct i2c_client *client;
	struct input_dev *input_ps;
	struct input_dev *input_als;
	struct mutex lock;
	/*used to detect ps far event*/
	struct delayed_work work_ps;
#define PS_WORK_INTERVAL		500
	int interval;
#define PS_NEAR_FAR_THRESHOLD		0xc8
	int ps_threshold;
	int gpio_int;
	int irq;

#define JSA1212_STATE_PS_DISABLED	(0)
#define JSA1212_STATE_PS_ENABLED	(1<<0)

#define JSA1212_STATE_ALS_DISABLED	(0)
#define JSA1212_STATE_ALS_ENABLED	(1<<1)
	int state;
	int state_suspend;

}jsa1212_data;

/* Calibration: output_lux = sensor_lux * scaling_factor */
#define ALS_SCALE_MULTIPLIER	8500
#define ALS_SCALE_DIVIDER	1000

/* Value range: sensor_raw is unsigned 12-bit value */
#define ALS_INPUT_MINVAL	0x000
#define ALS_INPUT_MAXVAL	0xfff

/* Value range: maximum unscaled lux value sensor can give */
#define ALS_INPUT_MAXLUX	0x7ff

/* Evdev range visible via EVIOCGABS ioctl */
#define ALS_OUTPUT_MINVAL	0x0000
#define ALS_OUTPUT_MAXVAL	0xffff

/* Sensitivity: +percent_of_current or -absolute_change */
static int als_threshold = 2;

/* Measurement ranges for the als */
static const struct
{
	int maxval;
} als_ranges[JSA1212_ALS_RNG_COUNT] =
{
	[JSA1212_ALS_RNG_0_2048] = { ALS_INPUT_MAXLUX /  1 },
	[JSA1212_ALS_RNG_0_1024] = { ALS_INPUT_MAXLUX /  2 },
	[JSA1212_ALS_RNG_0_512]  = { ALS_INPUT_MAXLUX /  4 },
	[JSA1212_ALS_RNG_0_256]  = { ALS_INPUT_MAXLUX /  8 },
	[JSA1212_ALS_RNG_0_128]  = { ALS_INPUT_MAXLUX / 16 },
};

/* Currently active als measurement range */
static u8 als_range = JSA1212_ALS_RNG_0_2048;

/* Calibration: the scaling factor can be changed via sysfs */
static unsigned als_scale_multiplier = ALS_SCALE_MULTIPLIER;

/* Convert raw sensor value to uncalibrated lux value */
static int als_normalize_input(int input_raw)
{
	return input_raw * als_ranges[als_range].maxval / ALS_INPUT_MAXVAL;
}

/* Convert raw sensor value to calibrated lux value */
static int als_calculate_output(int input_raw)
{
	u64 range_max = als_ranges[als_range].maxval;
	u64 multiplier = range_max * als_scale_multiplier;
	u64 divisor = ALS_INPUT_MAXVAL * ALS_SCALE_DIVIDER;
	/* To preserve "zero lux", roundup at 7/8 instead of 1/2 */
	u64 roundup = divisor / 8;
	u64 value = (input_raw * multiplier + roundup) / divisor;

	return (int)value;
}

/* Select appropriate measurement range based on current sensor reading */
static int als_range_select(int input_lux)
{
	unsigned old = als_range;
	unsigned use = 0;
	unsigned i;

	for (i = 1; i < JSA1212_ALS_RNG_COUNT; ++i) {
		if (input_lux < als_ranges[i].maxval * 3 / 4)
			use = i;
	}

	return als_range = (u8)use, old != use;
}

static int sensor_read(struct sensor_data *data, u8 reg, u8 len, u8 *val)
{
	int ret;

	if (len > 1) {
		ret = i2c_smbus_read_i2c_block_data(data->client,
						reg, len, val);
		if (ret < 0)
			dev_err(&data->client->dev, "Err: read reg %02x=%02x\n",
							reg, ret);
		return ret;
	} else {

		ret = i2c_smbus_read_byte_data(data->client, reg);
		if (ret >= 0) {
			SENSOR_DBG(DBG_LEVEL3, "read i2c reg %02x=%02x",
								reg, ret);
			*val = ret;
			return 0;
		} else {
			dev_err(&data->client->dev,
				"Err: return %d when read i2c reg %02x\n",
				ret, reg);
			return ret;
		}
	}
}

static int sensor_write(struct sensor_data *data, u8 reg, u8 val)
{
	int ret;

	SENSOR_DBG(DBG_LEVEL3, "write i2c reg %02x=%02x", reg, val);

	ret = i2c_smbus_write_byte_data(data->client, reg, val);
	if (ret < 0) {
		dev_err(&data->client->dev,
			"Err: return %d when write i2c reg %02x=%02x\n",
			ret, reg, val);
	}
	return ret;
}
/*
  * Reset detection and Auto recovery. This patch comes from vendor to fix
  * light sensor work abnormal sometimes.
  */
#define CONFIG_JSA1212_AUTO_RECOVERY
static int sensor_init(struct sensor_data *data)
{
	int ret = 0;
#ifdef CONFIG_JSA1212_AUTO_RECOVERY
       u8 count;
#endif

	SENSOR_DBG(DBG_LEVEL1, "%s", data->client->name);

	ret = sensor_write(data, REG_CONFIGURE, JSA1212_CONF_PXS_SLP_100MS);
	if (ret < 0)
		return ret;

#ifdef CONFIG_JSA1212_AUTO_RECOVERY
	for (count = 0; count < 100; ++count) {
		u8 res = 0xff;
		/* Unlock Reg Test1 */
		sensor_write(data, REG_UNKN2, 0x29);
		/* Turn off test mode */
		sensor_write(data, REG_UNKN1, 0x00);
		/* Read Reg REG Test1 */
		sensor_read(data, REG_UNKN1, 1, &res);
		if (res == 0x00)
			break;
	}
	/* Lock Reg Test1 */
	sensor_write(data, REG_UNKN2, 0x00);
#endif

	ret = sensor_write(data, REG_INTERRUPT,
			   JSA1212_INT_PXS_PRST_4CONV |
			   JSA1212_INT_ALS_PRST_4CONV);
	if (ret < 0)
		return ret;

	return sensor_write(data, REG_ALS_RNG, als_range);
}

static int sensor_set_mode(struct sensor_data *data, int state)
{
	int ret = 0;
	u8 val;

	SENSOR_DBG(DBG_LEVEL2, "%s change state from %08x to %08x",
			data->client->name, data->state, state);

	if ((data->state ^ state) & JSA1212_STATE_PS_ENABLED) {
		if (state & JSA1212_STATE_PS_ENABLED) {
			SENSOR_DBG(DBG_LEVEL3, "enable ps");

			ret = sensor_write(data, REG_PROX_LT,
						data->ps_threshold);
			if (ret < 0)
				goto out;
			ret = sensor_write(data, REG_PROX_HT,
						data->ps_threshold);
			if (ret < 0)
				goto out;

			ret = sensor_read(data, REG_CONFIGURE, 1, &val);
			if (ret < 0)
				goto out;
			val |= JSA1212_CONF_PXS_ENABLE;
			ret = sensor_write(data, REG_CONFIGURE, val);
			if (ret < 0)
				goto out;

			data->state |= JSA1212_STATE_PS_ENABLED;
		} else {
			SENSOR_DBG(DBG_LEVEL3, "disable ps");

			ret = sensor_read(data, REG_CONFIGURE, 1, &val);
			if (ret < 0)
				goto out;
			val &= ~JSA1212_CONF_PXS_ENABLE;
			ret = sensor_write(data, REG_CONFIGURE, val);
			if (ret < 0)
				goto out;

			data->state &= ~JSA1212_STATE_PS_ENABLED;
		}
	}

	if ((data->state ^ state) & JSA1212_STATE_ALS_ENABLED) {
		if (state & JSA1212_STATE_ALS_ENABLED) {
			u8 status;

			SENSOR_DBG(DBG_LEVEL3, "enable als");

			/* Report on every als duty cycle when enabling */
			ret = sensor_read(data, REG_INTERRUPT, 1, &status);
			if (ret < 0)
				goto out;
			status &= ~JSA1212_INT_ALS_PRST_MASK;
			status |= JSA1212_INT_ALS_PRST_1CONV;
			ret = sensor_write(data, REG_INTERRUPT, status);
			if (ret < 0)
				goto out;

			/* Enable als */
			ret = sensor_read(data, REG_CONFIGURE, 1, &val);
			if (ret < 0)
				goto out;
			val |= JSA1212_CONF_ALS_ENABLE;
			ret = sensor_write(data, REG_CONFIGURE, val);
			if (ret < 0)
				goto out;

			data->state |= JSA1212_STATE_ALS_ENABLED;
		} else {
			SENSOR_DBG(DBG_LEVEL3, "disable als");

			ret = sensor_read(data, REG_CONFIGURE, 1, &val);
			if (ret < 0)
				goto out;
			val &= ~JSA1212_CONF_ALS_ENABLE;
			ret = sensor_write(data, REG_CONFIGURE, val);
			if (ret < 0)
				goto out;

			data->state &= ~JSA1212_STATE_ALS_ENABLED;
		}
	}

out:
	return ret;
}

static int sensor_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);

	SENSOR_DBG(DBG_LEVEL1, "%s", data->client->name);

	disable_irq(data->irq);
	mutex_lock(&data->lock);
	data->state_suspend = data->state;
	sensor_set_mode(data,
			JSA1212_STATE_PS_DISABLED |
			JSA1212_STATE_ALS_DISABLED);
	mutex_unlock(&data->lock);
	return 0;
}

static int sensor_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);

	SENSOR_DBG(DBG_LEVEL1, "%s", data->client->name);

	enable_irq(data->irq);
	mutex_lock(&data->lock);
	sensor_set_mode(data, data->state_suspend);
	mutex_unlock(&data->lock);
	return 0;
}

static void sensor_poll_work(struct work_struct *work)
{
	int ret;
	int delay;
	u8 val;
	struct sensor_data *data = container_of(to_delayed_work(work),
					struct sensor_data, work_ps);

	SENSOR_DBG(DBG_LEVEL1, "%s", data->client->name);

	mutex_lock(&data->lock);

	if (!(data->state & JSA1212_STATE_PS_ENABLED))
		goto out;

	/*exit when get far event*/
	ret = sensor_read(data, REG_PROX_DATA, 1, &val);
	if (ret < 0)
		goto out;
	if (val <= data->ps_threshold) {
		SENSOR_DBG(DBG_LEVEL2, "PS far %02x", val);

		input_report_abs(data->input_ps, ABS_X, 1);
		input_sync(data->input_ps);

		sensor_write(data, REG_PROX_HT, data->ps_threshold);
		goto out;
	}

	delay = msecs_to_jiffies(data->interval);
	schedule_delayed_work(&data->work_ps, delay);
out:
	mutex_unlock(&data->lock);
}

static irqreturn_t sensor_interrupt_handler(int irq, void *pri)
{
	int ret;
	u8 status;
	struct sensor_data *data = (struct sensor_data *)pri;

	SENSOR_DBG(DBG_LEVEL1, "%s", data->client->name);

	mutex_lock(&data->lock);

	ret = sensor_read(data, REG_INTERRUPT, 1, &status);
	if (ret < 0)
		goto out;

	if (status & JSA1212_INT_PXS_FLAG) {
		u8 val;
		int delay;

		SENSOR_DBG(DBG_LEVEL3, "PS irq:%02x", status);

		ret = sensor_read(data, REG_PROX_DATA, 1, &val);
		if (ret < 0)
			goto out;
		if (val > data->ps_threshold) {
			SENSOR_DBG(DBG_LEVEL3, "PS near:%02x", val);

			input_report_abs(data->input_ps, ABS_X, 0);
			input_sync(data->input_ps);

			/*change threshold to avoid interrupt and
			schedule delaywork to poll near event*/
			ret = sensor_write(data, REG_PROX_HT, 0xff);
			if (ret < 0)
				goto out;

			delay = msecs_to_jiffies(data->interval);
			schedule_delayed_work(&data->work_ps, delay);
		} else {
			SENSOR_DBG(DBG_LEVEL3, "Why here PS far:%02x", val);

			input_report_abs(data->input_ps, ABS_X, 1);
			input_sync(data->input_ps);
			sensor_write(data, REG_PROX_HT, data->ps_threshold);
		}
	}

	if (status & JSA1212_INT_ALS_FLAG) {
		u8 raw[2], val;
		int input_raw, input_lux, output_lux, th_low, th_high;

		SENSOR_DBG(DBG_LEVEL3, "ALS irq:%02x", status);

		/* Read sensor */
		ret = sensor_read(data, REG_ALSIR_DT1, 2, raw);
		if (ret < 0)
			goto out;
		input_raw = ((raw[1]<<8) | raw[0]) & ALS_INPUT_MAXVAL;

		/* Calculating output value depends on the current range,
		 * so it needs to be done before possible range change */
		output_lux = als_calculate_output(input_raw);

		SENSOR_DBG(DBG_LEVEL2, "ALS dta:%03x rng:%u/%03x val:%03x",
			   input_raw, als_range, als_ranges[als_range].maxval,
			   output_lux);

		/* Update thresholds / switch measuremente range */
		input_lux = als_normalize_input(input_raw);
		if (als_range_select(input_lux)) {
			/* Switch measurement range */
			ret = sensor_write(data, REG_ALS_RNG, als_range);
			if (ret < 0)
				goto out;

			/* Any value is out of thresholds */
			th_low  = ALS_INPUT_MAXVAL;
			th_high = ALS_INPUT_MINVAL;

			/* Report on every als duty cycle */
			status &= ~JSA1212_INT_ALS_PRST_MASK;
			status |= JSA1212_INT_ALS_PRST_1CONV;
		} else {
			/* Threshold: +/-N% from current value */
			if (als_threshold <= 0 ) {
				/* Negative value = absolute */
				th_low  = input_raw + als_threshold;
				th_high = input_raw - als_threshold;
			} else {
				/* Positive value = % of current */
				th_low  = input_raw * (100 - als_threshold) / 100;
				th_high = input_raw * 2 - th_low;
			}

			/* Report if 4 duty cycles are out of thresholds */
			status &= ~JSA1212_INT_ALS_PRST_MASK;
			status |= JSA1212_INT_ALS_PRST_4CONV;
		}
		if (th_low == input_raw)
			--th_low;
		if (th_high == input_raw)
			++th_high;
		if (th_low < ALS_INPUT_MINVAL)
			th_low = ALS_INPUT_MINVAL;
		if (th_high > ALS_INPUT_MAXVAL)
			th_high = ALS_INPUT_MAXVAL;

		SENSOR_DBG(DBG_LEVEL2, "ALS thr:%03x - %03x", th_low, th_high);

		val = th_low & 0xff;
		ret = sensor_write(data, REG_ALSIR_TH1, val);
		if (ret < 0)
			goto out;
		val = ((th_low >> 8) & 0xf) | ((th_high & 0xf) << 4);
		ret = sensor_write(data, REG_ALSIR_TH2, val);
		if (ret < 0)
			goto out;
		val = th_high >> 4;
		ret = sensor_write(data, REG_ALSIR_TH3, val);
		if (ret < 0)
			goto out;

		/* Emit event */
		SENSOR_DBG(DBG_LEVEL2, "ALS eve:%08x", output_lux);
		input_report_abs(data->input_als, ABS_MISC, output_lux);
		input_sync(data->input_als);

	}

	/*clear both ps and als flag bit7, bit3*/
	status &= ~(JSA1212_INT_ALS_FLAG | JSA1212_INT_PXS_FLAG);
	ret = sensor_write(data, REG_INTERRUPT, status);
out:
	mutex_unlock(&data->lock);
	return IRQ_HANDLED;
}

static int sensor_get_data_init(struct sensor_data *data)
{
	int ret = 0;

	SENSOR_DBG(DBG_LEVEL1, "%s", data->client->name);

	ret = gpio_request(data->gpio_int, DRIVER_NAME);
	if (ret < 0) {
		dev_err(&data->client->dev, "Err: request gpio %d\n",
						data->gpio_int);
		goto out;
	}

	gpio_direction_input(data->gpio_int);
	data->irq = gpio_to_irq(data->gpio_int);
	//irq_set_status_flags(data->irq, IRQ_NOAUTOEN);
	ret = request_threaded_irq(data->irq, NULL, sensor_interrupt_handler,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT, DRIVER_NAME, data);
	if (ret < 0) {
		dev_err(&data->client->dev,
			"Fail to request irq:%d ret=%d\n", data->irq, ret);
		gpio_free(data->gpio_int);
		return ret;
	}

	INIT_DELAYED_WORK(&data->work_ps, sensor_poll_work);
out:
	return ret;
}

/* Remove unused P-sensor function of jsa1212 */
#define DISABLE_PSX 1

static int sensor_input_init(struct sensor_data *data)
{
	int ret;
	struct input_dev *input;

	SENSOR_DBG(DBG_LEVEL1, "%s", data->client->name);
#if !DISABLE_PSX
	input = input_allocate_device();
	if (!input) {
		dev_err(&data->client->dev, "input device allocate failed\n");
		return -ENOMEM;
	}
	input->name = PS_INPUT_NAME;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &data->client->dev;
	set_bit(EV_ABS, input->evbit);
	set_bit(ABS_X, input->absbit);

	ret = input_register_device(input);
	if (ret) {
		dev_err(&data->client->dev,
			"unable to register input device %s:%d\n",
			input->name, ret);
		input_free_device(input);
		return ret;
	}
	data->input_ps = input;
#endif
	input = input_allocate_device();
	if (!input) {
		dev_err(&data->client->dev, "input device allocate failed\n");
		ret = -ENOMEM;
		goto err;
	}
	input->name = ALS_INPUT_NAME;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &data->client->dev;
	set_bit(EV_ABS, input->evbit);
	set_bit(ABS_MISC, input->absbit);

	input_set_abs_params(input, ABS_MISC,
			     ALS_OUTPUT_MINVAL, ALS_OUTPUT_MAXVAL, 0, 0);

	ret = input_register_device(input);
	if (ret) {
		dev_err(&data->client->dev,
			"unable to register input device %s:%d\n",
			input->name, ret);
		input_free_device(input);
		goto err;
	}
	data->input_als = input;
	return 0;
err:
#if !DISABLE_PSX
	input_unregister_device(data->input_ps);
#endif
	return ret;
}

#define SENSOR_SYSFS_POWERON		1
#define SENSOR_SYSFS_POWERDOWN		0
static ssize_t ps_sensor_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);
	int enabled;

	SENSOR_DBG(DBG_LEVEL1, "%s", data->client->name);

	mutex_lock(&data->lock);

	if (data->state & JSA1212_STATE_PS_ENABLED)
		enabled = 1;
	else
		enabled = 0;

	mutex_unlock(&data->lock);

	return sprintf(buf, "%d\n", enabled);
}

static ssize_t ps_sensor_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);
	unsigned long val;
	int ret;

	SENSOR_DBG(DBG_LEVEL1, "%s", data->client->name);

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	SENSOR_DBG(DBG_LEVEL1, " %x", (int)val);

	if (val != SENSOR_SYSFS_POWERON && val != SENSOR_SYSFS_POWERDOWN)
		return -EINVAL;

	mutex_lock(&data->lock);

	if (val) {
		u8 ps;

		sensor_set_mode(data, data->state | JSA1212_STATE_PS_ENABLED);
		/*in case of far event can't trigger at the first time*/
		msleep(1);
		ret = sensor_read(data, REG_PROX_DATA, 1, &ps);
		if (ret >= 0) {
			input_report_abs(data->input_ps, ABS_X,
						ps <= data->ps_threshold);
			input_sync(data->input_ps);
		}
	} else {
		int state = data->state & ~JSA1212_STATE_PS_ENABLED;
		sensor_set_mode(data, state);
	}

	mutex_unlock(&data->lock);

	return count;
}

static ssize_t als_sensor_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);
	int enabled;

	SENSOR_DBG(DBG_LEVEL1, "%s", data->client->name);

	mutex_lock(&data->lock);

	if (data->state & JSA1212_STATE_ALS_ENABLED)
		enabled = 1;
	else
		enabled = 0;

	mutex_unlock(&data->lock);

	return sprintf(buf, "%d\n", enabled);
}

static ssize_t als_sensor_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);
	unsigned long val;

	SENSOR_DBG(DBG_LEVEL1, "%s", data->client->name);

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	SENSOR_DBG(DBG_LEVEL1, " %x", (int)val);

	if (val != SENSOR_SYSFS_POWERON && val != SENSOR_SYSFS_POWERDOWN)
		return -EINVAL;

	mutex_lock(&data->lock);

	if (val)
		sensor_set_mode(data, data->state | JSA1212_STATE_ALS_ENABLED);
	else
		sensor_set_mode(data, data->state & ~JSA1212_STATE_ALS_ENABLED);

	mutex_unlock(&data->lock);

	return count;
}

static ssize_t ps_sensor_thresh_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);

	SENSOR_DBG(DBG_LEVEL1, "%s", data->client->name);

	return sprintf(buf, "%d\n", data->ps_threshold);
}

static ssize_t ps_sensor_thresh_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);
	unsigned long val;

	SENSOR_DBG(DBG_LEVEL1, "%s", data->client->name);

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	SENSOR_DBG(DBG_LEVEL1, " %x", (int)val);

	mutex_lock(&data->lock);

	data->ps_threshold = val;
	sensor_write(data, REG_PROX_LT, data->ps_threshold);
	sensor_write(data, REG_PROX_HT, data->ps_threshold);

	mutex_unlock(&data->lock);

	return count;
}

#ifdef CONFIG_JSA1212_DEBUG
static ssize_t sensor_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", debug_level);
}

static ssize_t sensor_debug_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	debug_level = val;
	return count;
}

static DEVICE_ATTR(debug, S_IRUGO|S_IWUSR, sensor_debug_show,
		sensor_debug_store);
#endif

static ssize_t sensor_als_scale_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", als_scale_multiplier);
}

static ssize_t sensor_als_scale_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	/* The value must be positive to make any sense */
	if (val <= 0)
		return -EINVAL;

	/* Too large values could lead to exceeding the min-max
	 * range userspace can query via EVIOCGABS ioctl */
	if (val > ALS_OUTPUT_MAXVAL * ALS_SCALE_DIVIDER / ALS_INPUT_MAXLUX)
		return -EINVAL;

	als_scale_multiplier = val;
	return count;
}

static DEVICE_ATTR(als_scale, S_IRUGO|S_IWUSR, sensor_als_scale_show,
		   sensor_als_scale_store);

static ssize_t sensor_als_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", als_threshold);
}

static ssize_t sensor_als_threshold_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	long val;

	if (kstrtol(buf, 0, &val))
		return -EINVAL;

	/* Positive values define threshold as percent of the current
	 * sensor value. In order not to overflow the calculations at
	 * the interrupt handler only values < 100% can be allowed. */
	if (val >= 100)
		return -EINVAL;

	/* Negative values are taken as absolute sensor value steps.
	 * This is probably useful only for manual calibration purposes
	 * and does not have similar overflow risk. But still, using
	 * values larger than the whole sensor range makes no sense. */
	if (val < -ALS_INPUT_MAXVAL)
		return -EINVAL;

	als_threshold = val;
	return count;
}

static DEVICE_ATTR(als_threshold, S_IRUGO|S_IWUSR, sensor_als_threshold_show,
		   sensor_als_threshold_store);

static DEVICE_ATTR(ps_enable, S_IRUGO|S_IWUSR,
		ps_sensor_enable_show, ps_sensor_enable_store);
static DEVICE_ATTR(als_enable, S_IRUGO|S_IWUSR,
		als_sensor_enable_show, als_sensor_enable_store);
static DEVICE_ATTR(ps_thresh, S_IRUGO|S_IWUSR,
		ps_sensor_thresh_show, ps_sensor_thresh_store);

static struct attribute *sensor_default_attributes[] = {
	&dev_attr_ps_enable.attr,
	&dev_attr_als_enable.attr,
	&dev_attr_ps_thresh.attr,
#ifdef CONFIG_JSA1212_DEBUG
	&dev_attr_debug.attr,
#endif
	&dev_attr_als_scale.attr,
	&dev_attr_als_threshold.attr,
	NULL
};

static struct attribute_group sensor_default_attribute_group = {
	.attrs = sensor_default_attributes
};

static int sensor_data_init(struct i2c_client *client,
			struct sensor_data *data)
{
	int ret = 0;

	data->client = client;
	mutex_init(&data->lock);
	data->state = 0;
	data->interval = PS_WORK_INTERVAL;
	data->ps_threshold = PS_NEAR_FAR_THRESHOLD;

	data->gpio_int = acpi_get_gpio_by_index(&client->dev, 0, NULL);
	if (data->gpio_int < 0) {
		dev_warn(&client->dev, "Fail to get gpio pin by ACPI\n");
		data->gpio_int = INT_GPIO;
	}
	SENSOR_DBG(DBG_LEVEL3, "gpios:%d", data->gpio_int);

	i2c_set_clientdata(client, data);
	return ret;
}

static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *devid)
{
	int ret = 0;

	SENSOR_DBG(DBG_LEVEL3, "i2c device:%s", devid->name);

	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_I2C | I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&client->dev, "client not i2c capable\n");
		return -ENODEV;
	}

	sensor_data_init(client, &jsa1212_data);

	ret = sensor_input_init(&jsa1212_data);
	if (ret < 0) {
		dev_err(&client->dev, "input init %d\n", ret);
		goto out;
	}

	ret = sensor_get_data_init(&jsa1212_data);
	if (ret) {
		dev_err(&client->dev, "sensor_get_data_init\n");
		goto get_data;
	}

	ret = sysfs_create_group(&client->dev.kobj,
					&sensor_default_attribute_group);
	if (ret) {
		dev_err(&client->dev, "sysfs create group\n");
		goto sys_init;
	}

	ret = sensor_init(&jsa1212_data);
	if (ret) {
		dev_err(&client->dev, "sensor_init\n");
		goto init;
	}

	return 0;
init:
	sysfs_remove_group(&jsa1212_data.client->dev.kobj,
					&sensor_default_attribute_group);
sys_init:
	free_irq(jsa1212_data.irq, &jsa1212_data);
	gpio_free(jsa1212_data.gpio_int);
get_data:
	input_unregister_device(jsa1212_data.input_als);
	input_unregister_device(jsa1212_data.input_ps);
out:
	return ret;
}

static int sensor_remove(struct i2c_client *client)
{
	struct sensor_data *data = i2c_get_clientdata(client);

	free_irq(data->irq, data);
	sysfs_remove_group(&data->client->dev.kobj,
					&sensor_default_attribute_group);
	gpio_free(data->gpio_int);

	input_unregister_device(data->input_als);
	input_unregister_device(data->input_ps);

	return 0;
}

static const struct i2c_device_id jsa1212_id[] = {
	{DRIVER_NAME, 0},
	{},
};

static const struct dev_pm_ops sensor_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(sensor_suspend, sensor_resume)
};

static struct i2c_driver jsa1212_driver = {
	.probe		= sensor_probe,
	.remove		= sensor_remove,
	.driver	= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM_SLEEP
		.pm	= &sensor_pm,
#endif
	},
	.id_table = jsa1212_id,
};

//#define CONFIG_JSA1212_MANUAL_DEVICE
#ifdef CONFIG_JSA1212_MANUAL_DEVICE
static int register_i2c_device(int bus, int addr, char *name)
{
	int ret = 0;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct i2c_board_info info;

	SENSOR_DBG(DBG_LEVEL2, "%s", name);

	memset(&info, 0, sizeof(info));
	strlcpy(info.type, name, I2C_NAME_SIZE);
	adapter = i2c_get_adapter(bus);
	if (!adapter) {
		printk(KERN_ERR "Err: invalid i2c adapter %d\n", bus);
		return -ENODEV;
	} else {
		info.addr = addr;
		client = i2c_new_device(adapter, &info);
		if (!client) {
			printk(KERN_ERR "Fail to add i2c device%d:%d\n",
						bus, addr);
		}
	}
	return ret;
}
#endif

static int __init psals_jsa1212_init(void)
{
	int ret;

	SENSOR_DBG(DBG_LEVEL2, "%s", DRIVER_NAME);

#ifdef CONFIG_JSA1212_MANUAL_DEVICE
	register_i2c_device(3, 0x44, DRIVER_NAME);
#endif

	ret = i2c_add_driver(&jsa1212_driver);
	if (ret < 0)
		printk(KERN_ERR "Fail to register jsa1212 driver\n");

	return ret;
}

static void __exit psals_jsa1212_exit(void)
{
	SENSOR_DBG(DBG_LEVEL2, "%s", DRIVER_NAME);

	i2c_del_driver(&jsa1212_driver);
}

module_init(psals_jsa1212_init);
module_exit(psals_jsa1212_exit);

MODULE_DESCRIPTION("JSA1212 Sensor Driver");
MODULE_AUTHOR("qipeng.zha@intel.com");
MODULE_LICENSE("GPL");
