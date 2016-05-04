
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/power/dc_xpwr_battery.h>
#include <asm/intel_em_config.h>

#define THERM_CURVE_MAX_SAMPLES 18
#define THERM_CURVE_MAX_VALUES	4

static struct dollarcove_fg_pdata pdata;

int bat_curve[] = {
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x2,
	0x3, 0x4, 0x6, 0xA, 0xF, 0x17, 0x25, 0x29,
	0x2D, 0x31, 0x34, 0x3a, 0x3f, 0x44, 0x48, 0x4a,
	0x4e, 0x52, 0x53, 0x55, 0x59, 0x5D, 0x61, 0x64
};

/*
 * This array represents the Battery Pack thermistor
 * temperature and corresponding ADC value limits
 */
static int const therm_curve_data[THERM_CURVE_MAX_SAMPLES]
	[THERM_CURVE_MAX_VALUES] = {
	/* {temp_max, temp_min, adc_max, adc_min} */
	{-15, -20, 682, 536},
	{-10, -15, 536, 425},
	{-5, -10, 425, 338},
	{0, -5, 338, 272},
	{5, 0, 272, 220},
	{10, 5, 220, 179},
	{15, 10, 179, 146},
	{20, 15, 146, 120},
	{25, 20, 120, 100},
	{30, 25, 100, 83},
	{35, 30, 83, 69},
	{40, 35, 69, 58},
	{45, 40, 58, 49},
	{50, 45, 49, 41},
	{55, 50, 41, 35},
	{60, 55, 35, 30},
	{65, 60, 30, 25},
	{70, 65, 25, 22},
};

static int conv_adc_temp(int adc_val, int adc_max, int adc_diff, int temp_diff)
{
	int ret;

	ret = (adc_max - adc_val) * temp_diff;
	return ret / adc_diff;
}

static bool is_valid_temp_adc_range(int val, int min, int max)
{
	if (val > min && val <= max)
		return true;
	else
		return false;
}

static int dc_xpwr_get_batt_temp(int adc_val, int *temp)
{
	int i;

	for (i = 0; i < THERM_CURVE_MAX_SAMPLES; i++) {
		/* linear approximation for battery pack temperature */
		if (is_valid_temp_adc_range(adc_val, therm_curve_data[i][3],
					    therm_curve_data[i][2])) {

			*temp = conv_adc_temp(adc_val, therm_curve_data[i][2],
					     therm_curve_data[i][2] -
					     therm_curve_data[i][3],
					     therm_curve_data[i][0] -
					     therm_curve_data[i][1]);

			*temp += therm_curve_data[i][1];
			break;
		}
	}

	if (i >= THERM_CURVE_MAX_SAMPLES)
		return -ERANGE;

	return 0;

}
static bool dollarcove_is_valid_batid(void)
{
	struct em_config_oem0_data data;
	bool ret = true;

	if (!em_config_get_oem0_data(&data))
		ret = false;

	return ret;
}

static void *get_platform_data(void)
{
	int i;

	if (dollarcove_is_valid_batid()) {
		snprintf(pdata.battid, (BATTID_LEN + 1),
				"%s", "INTN0001");
		pdata.technology = POWER_SUPPLY_TECHNOLOGY_LION;
	} else {
		snprintf(pdata.battid, (BATTID_LEN + 1),
				"%s", "UNKNOWNB");
		pdata.technology = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
	}

	pdata.batt_adc_to_temp = dc_xpwr_get_batt_temp;
	pdata.design_cap = 4450;
	pdata.design_min_volt = 3400;
	pdata.design_max_volt = 4350;
	pdata.max_temp = 55;
	pdata.min_temp = 0;

	return &pdata;
}

void *dollarcove_fg_pdata(void *info)
{
	return get_platform_data();
}
