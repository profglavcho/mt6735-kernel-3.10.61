/*
 * Copyright (C) 2009 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*******************************************************************************
 *
 * Filename:
 * ---------
 * hdmi_customization.h
 *
 * Project:
 * --------
 *   Android
 *
 * Description:
 * ------------
 *   This file implements Customization base function
 *
 *******************************************************************************/
#include <mach/mt_pm_ldo.h>
#include <pmic_drv.h>

#include <cust_gpio_usage.h>
#include <hdmi_cust.h>
#include <mach/mt_gpio.h>

/******************************************************************
** Basic define
******************************************************************/
#ifndef s32
	#define s32 signed int
#endif
#ifndef s64
	#define s64 signed long long
#endif

static bool cust_power_on = false;
int cust_hdmi_power_on(int on)
{
	if(on > 0) 
	{
	    printk("MHL_Power power  \n" );
#ifdef PMIC_APP_MHL_POWER_LDO1
		if(cust_power_on == false)
		{
			hwPowerOn(PMIC_APP_MHL_POWER_LDO1, VOL_1200,"MHL");
			cust_power_on = true;
		}
#else
		printk("Error: PMIC_APP_MHL_POWER_LDO1 not defined -\n" );
#endif
	}
	else
	{
#ifdef PMIC_APP_MHL_POWER_LDO1
		if(cust_power_on == true)
		{
			hwPowerDown(PMIC_APP_MHL_POWER_LDO1, "MHL");
			cust_power_on = false;
		}
#endif
	}
	return 0;
}

int cust_hdmi_dpi_gpio_on(int on)
{
    unsigned int dpi_pin_start = 0;
	if(on > 0)
	{
#ifdef GPIO_EXT_DISP_DPI0_PIN
		for(dpi_pin_start = GPIO_EXT_DISP_DPI0_PIN; dpi_pin_start < GPIO_EXT_DISP_DPI0_PIN + 16; dpi_pin_start++)
		{
			mt_set_gpio_mode(dpi_pin_start, GPIO_MODE_01);
		}
		printk("%s, %d GPIO_EXT_DISP_DPI0_PIN is defined+ %x\n", __func__, __LINE__, GPIO_EXT_DISP_DPI0_PIN);
#else
		printk("%s,%d Error: GPIO_EXT_DISP_DPI0_PIN is not defined\n", __func__, __LINE__);
#endif
	}
	else
	{
#ifdef GPIO_EXT_DISP_DPI0_PIN
		for(dpi_pin_start = GPIO_EXT_DISP_DPI0_PIN; dpi_pin_start < GPIO_EXT_DISP_DPI0_PIN + 16; dpi_pin_start++)
		{
			mt_set_gpio_mode(dpi_pin_start, GPIO_MODE_00);
			mt_set_gpio_dir(dpi_pin_start, GPIO_DIR_IN);
			mt_set_gpio_pull_enable(dpi_pin_start, GPIO_PULL_ENABLE);
			mt_set_gpio_pull_select(dpi_pin_start, GPIO_PULL_DOWN);
		}
		printk("%s, %d GPIO_EXT_DISP_DPI0_PIN is defined- %x\n", __func__, __LINE__, GPIO_EXT_DISP_DPI0_PIN);
#endif
	}
	
	return 0;
}

int cust_hdmi_i2s_gpio_on(int on)
{
    if(on ==1)
    {
#ifdef GPIO_MHL_I2S_OUT_WS_PIN
		mt_set_gpio_mode(GPIO_MHL_I2S_OUT_WS_PIN, GPIO_MODE_01);
		mt_set_gpio_mode(GPIO_MHL_I2S_OUT_CK_PIN, GPIO_MODE_01);
		mt_set_gpio_mode(GPIO_MHL_I2S_OUT_DAT_PIN, GPIO_MODE_01);

		mt_set_gpio_dir(GPIO_MHL_I2S_OUT_WS_PIN, GPIO_DIR_OUT);
#else
		printk("%s,%d. GPIO_MHL_I2S_OUT_WS_PIN is not defined\n", __func__, __LINE__);
#endif
    }
    //gpio: uart 
    else if (on ==2)
    {
#ifdef GPIO_MHL_I2S_OUT_WS_PIN
		mt_set_gpio_mode(GPIO_MHL_I2S_OUT_WS_PIN, GPIO_MODE_02);
		mt_set_gpio_mode(GPIO_MHL_I2S_OUT_CK_PIN, GPIO_MODE_01);
		mt_set_gpio_mode(GPIO_MHL_I2S_OUT_DAT_PIN, GPIO_MODE_02);
        printk("%s,%d.  switch_to_share_pin_mode\n", __func__, __LINE__);
#else
        printk("%s,%d. GPIO_MHL_I2S_OUT_WS_PIN is not defined\n", __func__, __LINE__);
#endif
    }
    else
    {
#ifdef GPIO_MHL_I2S_OUT_WS_PIN
		mt_set_gpio_mode(GPIO_MHL_I2S_OUT_WS_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_MHL_I2S_OUT_WS_PIN, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(GPIO_MHL_I2S_OUT_WS_PIN, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(GPIO_MHL_I2S_OUT_WS_PIN, GPIO_PULL_DOWN);

		mt_set_gpio_mode(GPIO_MHL_I2S_OUT_CK_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_MHL_I2S_OUT_CK_PIN, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(GPIO_MHL_I2S_OUT_CK_PIN, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(GPIO_MHL_I2S_OUT_CK_PIN, GPIO_PULL_DOWN);

		mt_set_gpio_mode(GPIO_MHL_I2S_OUT_DAT_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_MHL_I2S_OUT_DAT_PIN, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(GPIO_MHL_I2S_OUT_DAT_PIN, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(GPIO_MHL_I2S_OUT_DAT_PIN, GPIO_PULL_DOWN);
#else
		printk("%s,%d. GPIO_MHL_I2S_OUT_WS_PIN is not defined\n", __func__, __LINE__);
#endif
	}
	
	return 0;
}

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
int get_hdmi_i2c_addr(void)
{
#ifdef SII_I2C_ADDR;
    return (SII_I2C_ADDR);    
#else
    printk("dynamic_switch_i2c_address +\n");

	int res;
	int addr = 0x72;
	int data[4];

	res = IMM_GetOneChannelValue(12, data, NULL);
	printk("IMM_GetOneChannelValue - res=%d\n", res);
	if(res == 0)
	{
		printk("data: %d,%d, %d, %d\n", data[0], data[1], data[2], data[3]);
		if((data[1] >= 73 && data[1] <= 81)  ||  //ID 2.5
		   (data[1] >= 95 && data[1] <= 103) ||  //ID 3
		   (data[1] >= 131 && data[1] <= 139))   //ID 4
		{
			printk("ID 2.5, ID 3, ID 4, Need switch I2C address\n");
			addr = 0x76;
		}
		else
		{
			printk("ID 2, Do not need switch I2C address\n");
		}
	}
	return addr;
#endif

}

int get_hdmi_i2c_channel(void)
{
    return (HDMI_I2C_CHANNEL);
}