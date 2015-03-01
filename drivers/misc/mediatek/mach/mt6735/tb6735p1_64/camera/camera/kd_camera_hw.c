#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"

/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw_2]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_INFO, PFX , fmt, ##arg)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)         xlog_printk(ANDROID_LOG_ERR, PFX , fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...) \
                do {    \
                   xlog_printk(ANDROID_LOG_INFO, PFX , fmt, ##arg); \
                } while(0)
#else
#define PK_DBG(a,...)
#define PK_ERR(a,...)
#define PK_XLOG_INFO(fmt, args...)
#endif

/*
#ifndef BOOL
typedef unsigned char BOOL;
#endif
*/

/* Mark: need to verify whether ISP_MCLK1_EN is required in here //Jessy @2014/06/04*/
extern void ISP_MCLK1_EN(BOOL En);
extern void ISP_MCLK2_EN(BOOL En);
extern void ISP_MCLK3_EN(BOOL En);


int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{

u32 pinSetIdx = 0;//default main sensor

#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4
#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3


u32 pinSet[3][8] = {
                        //for main sensor
                     {  CAMERA_CMRST_PIN, // The reset pin of main sensor uses GPIO10 of mt6306, please call mt6306 API to set
                        CAMERA_CMRST_PIN_M_GPIO,   /* mode */
                        GPIO_OUT_ONE,              /* ON state */
                        GPIO_OUT_ZERO,             /* OFF state */
                        CAMERA_CMPDN_PIN,
                        CAMERA_CMPDN_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     },
                     //for sub sensor
                     {  CAMERA_CMRST1_PIN,
                        CAMERA_CMRST1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                        CAMERA_CMPDN1_PIN,
                        CAMERA_CMPDN1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     },
                     //for main_2 sensor
                     {  GPIO_CAMERA_INVALID,
                        GPIO_CAMERA_INVALID,   /* mode */
                        GPIO_OUT_ONE,               /* ON state */
                        GPIO_OUT_ZERO,              /* OFF state */
                        GPIO_CAMERA_INVALID,
                        GPIO_CAMERA_INVALID,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     }
                   };

    
	
    PK_DBG("pinSetIdx:%x, currSensorName: %s,SensorIdx:%d, mode_name:%s\n", pinSetIdx, currSensorName, SensorIdx, mode_name);
    if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx){
        pinSetIdx = 0;
    }
    else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
        pinSetIdx = 1;
    }
    else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx) {
        pinSetIdx = 2;
    }
	else
	{
		PK_DBG("not match... just return!");
	}
   
    //power ON
    if (On) {

#if 1
         ISP_MCLK1_EN(1);
         ISP_MCLK2_EN(1);
         ISP_MCLK3_EN(1);
#else
        if(pinSetIdx == 0 ) {
           ISP_MCLK1_EN(1);
        }
        else if (pinSetIdx == 1) {
            ISP_MCLK1_EN(1);
        }
#endif 

        
#ifdef MTK_MT6306_SUPPORT
        PK_DBG("MTK_MT6306_SUPPORT is defined, mt6306_set_gpio_dir: %d\n", pinSet[pinSetIdx][IDX_PS_CMRST]);
#endif       

        if (pinSetIdx == 0 && currSensorName && (0 == strcmp(currSensorName,SENSOR_DRVNAME_HI545_MIPI_RAW)))

        {
            //First Power Pin low and Reset Pin Low
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }

            
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {

#ifndef MTK_MT6306_SUPPORT
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
#else
                if(mt6306_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt6306_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}                  
#endif 
                
            }

        #if 1
	    //VCAM_IO
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800, mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);
                goto _kdCISModulePowerOn_exit_;
            }
	    #endif
        
        #if 1
	    //VCAM_A
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_A), power id = %d\n", CAMERA_POWER_VCAM_A);
                goto _kdCISModulePowerOn_exit_;
            }
        #endif
        
			#if 0
			//VCAM_D
			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name))
			{
				PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_D), power id = %d\n", CAMERA_POWER_VCAM_D);
				goto _kdCISModulePowerOn_exit_;
			}
			#endif

            #if 1
            //AF_VCC
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_2800,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable analog power (VCAM_AF), power id = %d \n", CAMERA_POWER_VCAM_AF);
                goto _kdCISModulePowerOn_exit_;
            }
            #endif
            
            //mdelay(15);
            
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }
			mdelay(10);

            //enable active sensor
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
#ifndef MTK_MT6306_SUPPORT
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
#else             
                if(mt6306_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt6306_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}                
#endif
            }

	        mdelay(5);
				#if 0
					while(1){
						mdelay(1000);
						PK_DBG("main camera sleep here...");
					}
				#endif
            
        }
        else if (pinSetIdx == 1 && currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV2680_MIPI_RAW, currSensorName)))
        {
        	#if 0
             if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}     
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}      
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}        
			#endif
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}       
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}        
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}

                  
				//IOVDD
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800,mode_name)){ PK_DBG("[CAMERA SENSOR] Fail to enable VCAM IO power\n");}
				mdelay(5);
				//DVDD
				PK_DBG("[ON_general 1.8V]sensorIdx:%d \n",SensorIdx);
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1500,mode_name)){ PK_DBG("[CAMERA SENSOR] Fail to enable VCAM D power\n");}
				mdelay(5);

				//AVDD
				if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name)) { PK_DBG("[CAMERA SENSOR] Fail to enable VCAM A power\n");}
				
                
				//AF_VCC
				mdelay(1);

                 
    		  	if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]){
    			  	if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
    			  	if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
    			  	if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
    			  	#if 0
    			  	//RST pin
    				 if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
    				 if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
    				 if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
					#endif
				}
		  	
				mdelay(1);

				#if 0
					while(1){
						mdelay(1000);
						PK_DBG("sub camera sleep here...");
					}
				#endif
        }
        else
        {
            // for i2c power is ok, which can make the i2c operation error, but not timeout
            //IOVDD
		    if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800,mode_name)){ PK_DBG("[CAMERA SENSOR] Fail to enable VCAM IO power\n");}
			mdelay(5);
        }
    }
    else {//power OFF

        /* Mark: need to verify whether ISP_MCLK1_EN is required in here //Jessy @2014/06/04 */
#if 1
        ISP_MCLK1_EN(0);
        ISP_MCLK2_EN(0);
        ISP_MCLK3_EN(0);
#else
        if(pinSetIdx == 0 ) {
            ISP_MCLK1_EN(0);
        }
        else if (pinSetIdx == 1) {
            ISP_MCLK1_EN(0);
        }
#endif         

        if (pinSetIdx == 0 && currSensorName && (0 == strcmp(currSensorName,SENSOR_DRVNAME_HI545_MIPI_RAW)))

        {   
	    //Set Reset Pin Low
             if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
#ifndef MTK_MT6306_SUPPORT
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! (CMRST)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");}
#else
                if(mt6306_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! (CMRST)\n");}
                if(mt6306_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! (CMRST)\n");} 
#endif
            }
	        mdelay(10);

            //Set Power Pin low and Reset Pin Low
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! (CMPDN)\n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! (CMPDN)\n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! (CMPDN)\n");}
            }
            
            


            //AF_VCC
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_AF,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to OFF AF power (VCAM_AF), power id = %d \n", CAMERA_POWER_VCAM_AF);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
			#if 0
			//VCAM_D
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_D), power id= (%d) \n", CAMERA_POWER_VCAM_D);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
			#endif
			
	    	//VCAM_A
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF analog power (VCAM_A), power id= (%d) \n", CAMERA_POWER_VCAM_A);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
			
			
            //VCAM_IO
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO, mode_name)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
            //mdelay(10); 
            
        }
        else if (pinSetIdx == 1 && currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV2680_MIPI_RAW, currSensorName)))
        {
             PK_DBG("[%s][CameraPowerOFF] %s camera : %s\n", __FUNCTION__ , (pinSetIdx == 0 ? "Main" : "Sub") , currSensorName);
            if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
				#if 0
                  if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                  if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}           
                  if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
				#endif  

                  if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
                  if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
                  if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //high == power down lens module
            }
            
            
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {PK_DBG("[CAMERA SENSOR] Fail to OFF digital power: VCAM_A\n");}
			
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO,mode_name)) {PK_DBG("[CAMERA SENSOR] Fail to OFF digital power: VCAM_IO\n");}
            
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D, mode_name)) {PK_DBG("[CAMERA SENSOR] Fail to OFF digital power: VCAM_D\n");}

        }
        else
        {
            // power down DOVDD
            //VCAM_IO
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO, mode_name)) {
                PK_DBG("[CAMERA SENSOR] Fail to OFF digital power (VCAM_IO), power id = %d \n", CAMERA_POWER_VCAM_IO);
                //return -EIO;
                goto _kdCISModulePowerOn_exit_;
            }
           mdelay(10); 
        }
    }

    return 0;

_kdCISModulePowerOn_exit_:
    return -EIO;
    
}

EXPORT_SYMBOL(kdCISModulePowerOn);

//!--
//


