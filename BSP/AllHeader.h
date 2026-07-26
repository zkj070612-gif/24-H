#ifndef __ALLHEADER_H_
#define __ALLHEADER_H_


#define u8  uint8_t
#define u16 uint16_t 
#define u32 uint32_t 

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "ti_msp_dl_config.h"
#include "delay.h"
#include "config.h"
#include "bsp.h"

#include "usart0.h"
#include "usart1_ble.h"
#include "magnet.h"



#include "bsp_flash_W25Q64.h"

#include "bsp_beep_led.h"
#include "bsp_key.h"
#include "bsp_timer.h"

#include "bsp_RGB.h"


#include "app_rgb.h"

#include "bsp_ir_receiver.h"
#include "app_irremote.h"

#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "app_motor.h"
#include "bsp_PID_motor.h"


#include "oled.h" 

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

/* 选择使用的姿态传感器 (二选一) */
//#define USE_MPU6050
#define USE_BNO085

#ifdef USE_MPU6050
    #include "bsp_mpu6050.h"
    #include "get_mpu6050.h"
#else
    #include "bsp_bno085.h"
    #include "get_bno085.h"

    /* 兼容宏定义 */
    #define DMP_Init()              BNO085_DMP_Init()
    #define Get_EulerAngles()       BNO085_Get_EulerAngles()
    #define Get_Filter              BNO085_Get_Filter
    #define Dir_PID                 BNO085_Dir_PID
    #define Get_Angle               BNO085_Get_Angle
    #define Get_CalibratedAngles()  BNO085_Get_CalibratedAngles()
    #define AngleOffsetCalc()       BNO085_AngleOffsetCalc()
    #define navigetion_0_360_limit  BNO085_navigetion_0_360_limit
    #define get_minor_arc           BNO085_get_minor_arc
    #define MPU6050ReadGyro         BNO085_ReadGyro_Short
    #define MPU6050ReadAcc          BNO085_ReadAccel_Short
    #define MPU6050_GetTemp         BNO085_GetTemp
#endif

#include "Filter.h"


#include "bsp_ultrasonic.h"

#include "task.h"

#include "Kalman.h"

#include "bsp_ir.h"

#include "control.h"
#include "app_irtracking_eight.h"




#endif

