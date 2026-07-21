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
#include "bsp_mpu6050.h"
#include "get_mpu6050.h"
#include "Filter.h"


#include "bsp_ultrasonic.h"

#include "task.h"

#include "Kalman.h"

#include "bsp_ir.h"

#include "control.h"
#include "app_irtracking_eight.h"




#endif

