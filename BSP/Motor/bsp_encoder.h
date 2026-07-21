#ifndef _BSP_ENCODER_H_
#define _BSP_ENCODER_H_

#include "AllHeader.h"

typedef enum {
    FORWARD,  // 姝ｅ悜
    REVERSAL  // 鍙嶅悜
} ENCODER_DIR;

typedef struct {
    volatile long long temp_count; //淇濆瓨瀹炴椂璁℃暟鍊?
    int count;         				//鏍规嵁瀹氭椂鍣ㄦ椂闂存洿鏂扮殑璁℃暟鍊?
    ENCODER_DIR dir;            	 //鏃嬭浆鏂瑰悜
    int ALLcount;  //寮€鏈哄埌鐜板湪鎬荤殑缂栫爜鍣ㄨ鏁?
} ENCODER_RES;


void encoder_init(void);
ENCODER_DIR get_encoderL_dir(void);
ENCODER_DIR get_encoderR_dir(void);



void Get_Odometry(void);
void Encoder_Get_ALL(int *Encoder_all);
void Encoder_Get_Temp(int *Encoder_temp);
void encoder_update(void);

#endif
