#ifndef __CONTROL_H
#define __CONTROL_H

#include "bsp.h"

typedef enum {
    COMP_STATE_IDLE = 0,
    COMP_STATE_STRAIGHT_FIRST,
    COMP_STATE_ARC_FIRST,
    COMP_STATE_STRAIGHT_SECOND,
    COMP_STATE_ARC_SECOND,
    COMP_STATE_FINISHED,
    COMP_STATE_ERROR
} CompetitionState;

extern int encoder_odometry_flag;
extern volatile int odometry_sum;
extern volatile int RGB_BEEP_flag;

uint8_t switch_mode(void);
void Competition_Init(uint8_t mode);
void Competition_Update(void);
bool Competition_IsFinished(void);
uint8_t Competition_GetLap(void);
CompetitionState Competition_GetState(void);

#endif
