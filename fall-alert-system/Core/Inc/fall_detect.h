#ifndef __FALL_DETECT__H_
#define __FALL_DETECT__H_

#include "main.h"

void FallDetect_Init(void);
uint8_t FallDetect_Updata(float svm, float gyr, float angle);
uint8_t FallDetect_GetState(void);


#endif
