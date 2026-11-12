#ifndef MPU_H
#define MPU_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include "tim.h"
#include "i2c.h"
#include<math.h>

bool Wake_MPU(void);    //唤醒MPU6050
void MPU_Get_Angle(void);   //读取MPU的原始数据
void Calculate_Oula(void);  //计算欧拉角
void control_Yaw_Angle(uint8_t Yaw_Angle);  //控制偏航舵机的角度
void control_Pitch_Angle(uint8_t Pitch_Angle);  //控制俯仰舵机的角度

#endif
