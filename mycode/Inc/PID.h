#ifndef PID_H
#define PID_H

#include "stm32f1xx_hal.h"

typedef struct {
  float Kp;  //比例系数
  float Ki;  //积分系数
  float Kd;  //微分系数

  float setpoint;  //设定值

  int64_t last_time;  //上一次计算时间
  float last_error;  //上一次误差
  float last_integral;  //上一次积分值
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd);
void PID_SetPoint(PID_TypeDef *pid, float setpoint);
float PID_Compute(PID_TypeDef *pid, float FeedBack);
uint32_t GetUs(void);


#endif
