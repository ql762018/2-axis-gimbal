#include "PID.h"

//对PID结构体进行初始化
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd)
{
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  pid->setpoint = 0.0f;
  pid->last_time = -1;
  pid->last_error = 0.0f;
  pid->last_integral = 0.0f;
}

//设置PID的目标值
void PID_SetPoint(PID_TypeDef *pid, float setpoint)
{
  pid->setpoint = setpoint;
}

//计算PID输出值
float PID_Compute(PID_TypeDef *pid, float FeedBack)
{
  float error = pid->setpoint - FeedBack; //当前误差

  uint64_t NowT = GetUs();
  float DeltaT = NowT - pid->last_time;   //计算时间间隔，单位为微秒
  //第一次计算时，last_time为-1，DeltaT为0
  if (pid->last_time == -1)
  {
    DeltaT = 0.0f; //第一次计算时，时间间隔为0
  }
  else
  {
    DeltaT /= 1000000.0f; //将微秒转换为秒
  }

  float ErrorDerivative = (error - pid->last_error) / DeltaT;   //计算误差的变化率
  float ErrorIntegral = pid->last_integral + error * DeltaT;    //计算误差的积分值

  float COp = pid->Kp * error;    //比例项输出值
  float COi = pid->Ki * ErrorIntegral;    //积分项输出值
  float COd = pid->Kd * ErrorDerivative;    //微分项输出值

  pid->last_time = NowT;
  pid->last_error = error;
  pid->last_integral = ErrorIntegral;
  
  return COp + COi + COd;
}

//获取当前时间 单位；微秒
uint32_t GetUs(void)
{
  uint32_t ms, cycle_cnt;
  do {
    ms = HAL_GetTick();
    cycle_cnt = SysTick->VAL;
  } while (ms != HAL_GetTick());
  return ms * 1000 + ((SysTick->LOAD - cycle_cnt) / (SystemCoreClock/1000000UL));
}

