#include "MPU.h"

static float ax,ay,az;  //陀螺仪的三轴加速度
static float gx,gy,gz;  //陀螺仪的三轴角速度
extern float yaw, pitch;    //偏航角、俯仰角

//唤醒MPU6050
bool Wake_MPU(void)
{
  uint8_t wake = 0x00;
  if (HAL_I2C_Mem_Write(&hi2c2, 0xD0, 0x6B, I2C_MEMADD_SIZE_8BIT, &wake, 1, 100) != HAL_OK)
  {
  return false;
  }
  return true;
}

//读取MPU的原始数据
void MPU_Get_Angle(void)
{
  int16_t ax_raw, ay_raw, az_raw;    //陀螺仪的三轴加速度初始数据
  int16_t gx_raw, gy_raw, gz_raw;    //陀螺仪的三轴角速度初始数据
  uint8_t A_raw[6], G_raw[6];    //陀螺仪的三轴加速度和角速度原始数据}

  if(HAL_I2C_Mem_Read(&hi2c2, 0xD0, 0x3B, I2C_MEMADD_SIZE_8BIT, A_raw, 6, 100) != HAL_OK)
  {
    return;    //如果读取失败，直接返回
  }

//将原始数据转换为有符号整数
  ax_raw = (int16_t)(A_raw[0] << 8 | A_raw[1]);
  ay_raw = (int16_t)(A_raw[2] << 8 | A_raw[3]);
  az_raw = (int16_t)(A_raw[4] << 8 | A_raw[5]);

//将原始数据转换为加速度值
  ax = ax_raw * 6.1035e-5f;
  ay = ay_raw * 6.1035e-5f;
  az = az_raw * 6.1035e-5f;

  if(HAL_I2C_Mem_Read(&hi2c2, 0xD0, 0x43, I2C_MEMADD_SIZE_8BIT, G_raw, 6, 100) != HAL_OK)
  {
      return;    //如果读取失败，直接返回
  }
//将原始数据转换为有符号整数
  gx_raw = (int16_t)(G_raw[0] << 8 | G_raw[1]);
  gy_raw = (int16_t)(G_raw[2] << 8 | G_raw[3]);
  gz_raw = (int16_t)(G_raw[4] << 8 | G_raw[5]);

//将原始数据转换为角速度值
  gx = gx_raw * 6.1035e-2f;
  gy = gy_raw * 6.1035e-2f;
  gz = gz_raw * 6.1035e-2f;
}

//计算欧拉角
void Calculate_Oula(void)
{
  static uint32_t Next_Time = 0; //下一次计算欧拉角的时间
//延迟逻辑
  if(HAL_GetTick() < Next_Time)
  {
    return;
  }
//读取陀螺仪的原始数据
  MPU_Get_Angle();

//通过陀螺仪计算欧拉角
  float yaw_g = yaw + gz * 0.01 + 0.01;    //偏航角
  float pitch_g = pitch + gy * 0.01;    //俯仰角

//通过加速度计计算欧拉角
  float pitch_a = atan2(ay, az) * 180 / M_PI;    //俯仰角

//融合陀螺仪和加速度计的欧拉角
  yaw = yaw_g;    //偏航角
  pitch = 1.00086f * pitch_g - 0.00086f * pitch_a;  //俯仰角

  Next_Time += 10;    //下一次计算欧拉角的时间
}

//控制偏航舵机的角度
void control_Yaw_Angle(uint8_t Yaw_Angle)
{
  uint16_t pwm_value = (Yaw_Angle * 2000) / 180 + 500;    //将角度转换为PWM值
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value);    //设置偏航舵机的PWM值
}

//控制俯仰舵机的角度
void control_Pitch_Angle(uint8_t Pitch_Angle)
{
  uint16_t pwm_value = (Pitch_Angle * 2000) / 180 + 500;    //将角度转换为PWM值
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_value);    //设置俯仰舵机的PWM值
}