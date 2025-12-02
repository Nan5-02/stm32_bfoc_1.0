#include "FOC.h"

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))) // 限幅函数

void Motor_LinkTim(FOC *foc, Tim *tim)
{
   foc->tim = *tim;
}

// 关联传感器到FOC结构体
void Motor_LinkSensor(FOC *foc, Sensor *sensor)
{
   foc->sensor = *sensor;
}

// 关联位置PID到FOC结构体
void Motor_LinkPositionPID(FOC *foc, PID_Controller *pid)
{
   foc->position_pid = *pid;
}

// 关联速度PID到FOC结构体
void Motor_LinkVelocityPID(FOC *foc, PID_Controller *pid)
{
   foc->velocity_pid = *pid;
}

void Motor_SetPositionPID(FOC *foc, PID_Controller pid)
{
   foc->position_pid.Kp = pid.Kp;
   foc->position_pid.Ki = pid.Ki;
   foc->position_pid.Kd = pid.Kd;
}

void Motor_SetVelocityPID(FOC *foc, PID_Controller pid)
{
   foc->velocity_pid.Kp = pid.Kp;
   foc->velocity_pid.Ki = pid.Ki;
   foc->velocity_pid.Kd = pid.Kd;
}

// 设置FOC控制目标
void Motor_SetTarget(FOC *foc, float target)
{
   if (foc->control_target == 1) // 位置控制
   {
      foc->position_pid.setpoint = target;
   }
   else if (foc->control_target == 2) // 速度控制
   {
      foc->velocity_pid.setpoint = target;
   }
   else if (foc->control_target == 3) // 力矩控制
   {
      foc->Uq = target;
   }
}

// 设置目标位置
void Motor_SetTargetAngle(FOC *foc, float angle)
{
   foc->position_pid.setpoint = angle;
}

// 设置目标速度
void Motor_SetTargetVelocity(FOC *foc, float velocity)
{
   foc->velocity_pid.setpoint = velocity;
}

// 角度归一化到0~2PI
float _normalizeAngle(float angle)
{
   while (angle < 0.0f)
      angle += M2PI;
   while (angle >= M2PI)
      angle -= M2PI;
   return angle;
}

// 求解电角度
float _electricalAngle(struct FOC *foc)
{
   return _normalizeAngle((float)(foc->pole_pairs * foc->dir) * foc->sensor.angle - foc->zero_electric_angle);
}

// 设置PWM到控制器输出
void setPWM(struct FOC *foc, float Ua, float Ub, float Uc)
{
   // 计算占空比，限制在0~1之间
   float dc_a = _constrain(Ua / foc->voltage_power_supply, 0.0f, 1.0f);
   float dc_b = _constrain(Ub / foc->voltage_power_supply, 0.0f, 1.0f);
   float dc_c = _constrain(Uc / foc->voltage_power_supply, 0.0f, 1.0f);

   // printf("%.3f,%.3f,%.3f\r\n", dc_a * 100, dc_b * 100, dc_c * 100);

   // 写入PWM
   __HAL_TIM_SetCompare(&foc->tim.port, foc->tim.channelA, (uint32_t)(dc_a * 1800));
   __HAL_TIM_SetCompare(&foc->tim.port, foc->tim.channelB, (uint32_t)(dc_b * 1800));
   __HAL_TIM_SetCompare(&foc->tim.port, foc->tim.channelC, (uint32_t)(dc_c * 1800));
}

void setPhaseVoltage(struct FOC *foc)
{
   if (foc->Uq > foc->voltage_limit)
      foc->Uq = foc->voltage_limit;
   else if (foc->Uq < -foc->voltage_limit)
      foc->Uq = -foc->voltage_limit;

   // 帕克逆变换
   float Ualpha = -foc->Uq * sin(foc->angle_el);
   float Ubeta = foc->Uq * cos(foc->angle_el);

   // 克拉克逆变换
   float Ua = Ualpha + foc->voltage_power_supply / 2;
   float Ub = (sqrt(3) * Ubeta - Ualpha) / 2 + foc->voltage_power_supply / 2;
   float Uc = (-Ualpha - sqrt(3) * Ubeta) / 2 + foc->voltage_power_supply / 2;

#if 1
   // float Umin = fmin(Ua, fmin(Ub, Uc));
   // float Umax = fmax(Ua, fmax(Ub, Uc));
   // float center = (Umax + Umin) / 2;

   float Umin = fmin(Ua, fmin(Ub, Uc));
   Ua -= Umin;
   Ub -= Umin;
   Uc -= Umin;

   // Ua += center;
   // Ub += center;
   // Uc += center;
#endif

   // 设置PWM
   setPWM(foc, Ua, Ub, Uc);
}

void foc_Init(struct FOC *foc)
{

   HAL_TIM_PWM_Start(&foc->tim.port, foc->tim.channelA);
   HAL_TIM_PWM_Start(&foc->tim.port, foc->tim.channelB);
   HAL_TIM_PWM_Start(&foc->tim.port, foc->tim.channelC);
   HAL_Delay(100);

   if (foc->zero_electric_angle != 0.0f)
   {
      // printf("Zero electric angle preset: %f\r\n", foc->zero_electric_angle);
      return;
   }

   foc->angle_el = _3PI_2; // 计算电角度

   for (int i = 0; i < 100; i++)
   {
      foc->Uq += foc->voltage_sensor_align / 100.0f; // 平滑增加对齐电压
      // printf("Aligning... Uq: %f\r\n", foc->Uq);
      setPhaseVoltage(foc); // 施加一个固定电压矢量
      HAL_Delay(10);
   }
   HAL_Delay(500);
   Sensor_Update(&foc->sensor);
   foc->zero_electric_angle = _electricalAngle(foc); // 读取当前电角度作为零点
   // printf("Zero electric angle: %f\r\n", foc->zero_electric_angle);
   HAL_Delay(500);
   foc->Uq = 0.0f;       // 重置电压
   setPhaseVoltage(foc); // 停止施加电压
}

void foc_loop(FOC *foc)
{
   Sensor_Update(&foc->sensor);
   foc->angle_el = _electricalAngle(foc);
   if (foc->control_target == 1) // 速度控制
      foc->Uq = foc->position_pid.output;
   else if (foc->control_target == 2) // 速度控制
      foc->Uq = foc->velocity_pid.output;
}

void foc_move(FOC *foc)
{
   setPhaseVoltage(foc);
}

void openLoopSpeedControl(struct FOC *foc)
{
   static uint32_t open_loop_timestamp;
   uint32_t now_time_us = DWT_Get_Microsecond();

   float dt;
   if (open_loop_timestamp == 0)
   {
      dt = 0.0f; // 第一次调用不推进角度
   }
   else
   {
      dt = (float)(now_time_us - open_loop_timestamp) * 1e-6f; // us -> s
   }

   if (dt <= 0 || dt > 0.5f)
      dt = 1e-3f;

   foc->angle_el = _normalizeAngle(foc->angle_el + foc->target_velocity * dt);

   open_loop_timestamp = now_time_us;
}

void Pidloop(FOC *foc)
{
   if (foc->control_target == 1) // 位置控制
      PidCompute(&foc->position_pid, foc->sensor.angle_with_rotations - foc->zero_electric_angle);
   else if (foc->control_target == 2) // 速度控制
      PidCompute(&foc->velocity_pid, foc->sensor.velocity);
}

void Motor_SetFocControlMode(FOC *foc, uint8_t mode)
{
   foc->control_target = mode;
}

void Motor_SetFocDirection(FOC *foc, uint8_t dir)
{
   if (dir == 0)
      foc->dir = 1;
   else
      foc->dir = -1;
}

void Motor_SetElectricalpolePairs(FOC *foc, uint8_t pole_pairs)
{
   foc->pole_pairs = pole_pairs;
}
