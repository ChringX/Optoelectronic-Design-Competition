#include "pid.h"
#include "motor.h"
#include "stdio.h"

struct I_pid_obj motor_L0 = {0,0,0,0,0,0};
struct I_pid_obj motor_L1 = {0,0,0,0,0,0};
struct I_pid_obj motor_R0 = {0,0,0,0,0,0};
struct I_pid_obj motor_R1 = {0,0,0,0,0,0};
struct PID_param motor_pid_paramL0,motor_pid_paramL1,motor_pid_paramR0,motor_pid_paramR1;

struct P_pid_obj line_pid_obj = {0,0,0,0,0,0};
struct PID_param line_pid_param;

struct P_pid_obj gyroT_pid = {0,0,0,0,0,0};//停下转
struct P_pid_obj gyroG_pid = {0,0,0,0,0,0};//自平衡
struct P_pid_obj small_gyroT_pid = {0,0,0,0,0,0};
struct PID_param gyroT_pid_param, gyroG_pid_param, small_gyroT_pid_param;

struct P_pid_obj GyroP_pid ={0,0,0,0,0,0};//漂移
struct PID_param GyroP_pid_param;
//增量式PID
//带抗积分饱和
void incremental_PID (struct I_pid_obj *motor, struct PID_param *pid)
{
	float proportion = 0, integral = 0, differential = 0;
	
	motor->bias = motor->target - motor->measure;
	
	proportion = motor->bias - motor->last_bias;
	
	//抗积分饱和
	if (motor->output > pid->outputMax || motor->measure > pid->actualMax)
	{
		if (motor->bias < 0)
			integral = motor->bias;	
	}
	else if (motor->output < -pid->outputMax || motor->measure < -pid->actualMax)
	{
		if (motor->bias > 0)
			integral = motor->bias;
	}
	else
	{
		integral = motor->bias;
	}
	
	differential = (motor->bias - 2 * motor->last_bias + motor->last2_bias);
	
	motor->output += pid->kp*proportion + pid->ki*integral + pid->kd*differential;
	
	motor->last2_bias = motor->last_bias;
	motor->last_bias = motor->bias;
}

//位置式PID
//带抗积分饱和
//带微分项低通滤波
float positional_PID (struct P_pid_obj *obj, struct PID_param *pid)
{
	float differential = 0;
	
	obj->bias = obj->target - obj->measure;
	
	if (obj->output >= pid->outputMax)
	{
		if (obj->bias < 0)
			obj->integral += obj->bias;
	}
	else if (obj->output <= pid->outputMin)
	{
		if (obj->bias > 0)
			obj->integral += obj->bias;
	}
	else
	{
		obj->integral += obj->bias;
	}
	
	//微分项低通滤波
	differential = (obj->bias - obj->last_bias) * pid->differential_filterK + 
					(1 - pid->differential_filterK) * obj->last_differential;
	
	obj->output = pid->kp * obj->bias + pid->ki * obj->integral + pid->kd * differential;
	
	obj->last_bias = obj->bias;
	obj->last_differential = differential;
	
	return obj->output;
}


void pid_init(void)
{
	line_pid_param.kp = 20;//15,17
	line_pid_param.ki = 0;
	line_pid_param.kd = 1.0;//0.0
	line_pid_param.differential_filterK = 0.5;
	line_pid_param.outputMax = 500;
	line_pid_param.outputMin = -500;
	
	gyroT_pid_param.kp = 100;//95
	gyroT_pid_param.ki = 1.4;//1.4
	gyroT_pid_param.kd = 0.5;//0
	gyroT_pid_param.differential_filterK = 1;
	gyroT_pid_param.outputMax = 6500;//1500
	gyroT_pid_param.outputMin = -6500;
	
	small_gyroT_pid_param.kp = 500;
	small_gyroT_pid_param.ki = 2.5;
	small_gyroT_pid_param.kd = 0;
	small_gyroT_pid_param.differential_filterK = 1;
	small_gyroT_pid_param.outputMax = 6500;
	small_gyroT_pid_param.outputMin = -6500;
	
	gyroG_pid_param.kp = 2.5;  //3
	gyroG_pid_param.ki = 0;//0.004
	gyroG_pid_param.kd = 0.5;//0.5
	gyroG_pid_param.differential_filterK = 0.5;
	gyroG_pid_param.outputMax = 1000;
	gyroG_pid_param.outputMin = -1000;
	
	GyroP_pid_param.kp = 0.9;  //原来1.2 1.1
	GyroP_pid_param.ki = 0.004;
	GyroP_pid_param.kd = 0.5;
	GyroP_pid_param.differential_filterK = 0.5;
	GyroP_pid_param.outputMax = 500;
	GyroP_pid_param.outputMin = -500;
	
	motor_pid_clear();
}


void motor_pid_clear(void)
{
	motor_L0 = (struct I_pid_obj){0,0,0,0,0,0};
	motor_L1 = (struct I_pid_obj){0,0,0,0,0,0};
	motor_R0 = (struct I_pid_obj){0,0,0,0,0,0};
	motor_R1 = (struct I_pid_obj){0,0,0,0,0,0};
}

//usmart的调试函数，用于修改PID参数
//由于usmart不支持浮点数，所以输入一个整数和一个要除以的位数(deno)
void usmart_pid(uint16_t val,int deno,int mode)
{
//	float fval=val;
//	switch(mode)
//	{
//		case 1:
//			motor_pid_param.kp=fval/deno;  //mode1: 修改Kp
//			break;
//		case 2:
//			motor_pid_param.ki=fval/deno;  //mode2: Ki
//			break;
//		case 3:
//			motor_pid_param.kd=fval/deno;  //mode3: Kd
//			break;
//		case 4:
//			motor_L0.target=val-deno;  //mode4: Target
//			break;
//	}
//	printf("Kp:%f, Ki:%f, Kd:%f, Target:%d\r\n",
//				motor_pid_param.kp,motor_pid_param.ki,motor_pid_param.kd,motor_L0.target);
}
void chage_target(uint16_t targetq)
{
	motor_R0.target=targetq;
//	printf("%d",(int)motor_R0.target);
}
void speed_pid_kp(int param)
{
	motor_pid_paramR1.kp=param/10.0;
    motor_pid_clear();
}
void speed_pid_kd(int param)
{
	motor_pid_paramR1.kd=param/10.0;
	motor_pid_clear();
}
void speed_pid_ki(int param)
{
	motor_pid_paramR1.ki=param/100.0;
	motor_pid_clear();
}


