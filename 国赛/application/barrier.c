#include "barrier.h"
#include "sys.h"
#include "delay.h"
#include "speed_ctrl.h"
#include "motor.h"
#include "pid.h"
#include "imu_task.h"
#include "scaner.h"
#include "turn.h"
#include "map.h"
#include "motor_task.h"
#include "pid.h"
#include "math.h"
#include "bsp_buzzer.h"
#include "bsp_led.h"
#include "stdio.h"
#include "motor_task.h"
#include "QR.h"
#include "motion.h"
#include "bsp_linefollower.h"
#include "openmv.h"
#include "Rudder_control.h"
#include "encoder.h"
#include "dijkstra.h"
#include "eeprom.h"
#include "stdlib.h"

//color
//1 蓝队真宝藏
//2 红队真宝藏
//3 蓝队伪宝藏
//4 红队伪宝藏
//5 单独蓝色块
//6 单独红色块

uint8_t Area_Trea[4][2] = {0}; //每个区域的宝藏点位
uint8_t Start_Color;//1蓝0红
uint8_t j = 0;
uint8_t first_time_flag[26] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t CornerInRoad_Flag = 10;
uint8_t data[18] = {0,0,0,0,0,0,0,0,0,0,A63,A63,A63,A63,A63,A63,A63,A63};
uint8_t Be_Detected_Flag = 0;
uint8_t Reset_Length = 0;
void zhunbei(void)
{
	uint8_t _Flag = 0; //临时变量
	uint8_t Start_Flag = 0;
	uint8_t Send_Flag = 0;
	CarBrake();
	
    Start_Color = HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_8);
	
//	for(Send_Flag = 0;Send_Flag < 8;Send_Flag++)
//	{
//		Send_Byte(Send_Flag,A63);
//		vTaskDelay(100);
//	}
//	Send_Byte(Send_Flag,0);
//	vTaskDelay(50);
//	Send_Flag++;
//	Send_Byte(Send_Flag,A63);
//	vTaskDelay(50);
//	for(Send_Flag = 10;Send_Flag < 18;Send_Flag++)
//	{
//		Send_Byte(Send_Flag,A63);
//		vTaskDelay(50);
//	}
//	while(1);

//	do
//	{
//		Read_Byte(_Flag,&data[_Flag]);
//		vTaskDelay(50);
//		_Flag++;
//	}while(_Flag<18);
//	Send_Byte(7,A63);
//	vTaskDelay(50);
//	Send_Byte(3,A63);
//	vTaskDelay(50);
//	if(((Node[getNextConnectNode(A24,A23)].flag & In_Road) == In_Road)
//	|| ((Node[getNextConnectNode(A24,A23)].flag & Corner) == Corner))
//	{
//		for(uint8_t FlagFlag = 10;FlagFlag < 18;FlagFlag++)
//		{
//			if(A23 == data[FlagFlag])
//			{
//				Send_Byte(FlagFlag,A63);
//				vTaskDelay(50);
//				break;
//			}
//		}
//	}
//	while(1);
	//0-7八个宝藏点，8已去区域，9是否到达终点，10-17八个拐角/直线
	do
	{
		Read_Byte(_Flag,&data[_Flag]);
		vTaskDelay(100);
		_Flag++;
	}while(_Flag<18);  //从eeprom中读取数据
	
	if(Confirm_FirstTime() == 1) //第一次启动
	{
		buzzer_on();
		while((TreaPoint[0] == 0)&&(TreaPoint[1] == 0)&&(TreaPoint[2] == 0)&&
		  (TreaPoint[3] == 0)&&(TreaPoint[4] == 0)&&(TreaPoint[5] == 0)&&
		  (TreaPoint[6] == 0)&&(TreaPoint[7] == 0))
		{
			vTaskDelay(2);
		}
		
		for(Send_Flag = 0;Send_Flag < 8;Send_Flag++)
		{
			Send_Byte(Send_Flag,TreaPoint[Send_Flag]);
			vTaskDelay(50);
		}
		Send_Byte(Send_Flag,0); //Be_Detected_Flag
		Send_Flag++;
		vTaskDelay(50);
		if(Start_Color == 1)
		{
			Send_Byte(Send_Flag,P1);
		}
		else
		{
			Send_Byte(Send_Flag,P2);
		}
		buzzer_off();
		vTaskDelay(2000);
	}
	else if(Confirm_FirstTime() == 3)
	{
		for(uint8_t data2Point_Flag = 0;data2Point_Flag < 8;data2Point_Flag++)
		{
			TreaPoint[data2Point_Flag] = data[data2Point_Flag];
			vTaskDelay(50);
		}
	}
	
	Confirm_TreaPoint();
	
	if(Start_Color == 1)
	{
		Find_Min_Dis(P2);
	}
	else
	{
		Find_Min_Dis(P1);
	}
	
	if(Confirm_FirstTime() == 2)
	{
		if(Start_Color == 1)
		{
			dijk(&PhEleTechnology,P2,P1);
		}
		else
		{
			dijk(&PhEleTechnology,P1,P2);
		}
	}
	Route_Start();
	
	buzzer_on();
	while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15) == 0)  //等待拉高
	{
		vTaskDelay(2);
	}
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
	
	while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15))
	{
		vTaskDelay(2);
	}
	
	{
	 //切换io模式
		__HAL_RCC_GPIOB_CLK_ENABLE();
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Pin = GPIO_PIN_15;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
	
	buzzer_off();
	
	while(Infrared_ahead_LEFT==0)
	{
		vTaskDelay(2);
	}//出循环有挡板
	while(Infrared_ahead_LEFT==1)
	{
		vTaskDelay(2);
	}//出循环无挡板

	mpuZreset(imu.yaw,nodesr.nowNode.angle);
	motor_all.Cspeed = nodesr.nowNode.speed;
	pid_mode_switch(is_Line); 
}

//撞宝藏动作
void BreakTrea(void)
{
	float num;
	motor_all.Cspeed=nodesr.nowNode.speed;
	pid_mode_switch(is_Line);
	encoder_clear();
	if((nodesr.nowNode.flag&In_Road) == In_Road)
	{
		num = motor_all.Distance;
		while(fabsf(motor_all.Distance - num) < 30)
		{
			vTaskDelay(2);
		}
	}
	else
	{
		getline_error();
		while((Scaner.ledNum != 0) || (Scaner.detail != 0))
		{
			getline_error();
			vTaskDelay(2);
		}
		angle.AngleG = getAngleZ();
		motor_all.Gspeed = 2500;
		pid_mode_switch(is_Gyro);
		num = motor_all.Distance;
		while(fabsf(num - motor_all.Distance) < 10)
		{
			vTaskDelay(2);
		}
	}
	buzzer_on();
	encoder_clear();
	CarBrake();
	vTaskDelay(250);
	Be_Detected_Flag++;  //找到三个宝藏
}

void Route_Start(void)
{
	uint8_t temp = map.point;
	uint8_t k = 2;
	nodesr.nowNode = Node[getNextConnectNode(PhEleTechnology.route[0],PhEleTechnology.route[1])];
	while(1)
	{
		route[temp] = PhEleTechnology.route[k];
		temp++;
		k++;
		if(PhEleTechnology.route[k] == 255)
		{
			route[temp] = PhEleTechnology.route[k];
			break;
		}
	}
}

void Route_YesSet(void)
{
	nodesr.lastNode = nodesr.nowNode;
	
	uint8_t temp = map.point;
	uint8_t k = 2;
	
	nodesr.nowNode = Node[getNextConnectNode(PhEleTechnology.route[0],PhEleTechnology.route[1])];
	
	if(nodesr.lastNode.angle != nodesr.nowNode.angle)
	{
		Change_ScanMode();
		nodesr.lastNode.angle += 180;
		if(nodesr.lastNode.angle == 270)
		{
			nodesr.lastNode.angle = -90;
		}
		else if(nodesr.lastNode.angle == 360)
		{
			nodesr.lastNode.angle = 0;
		}
		mpuZreset(imu.yaw,nodesr.lastNode.angle);
	}
	
	while(1)
	{
		route[temp] = PhEleTechnology.route[k];
		temp++;
		k++;
		if(PhEleTechnology.route[k] == 255)
		{
			route[temp] = PhEleTechnology.route[k];
			
			if(nodesr.lastNode.angle != nodesr.nowNode.angle)
			{
				int break_time = 0;
				float relativeAngle;
				angle.AngleT=getAngleZ()+need2turn(nodesr.lastNode.angle,nodesr.nowNode.angle);
				if(angle.AngleT>180)	angle.AngleT -= 360;
				else if(angle.AngleT<-180)	angle.AngleT += 360;
				relativeAngle = angle.AngleT;
				pid_mode_switch(is_Turn);	
				while(fabsf(need2turn(angle.AngleT,getAngleZ())) > 5)
				{
					break_time++;
					float now_angle = getAngleZ();						
					getline_error();
					if(Scaner.lineNum==1 && (fabsf(need2turn(relativeAngle,getAngleZ()))<27/*fabsf(need2turn(now_angle,angle.AngleT))*0.3f*/) && break_time >= 500)
					{
						if(Scaner.detail&0x03)
						{
							pid_mode_switch(is_Free);
							if(ScanMode == is_Front)
							{
								motor_set_pwm(1, 1500);
								motor_set_pwm(2, 1500);
								motor_set_pwm(3, -1500);
								motor_set_pwm(4, -1500);
							}
							if(ScanMode == is_Back)//
							{
								motor_set_pwm(1, 1500);
								motor_set_pwm(2, 1500);
								motor_set_pwm(3, -1500);
								motor_set_pwm(4, -1500);
							}
						}
						vTaskDelay(1);
					}
					if(Scaner.lineNum==1 && (fabsf(need2turn(relativeAngle,getAngleZ()))<27/*fabsf(need2turn(now_angle,angle.AngleT))*0.3f*/) && break_time >= 500)
					{
						if(Scaner.detail&0xC0)
						{
							pid_mode_switch(is_Free);
							if(ScanMode == is_Front)
							{
								motor_set_pwm(1, -1500);
								motor_set_pwm(2, -1500);
								motor_set_pwm(3, 1500);
								motor_set_pwm(4, 1500);
							}
							if(ScanMode == is_Back)//
							{
								motor_set_pwm(1, -1500);
								motor_set_pwm(2, -1500);
								motor_set_pwm(3, 1500);
								motor_set_pwm(4, 1500);
							}
						}
						vTaskDelay(1);
					}
					if(Scaner.lineNum==1&&(Scaner.detail&0x18)&&(fabsf(need2turn(angle.AngleT,getAngleZ()))</*fabsf(need2turn(now_angle,angle.AngleT))*0.3f*/27))
					{
						break;
					}
					vTaskDelay(2);
					if(break_time >= 1500)
						break;
					if(Scaner.ledNum == 0 && motor_L0.measure < 10 && motor_R0.measure < 10 && motor_L1.measure < 10 && motor_R1.measure < 10 &&(fabsf(need2turn(angle.AngleT,getAngleZ()))<27))
						break;
					
				}
				break_time = 0;
				getline_error();
				if(Scaner.ledNum == 0)
				{
					struct PID_param origin_parm = gyroT_pid_param;
					gyroT_pid_param.ki = 0.5;
					
					float Angle11;
					if(nodesr.lastNode.angle - nodesr.nowNode.angle == -90 || nodesr.lastNode.angle - nodesr.nowNode.angle == 270)
						angle.AngleT = Angle11 = getAngleZ()+30;
					else if(nodesr.lastNode.angle - nodesr.nowNode.angle == -270 || nodesr.lastNode.angle - nodesr.nowNode.angle == 90)
						angle.AngleT = Angle11 = getAngleZ()-30;
					
					pid_mode_switch(is_Turn);	
					while(fabsf(Angle11-getAngleZ()) > 5)
					{
						break_time++;
						char now_angle = getAngleZ();						
						getline_error();
						static int seed;
						if(Scaner.lineNum ==1 && (fabsf(need2turn(angle.AngleT,getAngleZ()))<27/*fabsf(need2turn(now_angle,angle.AngleT))*0.3f*/) && break_time >= 500)
						{
							if(Scaner.detail&0x03)
							{
								pid_mode_switch(is_Free);
								if(ScanMode == is_Front)//右转
								{
									motor_set_pwm(1, 1500);
									motor_set_pwm(2, 1500);
									motor_set_pwm(3, -1500);
									motor_set_pwm(4, -1500);
								}
								if(ScanMode == is_Back)//
								{
									motor_set_pwm(1, 1500);
									motor_set_pwm(2, 1500);
									motor_set_pwm(3, -1500);
									motor_set_pwm(4, -1500);
								}
							}
							vTaskDelay(1);
						}
						if(Scaner.lineNum==1 && (fabsf(need2turn(angle.AngleT,getAngleZ()))<27/*fabsf(need2turn(now_angle,angle.AngleT))*0.3f*/) && break_time >= 500)
						{
							if(Scaner.detail&0xC0)
							{
								pid_mode_switch(is_Free);
								if(ScanMode == is_Front)
								{
									motor_set_pwm(1, -1500);
									motor_set_pwm(2, -1500);
									motor_set_pwm(3, 1500);
									motor_set_pwm(4, 1500);
								}
								if(ScanMode == is_Back)//
								{
									motor_set_pwm(1, -1500);
									motor_set_pwm(2, -1500);
									motor_set_pwm(3, 1500);
									motor_set_pwm(4, 1500);
								}
							}
							vTaskDelay(1);
						}
						if(Scaner.lineNum==1
						&&(Scaner.detail&0x18)
						&&(fabsf(need2turn(angle.AngleT,getAngleZ()))<27/*fabsf(need2turn(now_angle,angle.AngleT))*0.3f*/))
						{
							break;
						}
						vTaskDelay(2);
						if(break_time >= 1500)
							break;
					}	
					break_time = 0;
					gyroT_pid_param = origin_parm;
				}
			}
			
			motor_all.Cspeed = nodesr.nowNode.speed;
			pid_mode_switch(is_Line);
			
			nodesr.nowNode.flag |= STOPTURN;
			imu.compensateZ=need2turn(imu.yaw,nodesr.nowNode.angle);
			
			nodesr.flag|=0x80;
			
			break;
		}
	}
}

void Route_Set(void)
{
	nodesr.lastNode = nodesr.nowNode;
	
	nodesr.lastNode.angle += 180;
	if(nodesr.lastNode.angle == 270)
	{
		nodesr.lastNode.angle = -90;
	}
	else if(nodesr.lastNode.angle == 360)
	{
		nodesr.lastNode.angle = 0;
	}
	
	mpuZreset(imu.yaw,nodesr.lastNode.angle);
	
	uint8_t temp,k;
	if(Long_Trea()) //长宝藏
	{
		temp = map.point;
		k = 2;
		nodesr.nowNode = Node[getNextConnectNode(PhEleTechnology.route[0],PhEleTechnology.route[1])];
	}
	else
	{
		temp = map.point;
		k = 3;
		nodesr.nowNode = Node[getNextConnectNode(PhEleTechnology.route[1],PhEleTechnology.route[2])];
	}
	
	while(1)
	{
		route[temp] = PhEleTechnology.route[k];
		temp++;
		k++;
		if(PhEleTechnology.route[k] == 255)
		{
			route[temp] = PhEleTechnology.route[k];
			
			Change_ScanMode();
			
			if(nodesr.lastNode.angle != nodesr.nowNode.angle)
			{
				int break_time = 0;
				float relativeAngle;
				angle.AngleT=getAngleZ()+need2turn(nodesr.lastNode.angle,nodesr.nowNode.angle);
				if(angle.AngleT>180)	angle.AngleT -= 360;
				else if(angle.AngleT<-180)	angle.AngleT += 360;
				relativeAngle = angle.AngleT;
				pid_mode_switch(is_Turn);	
				while(fabsf(need2turn(angle.AngleT,getAngleZ())) > 5)
				{
					break_time++;
					float now_angle = getAngleZ();						
					getline_error();
					if(Scaner.lineNum==1 && (fabsf(need2turn(relativeAngle,getAngleZ()))<27/*fabsf(need2turn(now_angle,angle.AngleT))*0.3f*/) && break_time >= 500)
					{
						if(Scaner.detail&0x03)
						{
							pid_mode_switch(is_Free);
							if(ScanMode == is_Front)
							{
								motor_set_pwm(1, 1500);
								motor_set_pwm(2, 1500);
								motor_set_pwm(3, -1500);
								motor_set_pwm(4, -1500);
							}
							if(ScanMode == is_Back)//
							{
								motor_set_pwm(1, 1500);
								motor_set_pwm(2, 1500);
								motor_set_pwm(3, -1500);
								motor_set_pwm(4, -1500);
							}
						}
						vTaskDelay(1);
					}
					if(Scaner.lineNum==1 && (fabsf(need2turn(relativeAngle,getAngleZ()))<27/*fabsf(need2turn(now_angle,angle.AngleT))*0.3f*/) && break_time >= 500)
					{
						if(Scaner.detail&0xC0)
						{
							pid_mode_switch(is_Free);
							if(ScanMode == is_Front)
							{
								motor_set_pwm(1, -1500);
								motor_set_pwm(2, -1500);
								motor_set_pwm(3, 1500);
								motor_set_pwm(4, 1500);
							}
							if(ScanMode == is_Back)//
							{
								motor_set_pwm(1, -1500);
								motor_set_pwm(2, -1500);
								motor_set_pwm(3, 1500);
								motor_set_pwm(4, 1500);
							}
						}
						vTaskDelay(1);
					}
					if(Scaner.lineNum==1&&(Scaner.detail&0x18)&&(fabsf(need2turn(angle.AngleT,getAngleZ()))</*fabsf(need2turn(now_angle,angle.AngleT))*0.3f*/27))
					{
						break;
					}
					vTaskDelay(2);
					if(break_time >= 1500)
						break;
					if(Scaner.ledNum == 0 && motor_L0.measure < 10 && motor_R0.measure < 10 && motor_L1.measure < 10 && motor_R1.measure < 10 &&(fabsf(need2turn(angle.AngleT,getAngleZ()))<27))
						break;
					
				}
				break_time = 0;
				getline_error();
				if(Scaner.ledNum == 0)
				{
					struct PID_param origin_parm = gyroT_pid_param;
					gyroT_pid_param.ki = 0.5;
					
					float Angle11;
					if(nodesr.lastNode.angle - nodesr.nowNode.angle == -90 || nodesr.lastNode.angle - nodesr.nowNode.angle == 270)
						angle.AngleT = Angle11 = getAngleZ()+30;
					else if(nodesr.lastNode.angle - nodesr.nowNode.angle == -270 || nodesr.lastNode.angle - nodesr.nowNode.angle == 90)
						angle.AngleT = Angle11 = getAngleZ()-30;
					
					pid_mode_switch(is_Turn);	
					while(fabsf(Angle11-getAngleZ()) > 5)
					{
						break_time++;
						char now_angle = getAngleZ();						
						getline_error();
						static int seed;
						if(Scaner.lineNum ==1 && (fabsf(need2turn(angle.AngleT,getAngleZ()))<27/*fabsf(need2turn(now_angle,angle.AngleT))*0.3f*/) && break_time >= 500)
						{
							if(Scaner.detail&0x03)
							{
								pid_mode_switch(is_Free);
								if(ScanMode == is_Front)//右转
								{
									motor_set_pwm(1, 1500);
									motor_set_pwm(2, 1500);
									motor_set_pwm(3, -1500);
									motor_set_pwm(4, -1500);
								}
								if(ScanMode == is_Back)//
								{
									motor_set_pwm(1, 1500);
									motor_set_pwm(2, 1500);
									motor_set_pwm(3, -1500);
									motor_set_pwm(4, -1500);
								}
							}
							vTaskDelay(1);
						}
						if(Scaner.lineNum==1 && (fabsf(need2turn(angle.AngleT,getAngleZ()))<27/*fabsf(need2turn(now_angle,angle.AngleT))*0.3f*/) && break_time >= 500)
						{
							if(Scaner.detail&0xC0)
							{
								pid_mode_switch(is_Free);
								if(ScanMode == is_Front)
								{
									motor_set_pwm(1, -1500);
									motor_set_pwm(2, -1500);
									motor_set_pwm(3, 1500);
									motor_set_pwm(4, 1500);
								}
								if(ScanMode == is_Back)//
								{
									motor_set_pwm(1, -1500);
									motor_set_pwm(2, -1500);
									motor_set_pwm(3, 1500);
									motor_set_pwm(4, 1500);
								}
							}
							vTaskDelay(1);
						}
						if(Scaner.lineNum==1
						&&(Scaner.detail&0x18)
						&&(fabsf(need2turn(angle.AngleT,getAngleZ()))<27/*fabsf(need2turn(now_angle,angle.AngleT))*0.3f*/))
						{
							break;
						}
						vTaskDelay(2);
						if(break_time >= 1500)
							break;
					}	
					break_time = 0;
					gyroT_pid_param = origin_parm;
				}
			}
			
			motor_all.Cspeed = nodesr.nowNode.speed;
			pid_mode_switch(is_Line);
			imu.compensateZ=need2turn(imu.yaw,nodesr.nowNode.angle);
			
			nodesr.flag|=0x80;
			
			break;
		}
	}
}

void TREASURE(void)
{
	static uint8_t Detected_Area = 0; //已探明区域
	uint8_t k,m,now_color= 0;
	float num = 0;
	uint8_t Break_Flag = 0;
	int time ,turn_time= 0;
	uint8_t first_time1 = 0;
	
	encoder_clear();
	if(Long_Trea())
	{
		num = motor_all.Distance;
		while(fabsf(num - motor_all.Distance) < 20)
		{
			vTaskDelay(2);
		}
	}
	
	CarBrake();
	
	if(ScanMode == is_Front)
		open_mvR();
	else
		open_mv();
	
	vTaskDelay(1500);
	
	if(Start_Color == 1)    //蓝色启动
	{
		while(color == 0)
		{
			vTaskDelay(2);
			time++;
			if((time > 750) && (first_time1 == 0))
			{
				first_time1 = 1;
				angle.AngleT = getAngleZ()+10;
				pid_mode_switch(is_SmallTurn);
				while(fabsf(angle.AngleT - getAngleZ()) > 5)
				{
					turn_time++;
					vTaskDelay(2);
					if(turn_time >= 500)
						break;
				}
				turn_time = 0;
				time = 0;
			}
			if((time > 750) && (first_time1 == 1))
			{
				first_time1 = 2;
				angle.AngleT = getAngleZ()-15;
				pid_mode_switch(is_SmallTurn);
				while(fabsf(angle.AngleT - getAngleZ()) > 5)
				{
					turn_time++;
					vTaskDelay(2);
					if(turn_time >= 500)
						break;
				}
				turn_time = 0;
				time = 0;
			}
			if(time > 1500)
			{
				angle.AngleT = nodesr.nowNode.angle;
				pid_mode_switch(is_SmallTurn);
				while(fabsf(angle.AngleT - getAngleZ()) > 5)
				{
					turn_time++;
					vTaskDelay(2);
					getline_error();
					if(Scaner.lineNum==1&&(Scaner.detail&0x3C)&&turn_time>=50/*&&(fabsf(need2turn(angle.AngleT,getAngleZ()))<fabsf(need2turn(angle.AngleT,nodesr.nowNode.angle))*0.3f)*/)
						break;
					if(turn_time >= 500)
						break;
				}
				turn_time = 0;
				time = 0;
				TreaPoint[j] = A63;
				Point2data(TreaPoint[j]);
				if((nodesr.nowNode.flag & In_Road) == In_Road || (nodesr.nowNode.flag & Corner) == Corner)
				{
					Set_RoadLength(nodesr.nowNode.nodenum);
				}
				break;
			}
		}
		if(color == 1 ||color == 3)//己方宝藏或者是己方伪宝藏
		{
			if(color == 1)
			{//撞宝藏动作
				now_color = 1;
				BreakTrea();
				if(((nodesr.nowNode.flag & In_Road) == In_Road)
				|| ((nodesr.nowNode.flag & Corner) == Corner))
				{
					for(uint8_t FlagFlag = 10;FlagFlag < 18;FlagFlag++)
					{
						if(nodesr.nowNode.nodenum == data[FlagFlag])
						{
							Send_Byte(FlagFlag,A63);
							vTaskDelay(50);
							break;
						}
					}
				}
			}
			
			for(k = 0;k < 4;k++)     
			{
				for(m = 0;m < 2;m++)
				{
					if(nodesr.nowNode.nodenum == Area_Trea[k][m])  //找到己方宝藏对应的区域
					{
						Break_Flag = 1;
						break;
					}
				}
				if(Break_Flag == 1)
				{
					Break_Flag = 0;
					break;
				}
			}
			for(uint8_t i = 0;i < 8;i++)   //不去找己方宝藏区域的其他宝藏
			{
				if(TreaPoint[i] == Area_Trea[k][0] || TreaPoint[i] == Area_Trea[k][1])
				{
					TreaPoint[i] = A63;
					Point2data(TreaPoint[i]);
				}
			}
			for(uint8_t i = 0;i < 8;i++)
			{
				if(SymPoint(nodesr.nowNode.nodenum) == TreaPoint[i])
				{
					TreaPoint[i] = A63;
					Point2data(TreaPoint[i]);
					break;
				}
			}
			if((nodesr.nowNode.flag & In_Road) == In_Road || (nodesr.nowNode.flag & Corner) == Corner)
			{
				SYM_Set_RoadLength(SymPoint(nodesr.nowNode.nodenum));
			}
			if(color == 3)
			{
				if((nodesr.nowNode.flag & In_Road) == In_Road || (nodesr.nowNode.flag & Corner) == Corner)
				{
					Set_RoadLength(nodesr.nowNode.nodenum);
				}
			}
		}
		else if(color == 2 || color == 4) //发现的是假宝藏
		{
			TreaPoint[j] = A63;
			Point2data(TreaPoint[j]);
		
			for(k = 0;k < 4;k++)
			{
				for(m = 0;m < 2;m++)
				{
					if(SymPoint(nodesr.nowNode.nodenum) == Area_Trea[k][m])
					{
						Break_Flag = 1;
						break;
					}
				}
				if(Break_Flag == 1)
				{
					Break_Flag = 0;
					break;
				}
			}
			if(m == 0)
				m = 1;
			else if(m == 1)
				m = 0;
			for(uint8_t i = 0;i < 8;i++)
			{
				if(Area_Trea[k][m] == TreaPoint[i])
				{
					TreaPoint[i] = A63;
					Point2data(TreaPoint[i]);
					break;
				}
			}
			if((nodesr.nowNode.flag & In_Road) == In_Road || (nodesr.nowNode.flag & Corner) == Corner)
			{
				Set_RoadLength(nodesr.nowNode.nodenum);
			}
		}
	}
	else
	{
		while(color == 0)
		{
			vTaskDelay(2);
			time++;
			if((time > 750) && (first_time1 == 0))
			{
				first_time1 = 1;
				angle.AngleT = getAngleZ()+10;
				pid_mode_switch(is_SmallTurn);
				while(fabsf(angle.AngleT - getAngleZ()) > 5)
				{
					turn_time++;
					vTaskDelay(2);
					if(turn_time >= 500)
						break;
				}
				turn_time = 0;
				time = 0;
			}
			if((time > 750) && (first_time1 == 1))
			{
				first_time1 = 2;
				angle.AngleT = getAngleZ()-10;
				pid_mode_switch(is_SmallTurn);
				while(fabsf(angle.AngleT - getAngleZ()) > 5)
				{
					turn_time++;
					vTaskDelay(2);
					if(turn_time >= 500)
						break;
				}
				turn_time = 0;
				time = 0;
			}
			if(time > 1500)
			{
				angle.AngleT = nodesr.nowNode.angle;
				pid_mode_switch(is_SmallTurn);
				while(fabsf(angle.AngleT - getAngleZ()) > 5)
				{
					turn_time++;
					vTaskDelay(2);
					getline_error();
					if(Scaner.lineNum==1&&(Scaner.detail&0x3C)&&turn_time>=50/*&&(fabsf(need2turn(angle.AngleT,getAngleZ()))<fabsf(need2turn(angle.AngleT,nodesr.nowNode.angle))*0.3f)*/)
						break;
					if(turn_time >= 500)
						break;
				}
				turn_time = 0;
				time = 0;
				TreaPoint[j] = A63;
				Point2data(TreaPoint[j]);
				if((nodesr.nowNode.flag & In_Road) == In_Road || (nodesr.nowNode.flag & Corner) == Corner)
				{
					Set_RoadLength(nodesr.nowNode.nodenum);
				}
				break;
			}
		}
		if(color == 2 ||color == 4)//己方宝藏或者是假宝藏
		{
			if(color == 2)
			{//撞宝藏动作
				now_color = 2;
				BreakTrea();
				if(((nodesr.nowNode.flag & In_Road) == In_Road)
				|| ((nodesr.nowNode.flag & Corner) == Corner))
				{
					for(uint8_t FlagFlag = 10;FlagFlag < 18;FlagFlag++)
					{
						if(nodesr.nowNode.nodenum == data[FlagFlag])
						{
							Send_Byte(FlagFlag,A63);
							vTaskDelay(50);
							break;
						}
					}
				}
			}

			for(k = 0;k < 4;k++)     
			{
				for(m = 0;m < 2;m++)
				{
					if(nodesr.nowNode.nodenum == Area_Trea[k][m])  //找到己方宝藏对应的区域
					{
						Break_Flag = 1;
						break;
					}
				}
				if(Break_Flag == 1)
				{
					Break_Flag = 0;
					break;
				}
			}
			for(uint8_t i = 0;i < 8;i++)   //不去找己方宝藏区域的其他宝藏
			{
				if(TreaPoint[i] == Area_Trea[k][0] || TreaPoint[i] == Area_Trea[k][1])
				{
					TreaPoint[i] = A63;
					Point2data(TreaPoint[i]);
				}
			}
			for(uint8_t i = 0;i < 8;i++)
			{
				if(SymPoint(nodesr.nowNode.nodenum) == TreaPoint[i])
				{
					TreaPoint[i] = A63;
					Point2data(TreaPoint[i]);
					break;
				}
			}
			if((nodesr.nowNode.flag & In_Road) == In_Road || (nodesr.nowNode.flag & Corner) == Corner)
			{
				SYM_Set_RoadLength(SymPoint(nodesr.nowNode.nodenum));
			}
			if(color == 4)
			{
				if((nodesr.nowNode.flag & In_Road) == In_Road || (nodesr.nowNode.flag & Corner) == Corner)
				{
					Set_RoadLength(nodesr.nowNode.nodenum);
				}
			}
		}
		else if(color == 1 || color == 3) //发现的是假宝藏
		{
			TreaPoint[j] = A63;
			Point2data(TreaPoint[j]);
			
			for(k = 0;k < 4;k++)
			{
				for(m = 0;m < 2;m++)
				{
					if(SymPoint(nodesr.nowNode.nodenum) == Area_Trea[k][m])
					{
						Break_Flag = 1;
						break;
					}
				}
				if(Break_Flag == 1)
				{
					Break_Flag = 0;
					break;
				}
			}
			if(m == 0)
				m = 1;
			else if(m == 1)
				m = 0;
			for(uint8_t i = 0;i < 8;i++)
			{
				if(Area_Trea[k][m] == TreaPoint[i])
				{
					TreaPoint[i] = A63;
					Point2data(TreaPoint[i]);
					break;
				}
			}
			if((nodesr.nowNode.flag & In_Road) == In_Road || (nodesr.nowNode.flag & Corner) == Corner)
			{
				Set_RoadLength(nodesr.nowNode.nodenum);
			}
		}
	}
	
	close_mv();
	close_mvR();
	
	uint8_t nodenodenum = nodesr.nowNode.nodenum;
	
	CreatGraph(&PhEleTechnology_Test);
	CreatGraph(&PhEleTechnology);
	
	
	buzzer_off();
	
	if(TreaPoint[0] == A63 && TreaPoint[1] == A63 && TreaPoint[2] == A63 && TreaPoint[3] == A63 
	&& TreaPoint[4] == A63 && TreaPoint[5] == A63 && TreaPoint[6] == A63 && TreaPoint[7] == A63)
	{
		Detected_Area = 4;
	}
	
	if(Be_Detected_Flag == 3 || Detected_Area ==4)
	{
		if(Start_Color == 1)
		{
			dijk(&PhEleTechnology,nodesr.nowNode.nodenum,P1);
		}
		else
		{
			dijk(&PhEleTechnology,nodesr.nowNode.nodenum,P2);
		}
	}
	else
	{
		Find_Min_Dis(nodesr.nowNode.nodenum);
	}
	
	if((Start_Color == 1 && now_color == 1) || (Start_Color == 0 && now_color == 2))
		Route_YesSet();
	else
		Route_Set();
	
	if(Reset_Length)
	{
		for(uint8_t i = 0;i < ConnectionNum[nodenodenum];i++)
		{
			Node[Address[nodenodenum]+i].step = 1000;
		}
	}
	Reset_Length = 0;
}

void PLATFORM(void)
{
	float num = 0;
	encoder_clear();
	motor_all.Cspeed = 4000;
	num = motor_all.Distance;
	while(fabsf(motor_all.Distance - num) < 60)
	{
		vTaskDelay(2);
	}
	for(uint8_t Send_Flag = 0;Send_Flag < 8;Send_Flag++)
	{
		Send_Byte(Send_Flag,A63);
		vTaskDelay(50);
	}
	Send_Byte(9,A63);
	vTaskDelay(50);
	CarBrake();
	while(1)
	{
		vTaskDelay(2);
	}
}

void Find_Min_Dis(uint8_t nowNodeNum)
{
	int distance[8] = {0};
	uint8_t i = 0;
	int Min_Dis;
	for(i = 0;i < 8;i++)
	{
		distance[i] = dijk(&PhEleTechnology_Test,nowNodeNum,TreaPoint[i]);
	}
	
	Min_Dis = distance[0];
	j = 0;
	for(i = 1;i < 8;i++)
	{
		if(distance[i] < Min_Dis)
		{
			Min_Dis = distance[i];
			j = i;
		}
	}
	
	dijk(&PhEleTechnology,nowNodeNum,TreaPoint[j]);
}

void Point2data(uint8_t nodenum)
{
	uint8_t flag = 0;
	for(flag = 0;flag < 8;flag++)
	{
		if(nodenum == TreaPoint[flag])
		{
			Send_Byte(flag,A63);
			vTaskDelay(50);
			break;
		}
	}
}

uint8_t Confirm_FirstTime(void)
{
	if((data[0] == A63) && (data[1] == A63) && (data[2] == A63) && (data[3] == A63) 
	&& (data[4] == A63) && (data[5] == A63) && (data[6] == A63) && (data[7] == A63) && (data[9] == A63))
	{
		return 1;
	}
	else if((data[0] == A63) && (data[1] == A63) && (data[2] == A63) && (data[3] == A63) 
		 && (data[4] == A63) && (data[5] == A63) && (data[6] == A63) && (data[7] == A63)  && (data[9] != A63))
	{
		return 2;
	}
	else
	{
		return 3;
	}
}

uint8_t Long_Trea(void)
{
	if(nodesr.nowNode.nodenum == B3 || nodesr.nowNode.nodenum == B5 || nodesr.nowNode.nodenum == B6
	|| nodesr.nowNode.nodenum == B11 || nodesr.nowNode.nodenum == B12 || nodesr.nowNode.nodenum == B14)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t SymPoint(uint8_t nodenum)
{
	uint8_t symPoint;
	symPoint = 95 - nodenum;
	return symPoint;
}

void Set_RoadLength(uint8_t nodenum)
{
	for(uint8_t i = 0;i < 204;i++)
	{
		if(Node[i].nodenum == nodenum)
		{
			Node[i].step = 1000;
		}
	}
	for(uint8_t i = 0;i < ConnectionNum[nodenum];i++)
	{
		if(Node[Address[nodenum]+i].nodenum != nodesr.lastNode.nodenum)
		{
			Node[Address[nodenum]+i].step = 1000;
		}
	}
	Reset_Length = 1;
}

void NoReset_Length(uint8_t nodenum)
{
	for(uint8_t i = 0;i < 204;i++)
	{
		if(Node[i].nodenum == nodenum)
		{
			Node[i].step = 1000;
		}
	}
	for(uint8_t i = 0;i < ConnectionNum[nodenum];i++)
	{
		Node[Address[nodenum]+i].step = 1000;
	}
}

void SYM_Set_RoadLength(uint8_t nodenum)
{
	for(uint8_t i = 0;i < 204;i++)
	{
		if(Node[i].nodenum == nodenum)
		{
			Node[i].step = 1000;
		}
	}
	for(uint8_t i = 0;i < ConnectionNum[nodenum];i++)
	{
		Node[Address[nodenum]+i].step = 1000;
	}
	Reset_Length = 1;
}

void Change_ScanMode(void)
{
	if(ScanMode == is_Back)
	{
		ScanMode = is_Front;
	}
	else
	{
		ScanMode = is_Back;
	}
}

void Confirm_TreaPoint(void)
{
	uint8_t i,k,s;
	uint8_t m = 0;
	uint8_t n = 0;
	uint8_t q = 0;
	uint8_t p = 0;
	for(i = 0;i < 8;i++)
	{
		if(TreaPoint[i] == A1)     //区域一
		{
			Area_Trea[0][m] = TreaPoint[i];
			m++;
		}
		if(TreaPoint[i] == L1)
		{
			Area_Trea[0][m] = TreaPoint[i];
			m++;
		}
		if(TreaPoint[i] == A4)
		{
			Area_Trea[0][m] = TreaPoint[i];
			m++;
		}
		if(TreaPoint[i] == L3)
		{
			Area_Trea[0][m] = TreaPoint[i];
			m++;
		}
		if(TreaPoint[i] == B1)
		{
			Area_Trea[0][m] = TreaPoint[i];
			m++;
		}
		if(TreaPoint[i] == A8)
		{
			Area_Trea[0][m] = TreaPoint[i];
			m++;
		}
		if(TreaPoint[i] == A9)
		{
			Area_Trea[0][m] = TreaPoint[i];
			m++;
		}
		if(TreaPoint[i] == B3)
		{
			Area_Trea[0][m] = TreaPoint[i];
			m++;
		}
		if(TreaPoint[i] == A14)
		{
			Area_Trea[0][m] = TreaPoint[i];
			m++;
		}
		if(TreaPoint[i] == A16)
		{
			Area_Trea[0][m] = TreaPoint[i];
			m++;
		}
		if(TreaPoint[i] == B4)
		{
			Area_Trea[0][m] = TreaPoint[i];
			m++;
		}
		if(TreaPoint[i] == A20)
		{
			Area_Trea[0][m] = TreaPoint[i];
			m++;
		}
		if(TreaPoint[i] == A21)
		{
			Area_Trea[0][m] = TreaPoint[i];
			m++;
		}
		if(TreaPoint[i] == L6)
		{
			Area_Trea[0][m] = TreaPoint[i];
			m++;
		}
		if(TreaPoint[i] == L7)
		{
			Area_Trea[0][m] = TreaPoint[i];
			m++;
		}
		if(TreaPoint[i] == L8)
		{
			Area_Trea[0][m] = TreaPoint[i];
			m++;
		}
		if(TreaPoint[i] == B8)
		{
			Area_Trea[0][m] = TreaPoint[i];
			m++;
		}
		if(TreaPoint[i] == A26)
		{
			Area_Trea[0][m] = TreaPoint[i];
			m++;
		}
		
	
		
		if(TreaPoint[i] == A61)   //区域二
		{
			Area_Trea[1][n] = TreaPoint[i];
			n++;
		}
		if(TreaPoint[i] == A5)    
		{
			Area_Trea[1][n] = TreaPoint[i];
			n++;
		}
		if(TreaPoint[i] == L2)
		{
			Area_Trea[1][n] = TreaPoint[i];
			n++;
		}
		if(TreaPoint[i] == A6)
		{
			Area_Trea[1][n] = TreaPoint[i];
			n++;
		}
		if(TreaPoint[i] == L4)
		{
			Area_Trea[1][n] = TreaPoint[i];
			n++;
		}
		if(TreaPoint[i] == L5)
		{
			Area_Trea[1][n] = TreaPoint[i];
			n++;
		}
		if(TreaPoint[i] == B5)
		{
			Area_Trea[1][n] = TreaPoint[i];
			n++;
		}
		if(TreaPoint[i] == B6)
		{
			Area_Trea[1][n] = TreaPoint[i];
			n++;
		}
		if(TreaPoint[i] == A23)
		{
			Area_Trea[1][n] = TreaPoint[i];
			n++;
		}
		if(TreaPoint[i] == B7)
		{
			Area_Trea[1][n] = TreaPoint[i];
			n++;
		}
		if(TreaPoint[i] == L9)
		{
			Area_Trea[1][n] = TreaPoint[i];
			n++;
		}
		
		
		
		if(TreaPoint[i] == L10)    //区域三
		{
			Area_Trea[2][q] = TreaPoint[i];
			q++;
		}
		if(TreaPoint[i] == B10)
		{
			Area_Trea[2][q] = TreaPoint[i];
			q++;
		}
		if(TreaPoint[i] == A38)
		{
			Area_Trea[2][q] = TreaPoint[i];
			q++;
		}
		if(TreaPoint[i] == A62)
		{
			Area_Trea[2][q] = TreaPoint[i];
			q++;
		}
		if(TreaPoint[i] == B11)
		{
			Area_Trea[2][q] = TreaPoint[i];
			q++;
		}
		if(TreaPoint[i] == B12)
		{
			Area_Trea[2][q] = TreaPoint[i];
			q++;
		}
		if(TreaPoint[i] == L14)
		{
			Area_Trea[2][q] = TreaPoint[i];
			q++;
		}
		if(TreaPoint[i] == L15)
		{
			Area_Trea[2][q] = TreaPoint[i];
			q++;
		}
		if(TreaPoint[i] == A55)
		{
			Area_Trea[2][q] = TreaPoint[i];
			q++;
		}
		if(TreaPoint[i] == L17)
		{
			Area_Trea[2][q] = TreaPoint[i];
			q++;
		}
		if(TreaPoint[i] == A56)
		{
			Area_Trea[2][q] = TreaPoint[i];
			q++;
		}
		
		
		if(TreaPoint[i] == A35)    //区域四
		{
			Area_Trea[3][p] = TreaPoint[i];
			p++;
		}
		if(TreaPoint[i] == B13)
		{
			Area_Trea[3][p] = TreaPoint[i];
			p++;
		}
		if(TreaPoint[i] == B9)
		{
			Area_Trea[3][p] = TreaPoint[i];
			p++;
		}
		if(TreaPoint[i] == L11)
		{
			Area_Trea[3][p] = TreaPoint[i];
			p++;
		}
		if(TreaPoint[i] == L12)
		{
			Area_Trea[3][p] = TreaPoint[i];
			p++;
		}
		if(TreaPoint[i] == L13)
		{
			Area_Trea[3][p] = TreaPoint[i];
			p++;
		}
		if(TreaPoint[i] == A40)
		{
			Area_Trea[3][p] = TreaPoint[i];
			p++;
		}
		if(TreaPoint[i] == A41)
		{
			Area_Trea[3][p] = TreaPoint[i];
			p++;
		}
		if(TreaPoint[i] == A45)
		{
			Area_Trea[3][p] = TreaPoint[i];
			p++;
		}
		if(TreaPoint[i] == A47)
		{
			Area_Trea[3][p] = TreaPoint[i];
			p++;
		}
		if(TreaPoint[i] == B14)
		{
			Area_Trea[3][p] = TreaPoint[i];
			p++;
		}
		if(TreaPoint[i] == A52)
		{
			Area_Trea[3][p] = TreaPoint[i];
			p++;
		}
		if(TreaPoint[i] == A53)
		{
			Area_Trea[3][p] = TreaPoint[i];
			p++;
		}
		if(TreaPoint[i] == B16)
		{
			Area_Trea[3][p] = TreaPoint[i];
			p++;
		}
		if(TreaPoint[i] == L16)
		{
			Area_Trea[3][p] = TreaPoint[i];
			p++;
		}
		if(TreaPoint[i] == A57)
		{
			Area_Trea[3][p] = TreaPoint[i];
			p++;
		}
		if(TreaPoint[i] == L18)
		{
			Area_Trea[3][p] = TreaPoint[i];
			p++;
		}
		if(TreaPoint[i] == A60)
		{
			Area_Trea[3][p] = TreaPoint[i];
			p++;
		}
		
		for(k = 0;k < 204;k++)
		{
			if(TreaPoint[i] == Node[k].nodenum)
			{
				Node[k].function = Treasure;
				Set_CornerOrInRoad(Node[k].nodenum);
			}
		}
	}
	
	for(i = 10;i < 18;i++)
	{
		for(s = 0;s < 8;s++)
		{
			if(data[s] == A63 && data[i] != A63)
			{
				NoReset_Length(data[i]);
			}
		}
	}
	
	CreatGraph(&PhEleTechnology_Test);
	CreatGraph(&PhEleTechnology);
}

void Set_CornerOrInRoad(uint8_t nodenum)
{
	if(nodenum == A4)
	{
		if(first_time_flag[0] == 0)
		{
			first_time_flag[0]++;
			Send_Byte(CornerInRoad_Flag,nodenum);
			CornerInRoad_Flag++;
			vTaskDelay(50);
		}
	}
	else if(nodenum == A8)
	{
		if(first_time_flag[1] == 0)
		{
			first_time_flag[1]++;
			Send_Byte(CornerInRoad_Flag,nodenum);
			CornerInRoad_Flag++;
			vTaskDelay(50);
		}
	}
	else if(nodenum == A9)
	{
		if(first_time_flag[2] == 0)
		{
			first_time_flag[2]++;
			Send_Byte(CornerInRoad_Flag,nodenum);
			CornerInRoad_Flag++;
			vTaskDelay(50);
		}
	}
	else if(nodenum == A61)
	{
		if(first_time_flag[3] == 0)
		{
			first_time_flag[3]++;
			Send_Byte(CornerInRoad_Flag,nodenum);
			CornerInRoad_Flag++;
			vTaskDelay(50);
		}
	}
	else if(nodenum == A14)
	{
		if(first_time_flag[4] == 0)
		{
			first_time_flag[4]++;
			Send_Byte(CornerInRoad_Flag,nodenum);
			CornerInRoad_Flag++;
			vTaskDelay(50);
		}
	}
	else if(nodenum == A16)
	{
		if(first_time_flag[5] == 0)
		{
			first_time_flag[5]++;
			Send_Byte(CornerInRoad_Flag,nodenum);
			CornerInRoad_Flag++;
			vTaskDelay(50);
		}
	}
	else if(nodenum == B4)
	{
		if(first_time_flag[6] == 0)
		{
			first_time_flag[6]++;
			Send_Byte(CornerInRoad_Flag,nodenum);
			CornerInRoad_Flag++;
			vTaskDelay(50);
		}
	}
	else if(nodenum == L5)
	{
		if(first_time_flag[7] == 0)
		{
			first_time_flag[7]++;
			Send_Byte(CornerInRoad_Flag,nodenum);
			CornerInRoad_Flag++;
			vTaskDelay(50);
		}
	}
	else if(nodenum == A20)
	{
		if(first_time_flag[8] == 0)
		{
			first_time_flag[8]++;
			Send_Byte(CornerInRoad_Flag,nodenum);
			CornerInRoad_Flag++;
			vTaskDelay(50);
		}
	}
	else if(nodenum == A21)
	{
		if(first_time_flag[9] == 0)
		{
			first_time_flag[9]++;
			Send_Byte(CornerInRoad_Flag,nodenum);
			CornerInRoad_Flag++;
			vTaskDelay(50);
		}
	}
	else if(nodenum == A23)
	{
		if(first_time_flag[10] == 0)
		{
			first_time_flag[10]++;
			Send_Byte(CornerInRoad_Flag,nodenum);
			CornerInRoad_Flag++;
			vTaskDelay(50);
		}
	}
	else if(nodenum == L8)
	{
		if(first_time_flag[11] == 0)
		{
			first_time_flag[11]++;
			Send_Byte(CornerInRoad_Flag,nodenum);
			CornerInRoad_Flag++;
			vTaskDelay(50);
		}
	}
	else if(nodenum == L9)
	{
		if(first_time_flag[12] == 0)
		{
			first_time_flag[12]++;
			Send_Byte(CornerInRoad_Flag,nodenum);
			CornerInRoad_Flag++;
			vTaskDelay(50);
		}
	}
	else if(nodenum == L10)
	{
		if(first_time_flag[13] == 0)
		{
			first_time_flag[13]++;
			Send_Byte(CornerInRoad_Flag,nodenum);
			CornerInRoad_Flag++;
			vTaskDelay(50);
		}
	}
	else if(nodenum == L11)
	{
		if(first_time_flag[14] == 0)
		{
			first_time_flag[14]++;
			Send_Byte(CornerInRoad_Flag,nodenum);
			CornerInRoad_Flag++;
			vTaskDelay(50);
		}
	}
	else if(nodenum == A38)
	{
		if(first_time_flag[15] == 0)
		{
			first_time_flag[15]++;
			Send_Byte(CornerInRoad_Flag,nodenum);
			CornerInRoad_Flag++;
			vTaskDelay(50);
		}
	}
	else if(nodenum == A40)
	{
		if(first_time_flag[16] == 0)
		{
			first_time_flag[16]++;
			Send_Byte(CornerInRoad_Flag,nodenum);
			CornerInRoad_Flag++;
			vTaskDelay(50);
		}
	}
	else if(nodenum == A41)
	{
		if(first_time_flag[17] == 0)
		{
			first_time_flag[17]++;
			Send_Byte(CornerInRoad_Flag,nodenum);
			CornerInRoad_Flag++;
			vTaskDelay(50);
		}
	}
	else if(nodenum == L14)
	{
		if(first_time_flag[18] == 0)
		{
			first_time_flag[18]++;
			Send_Byte(CornerInRoad_Flag,nodenum);
			CornerInRoad_Flag++;
			vTaskDelay(50);
		}
	}
	else if(nodenum == B13)
	{
		if(first_time_flag[19] == 0)
		{
			first_time_flag[19]++;
			Send_Byte(CornerInRoad_Flag,nodenum);
			CornerInRoad_Flag++;
			vTaskDelay(50);
		}
	}
	else if(nodenum == A45)
	{
		if(first_time_flag[20] == 0)
		{
			first_time_flag[20]++;
			Send_Byte(CornerInRoad_Flag,nodenum);
			CornerInRoad_Flag++;
			vTaskDelay(50);
		}
	}
	else if(nodenum == A47)
	{
		if(first_time_flag[21] == 0)
		{
			first_time_flag[21]++;
			Send_Byte(CornerInRoad_Flag,nodenum);
			CornerInRoad_Flag++;
			vTaskDelay(50);
		}
	}
	else if(nodenum == A62)
	{
		if(first_time_flag[22] == 0)
		{
			first_time_flag[22]++;
			Send_Byte(CornerInRoad_Flag,nodenum);
			CornerInRoad_Flag++;
			vTaskDelay(50);
		}
	}
	else if(nodenum == A52)
	{
		if(first_time_flag[23] == 0)
		{
			first_time_flag[23]++;
			Send_Byte(CornerInRoad_Flag,nodenum);
			CornerInRoad_Flag++;
			vTaskDelay(50);
		}
	}
	else if(nodenum == A53)
	{
		if(first_time_flag[24] == 0)
		{
			first_time_flag[24]++;
			Send_Byte(CornerInRoad_Flag,nodenum);
			CornerInRoad_Flag++;
			vTaskDelay(50);
		}
	}
	else if(nodenum == A57)
	{
		if(first_time_flag[25] == 0)
		{
			first_time_flag[25]++;
			Send_Byte(CornerInRoad_Flag,nodenum);
			CornerInRoad_Flag++;
			vTaskDelay(50);
		}
	}
}

