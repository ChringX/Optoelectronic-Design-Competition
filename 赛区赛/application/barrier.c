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
uint8_t data[100];
uint8_t Be_Detected_Flag = 0;

void zhunbei(void)
{	
	
	uint8_t _Flag = 0; //临时变量
	uint8_t Send_Flag = 0;
	CarBrake();
	Start_Color = HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_8);
	uint8_t data2Point_Flag = 0;

//	for(Send_Flag = 0;Send_Flag < 8;Send_Flag++)
//	{
//		Send_Byte(Send_Flag,A63);
//	}
//	Send_Byte(Send_Flag,0);
//	Send_Flag++;
//	Send_Byte(Send_Flag,A63);
//	for(Send_Flag = 10;Send_Flag < 14;Send_Flag++)
//	{
//		Send_Byte(Send_Flag,0);
//	}
//	while(1);

	//0-7八个宝藏点，8已去区域，9是否到达终点，10-13四个拐角
	do
	{
		Read_Byte(_Flag,&data[_Flag]);
		vTaskDelay(50);
	//	printf("%d\r\n",data[_Flag]);
		_Flag++;
	}while(_Flag<14);  //从eeprom中读取数据
	
	
	if(Confirm_FirstTime() == 1 || Confirm_FirstTime() == 0) //第一次启动
	{
		while((TreaPoint[0] == 0)&&(TreaPoint[1] == 0)&&(TreaPoint[2] == 0)&&
		  (TreaPoint[3] == 0)&&(TreaPoint[4] == 0)&&(TreaPoint[5] == 0)&&
		  (TreaPoint[6] == 0)&&(TreaPoint[7] == 0))
		{
			buzzer_off();
			vTaskDelay(2);
		}
		buzzer_on();
		
		for(Send_Flag = 0;Send_Flag < 8;Send_Flag++)
		{
			Send_Byte(Send_Flag,TreaPoint[Send_Flag]);
			vTaskDelay(50);
		}
		Send_Byte(Send_Flag,0); //Be_Detected_Flag
		Send_Flag++;
		if(Start_Color == 1)
		{
			Send_Byte(Send_Flag,P1);
		}
		else
		{
			Send_Byte(Send_Flag,P2);
		}
	}
	else if(Confirm_FirstTime() == 3)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		vTaskDelay(1000);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		
		for(uint8_t iii = 0;iii<8;iii++)
		{
			while(data[iii] == A63 || data[iii] == 0)
			{
				Read_Byte(iii,&data[iii]);
				vTaskDelay(2);
			}
			
		}
//		printf("%d\r\n",data[1]);
		for(data2Point_Flag = 0;data2Point_Flag < 8;data2Point_Flag++)
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

//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
//	while(1)
//	{
//		vTaskDelay(2);
//	}
//	runWithAngle(nodesr.nowNode.angle,4500);
//	float num = 0;
//	num = motor_all.Distance;
//	while(motor_all.Distance - num < 5)
//	{
//		vTaskDelay(2);
//	}
//	encoder_clear();
	imu.compensateZ=need2turn(imu.yaw,nodesr.nowNode.angle);
	pid_mode_switch(is_Line); 
	motor_all.Cspeed = nodesr.nowNode.speed;
}

//撞宝藏动作
void BreakTrea(void)
{
	float num;
	motor_all.Cspeed=nodesr.nowNode.speed;
	pid_mode_switch(is_Line);
	getline_error();
	while((Scaner.ledNum != 0) || (Scaner.detail != 0))
	{
		getline_error();
		vTaskDelay(2);
	}
	angle.AngleG = getAngleZ();
	motor_all.Gspeed = 5000;
	pid_mode_switch(is_Gyro);
	if(ScanMode == is_Front)
	{
		num = motor_all.Distance;
		while(fabsf(num - motor_all.Distance) < 10)
		{
			vTaskDelay(2);
		}
	}
	else if(ScanMode == is_Back)
	{
		num = motor_all.Distance;
		while(fabsf(num - motor_all.Distance) < 10)
		{
			vTaskDelay(2);
		}
	}
	buzzer_on();
	encoder_clear();
	CarBrake();
	vTaskDelay(500);
	Be_Detected_Flag++;  //找到三个宝藏
}

void Change_ScanMode(void)
{
	if(ScanMode == is_Back)
	{
		ScanMode = is_Front;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	}
	else
	{
		ScanMode = is_Back;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	}
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
	while(1)
	{
		route[temp] = PhEleTechnology.route[k];
		temp++;
		k++;
		if(PhEleTechnology.route[k] == 255)
		{
			route[temp] = PhEleTechnology.route[k];
			
			Change_ScanMode();
			
			if(nodesr.lastNode.nodenum == B13 || nodesr.lastNode.nodenum == B4
				 || nodesr.lastNode.nodenum == A62 || nodesr.lastNode.nodenum == A61)
			{
				motor_all.Cspeed = 2000;
			}
			else
			{
				motor_all.Cspeed = nodesr.nowNode.speed;
			}
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
		nodesr.lastNode.angle = -90;
	else if(nodesr.lastNode.angle == 360)
		nodesr.lastNode.angle = 0;
	
	imu.compensateZ=need2turn(imu.yaw,nodesr.lastNode.angle);
	
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
				angle.AngleT = nodesr.nowNode.angle;  //转绝对角度
				pid_mode_switch(is_Turn);	
				while(fabsf(nodesr.nowNode.angle-getAngleZ()) > 5)
				{
					break_time++;
					getline_error();
					if(Scaner.lineNum==1&&(((Scaner.detail&0x30) == 0x30) || ((Scaner.detail&0x18) == 0x18) || ((Scaner.detail&0x0c) == 0x0c))/*&&break_time>=50*/&&(fabsf(need2turn(angle.AngleT,getAngleZ()))<fabsf(need2turn(angle.AngleT,nodesr.lastNode.angle))*0.3f))
					{
						break;
					}
					vTaskDelay(2);
					if(break_time >= 2500)
					{
						break;
					}
				}
				break_time = 0;
				getline_error();
				if(Scaner.ledNum == 0)
				{
					float Angle11;
					if(nodesr.lastNode.angle - nodesr.nowNode.angle == -90 || nodesr.lastNode.angle - nodesr.nowNode.angle == 270)
						angle.AngleT = Angle11 = getAngleZ()+30;
					else if(nodesr.lastNode.angle - nodesr.nowNode.angle == -270 || nodesr.lastNode.angle - nodesr.nowNode.angle == 90)
						angle.AngleT = Angle11 = getAngleZ()-30;
					
					pid_mode_switch(is_Turn);	
					while(fabsf(Angle11-getAngleZ()) > 5)
					{
						break_time++;
						getline_error();
						if(Scaner.lineNum==1&&(((Scaner.detail&0x30) == 0x30) || ((Scaner.detail&0x18) == 0x18) || ((Scaner.detail&0x0c) == 0x0c))/*&&break_time>=50*/&&(fabsf(need2turn(angle.AngleT,getAngleZ()))<fabsf(need2turn(angle.AngleT,nodesr.lastNode.angle))*0.3f))
						{
							break;
						}
						vTaskDelay(2);
						if(break_time >= 2500)
						{
							break;
						}
					}	
					break_time = 0;
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
	motor_all.Cspeed=4500;
	pid_mode_switch(is_Line);
	
	scaner_set.EdgeIgnore = 0;
		
//	CarBrake();
	
	if((nodesr.nowNode.flag & RealTrea) == RealTrea)
	{
		uint8_t RealTrea_Flag;
		
		if(Start_Color == 1)
			now_color = 1;
		else
			now_color = 2;
		
		BreakTrea();
		for(RealTrea_Flag = 0;RealTrea_Flag < 8;RealTrea_Flag++)
		{
			if(TreaPoint[RealTrea_Flag] == nodesr.nowNode.nodenum)
			{
				TreaPoint[RealTrea_Flag] = A63;
			}
		}
	}
	else
	{
		if(!Long_Trea()) //短宝藏
		{
			if((nodesr.nowNode.nodenum == A61 && route[map.point-4] == A23)|| (nodesr.nowNode.nodenum == A62 && route[map.point-4] == A38)
			|| (nodesr.nowNode.nodenum == B4 && route[map.point-4] == A4) || (nodesr.nowNode.nodenum == B13 && route[map.point-4] == A57))
			{
				num = motor_all.Distance;
				while(fabsf(num - motor_all.Distance) < 8)
				{
					vTaskDelay(2);
				}
			}
		}
		else
		{
			num = motor_all.Distance;
			while(fabsf(num - motor_all.Distance) < 25)
			{
				vTaskDelay(2);
			}
		}
		
		CarBrake();
		
		if(ScanMode == is_Front)
			open_mvR();
		else
			open_mv();
		
		vTaskDelay(1000);
		
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
					int break_time = 0;
					angle.AngleT = nodesr.nowNode.angle;
					pid_mode_switch(is_SmallTurn);
					while(fabsf(angle.AngleT - getAngleZ()) > 5)
					{
						turn_time++;
						vTaskDelay(2);
						getline_error();
						if(Scaner.lineNum==1&&(((Scaner.detail&0x30) == 0x30) || ((Scaner.detail&0x18) == 0x18) || ((Scaner.detail&0x0c) == 0x0c))&&break_time>=50/*&&(fabsf(need2turn(angle.AngleT,getAngleZ()))<fabsf(need2turn(angle.AngleT,nodesr.nowNode.angle))*0.3f)*/)
							break;
						if(turn_time >= 500)
							break;
					}
					turn_time = 0;
					time = 0;
					TreaPoint[j] = A63;
					break;
				}
			}
			if(color == 1 ||color == 3)//己方宝藏或者是己方伪宝藏
			{
				if(color == 1)
				{//撞宝藏动作
					now_color = 1;
					BreakTrea();
				}
				
				WrongSymmetry();
				
				for(k = 0;k < 4;k++)     
				{
					for(m = 0;m < 2;m++)
					{
						if(nodesr.nowNode.nodenum == Area_Trea[k][m])  //找到真宝藏对应的区域
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
				for(uint8_t i = 0;i < 8;i++)   //不去找真宝藏区域的其他宝藏
				{
					if(TreaPoint[i] == Area_Trea[k][0] || TreaPoint[i] == Area_Trea[k][1])
					{
						TreaPoint[i] = A63;
					}
				}
			}
			else if(color == 2 || color == 4) //发现的是假宝藏
			{
				TreaPoint[j] = A63;
				
				if(color == 2)
				{
					ComfirmSymmetry();
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
						}
					}
				}
				else if(color == 4)
					WrongSymmetry();
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
					int break_time = 0;
					angle.AngleT = nodesr.nowNode.angle;
					pid_mode_switch(is_SmallTurn);
					while(fabsf(angle.AngleT - getAngleZ()) > 5)
					{
						turn_time++;
						vTaskDelay(2);
						getline_error();
						if(Scaner.lineNum==1&&(((Scaner.detail&0x30) == 0x30) || ((Scaner.detail&0x18) == 0x18) || ((Scaner.detail&0x0c) == 0x0c))&&break_time>=50/*&&(fabsf(need2turn(angle.AngleT,getAngleZ()))<fabsf(need2turn(angle.AngleT,nodesr.nowNode.angle))*0.3f)*/)
							break;
						if(turn_time >= 500)
							break;
					}
					turn_time = 0;
					time = 0;
					TreaPoint[j] = A63;
					break;
				}
			}
			if(color == 2 ||color == 4)//己方宝藏或者是假宝藏
			{
				if(color == 2)
				{//撞宝藏动作
					now_color = 2;
					BreakTrea();
				}
				
				WrongSymmetry();
				
				for(k = 0;k < 4;k++)     
				{
					for(m = 0;m < 2;m++)
					{
						if(nodesr.nowNode.nodenum == Area_Trea[k][m])  //找到真宝藏对应的区域
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
				for(uint8_t i = 0;i < 8;i++)   //不去找真宝藏区域的其他宝藏
				{
					if(TreaPoint[i] == Area_Trea[k][0] || TreaPoint[i] == Area_Trea[k][1])
					{
						TreaPoint[i] = A63;
					}
				}
			}
			else if(color == 1 || color == 3) //发现的是假宝藏
			{
				TreaPoint[j] = A63;
				
				if(color == 1)
				{
					ComfirmSymmetry();
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
						}
					}
				}
				else if(color == 3)
					WrongSymmetry();
			}
		}
		close_mv();
		close_mvR();
	}
	
	
	buzzer_off();
	scaner_set.CatchsensorNum = 0;
	
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
}

void PLATFORM(void)
{
	float num = 0;
	num = motor_all.Distance;
	while(fabsf(motor_all.Distance - num) < 40)
	{
		vTaskDelay(2);
	}
	Send_Byte(9,A63);
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

void Confirm_TreaPoint(void)
{
	uint8_t i,k;
	uint8_t m = 0;
	uint8_t n = 0;
	uint8_t q = 0;
	uint8_t p = 0;
	for(i = 0;i < 8;i++)
	{
		if(TreaPoint[i] == 8)     //区域一
		{
			Area_Trea[0][m] = 8;
			m++;
		}
		if(TreaPoint[i] == 16)
		{
			Area_Trea[0][m] = 16;
			m++;
		}
		if(TreaPoint[i] == 20)
		{
			Area_Trea[0][m] = 20;
			m++;
			Send_Byte(11,1);
		}
		if(TreaPoint[i] == 33)
		{
			Area_Trea[0][m] = 33;
			m++;
		}
	
		
		if(TreaPoint[i] == 13)   //区域二
		{
			Area_Trea[1][n] = 13;
			n++;
			Send_Byte(10,1);
		}
		if(TreaPoint[i] == 22)    
		{
			Area_Trea[1][n] = 22;
			n++;
		}
		if(TreaPoint[i] == 28)
		{
			Area_Trea[1][n] = 28;
			n++;
		}
		if(TreaPoint[i] == 32)
		{
			Area_Trea[1][n] = 32;
			n++;
		}
		
		
		
		
		if(TreaPoint[i] == 45)    //区域三
		{
			Area_Trea[2][q] = 45;
			q++;
		}
		if(TreaPoint[i] == 49)
		{
			Area_Trea[2][q] = 49;
			q++;
		}
		if(TreaPoint[i] == 55)
		{
			Area_Trea[2][q] = 55;
			q++;
		}
		if(TreaPoint[i] == 64)
		{
			Area_Trea[2][q] = 64;
			q++;
			Send_Byte(13,1);
		}
		
		
		if(TreaPoint[i] == 44)    //区域四
		{
			Area_Trea[3][p] = 44;
			p++;
		}
		if(TreaPoint[i] == 57)
		{
			Area_Trea[3][p] = 57;
			p++;
			Send_Byte(12,1);
		}
		if(TreaPoint[i] == 61)
		{
			Area_Trea[3][p] = 61;
			p++;
		}
		if(TreaPoint[i] == 69)
		{
			Area_Trea[3][p] = 69;
			p++;
		}
		
		for(k = 0;k < 168;k++)
		{
			if(TreaPoint[i] == Node[k].nodenum)
			{
				Node[k].function = Treasure;
				
				if(Node[k].nodenum == A61)
				{
					Node[28].step = 1000;
					Node[27].step = 1000;
				}
				else if(Node[k].nodenum == B4)
				{
					Node[44].step = 1000;
					Node[46].step = 1000;
				}
				else if(Node[k].nodenum == B13)
				{
					Node[120].step = 1000;
					Node[123].step = 1000;
				}
				else if(Node[k].nodenum == A62)
				{
					Node[139].step = 1000;
					Node[140].step = 1000;
				}
			}
		}
	}
	
	if(data[10] == 1)
	{
		Node[28].step = 1000;
		Node[27].step = 1000;
	}
	if(data[11] == 1)
	{
		Node[44].step = 1000;
		Node[46].step = 1000;
	}
	if(data[12] == 1)
	{
		Node[120].step = 1000;
		Node[123].step = 1000;
	}
	if(data[13] == 1)
	{
		Node[139].step = 1000;
		Node[140].step = 1000;
	}
	
	CreatGraph(&PhEleTechnology_Test);
	CreatGraph(&PhEleTechnology);
}

void Point2data(void)
{
	uint8_t flag = 0;
	for(flag = 0;flag < 8;flag++)
	{
		if(nodesr.nowNode.nodenum == TreaPoint[flag])
		{
			Send_Byte(flag,A63);
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
	else if((((data[0] == A63) && (data[1] == A63) && (data[2] == A63) && (data[3] == A63) 
	&& (data[4] == A63) && (data[5] == A63) && (data[6] == A63) && (data[7] == A63) ) || data[8] == 3)&& (data[9] != A63))
	{
		return 2;
	}
	else if(data[9] == A63)
	{
		return 0;
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

//对称点
void ComfirmSymmetry(void)
{
	if(nodesr.nowNode.nodenum == B10)
		Node[81].flag |= RealTrea;
	else if(nodesr.nowNode.nodenum == B7)
		Node[86].flag |= RealTrea;
	else if(nodesr.nowNode.nodenum == B3)
		Node[166].flag |= RealTrea;
	else if(nodesr.nowNode.nodenum == B14)
		Node[1].flag |= RealTrea;
	else if(nodesr.nowNode.nodenum == B1)
		Node[163].flag |= RealTrea;
	else if(nodesr.nowNode.nodenum == B16)
		Node[3].flag |= RealTrea;
	else if(nodesr.nowNode.nodenum == B8)
		Node[96].flag |= RealTrea;
	else if(nodesr.nowNode.nodenum == B9)
		Node[71].flag |= RealTrea;
	else if(nodesr.nowNode.nodenum == B6)
		Node[110].flag |= RealTrea;
	else if(nodesr.nowNode.nodenum == B11)
		Node[58].flag |= RealTrea;
	else if(nodesr.nowNode.nodenum == B4)
		Node[143].flag |= RealTrea;
	else if(nodesr.nowNode.nodenum == B13)
		Node[24].flag |= RealTrea;
	else if(nodesr.nowNode.nodenum == A61)
		Node[140].flag |= RealTrea;
	else if(nodesr.nowNode.nodenum == A62)
		Node[27].flag |= RealTrea;
}

void WrongSymmetry(void)
{
	uint8_t _flag_;
	if(nodesr.nowNode.nodenum == B1)
	{
		for(_flag_ = 0;_flag_ < 8;_flag_++)
		{
			if(TreaPoint[_flag_] == B16)
			{
				TreaPoint[_flag_] = A63;
			}
		}
	}
	else if(nodesr.nowNode.nodenum == B3)
	{
		for(_flag_ = 0;_flag_ < 8;_flag_++)
		{
			if(TreaPoint[_flag_] == B14)
			{
				TreaPoint[_flag_] = A63;
			}
		}
	}
	else if(nodesr.nowNode.nodenum == B4)
	{
		for(_flag_ = 0;_flag_ < 8;_flag_++)
		{
			if(TreaPoint[_flag_] == B13)
			{
				TreaPoint[_flag_] = A63;
			}
		}
	}
	else if(nodesr.nowNode.nodenum == B5)
	{
		for(_flag_ = 0;_flag_ < 8;_flag_++)
		{
			if(TreaPoint[_flag_] == B12)
			{
				TreaPoint[_flag_] = A63;
			}
		}
	}
	else if(nodesr.nowNode.nodenum == B6)
	{
		for(_flag_ = 0;_flag_ < 8;_flag_++)
		{
			if(TreaPoint[_flag_] == B11)
			{
				TreaPoint[_flag_] = A63;
			}
		}
	}
	else if(nodesr.nowNode.nodenum == B7)
	{
		for(_flag_ = 0;_flag_ < 8;_flag_++)
		{
			if(TreaPoint[_flag_] == B10)
			{
				TreaPoint[_flag_] = A63;
			}
		}
	}
	else if(nodesr.nowNode.nodenum == B8)
	{
		for(_flag_ = 0;_flag_ < 8;_flag_++)
		{
			if(TreaPoint[_flag_] == B9)
			{
				TreaPoint[_flag_] = A63;
			}
		}
	}
	else if(nodesr.nowNode.nodenum == B9)
	{
		for(_flag_ = 0;_flag_ < 8;_flag_++)
		{
			if(TreaPoint[_flag_] == B8)
			{
				TreaPoint[_flag_] = A63;
			}
		}
	}
	else if(nodesr.nowNode.nodenum == B10)
	{
		for(_flag_ = 0;_flag_ < 8;_flag_++)
		{
			if(TreaPoint[_flag_] == B7)
			{
				TreaPoint[_flag_] = A63;
			}
		}
	}
	else if(nodesr.nowNode.nodenum == B11)
	{
		for(_flag_ = 0;_flag_ < 8;_flag_++)
		{
			if(TreaPoint[_flag_] == B6)
			{
				TreaPoint[_flag_] = A63;
			}
		}
	}
	else if(nodesr.nowNode.nodenum == B12)
	{
		for(_flag_ = 0;_flag_ < 8;_flag_++)
		{
			if(TreaPoint[_flag_] == B5)
			{
				TreaPoint[_flag_] = A63;
			}
		}
	}
	else if(nodesr.nowNode.nodenum == B13)
	{
		for(_flag_ = 0;_flag_ < 8;_flag_++)
		{
			if(TreaPoint[_flag_] == B4)
			{
				TreaPoint[_flag_] = A63;
			}
		}
	}
	else if(nodesr.nowNode.nodenum == B14)
	{
		for(_flag_ = 0;_flag_ < 8;_flag_++)
		{
			if(TreaPoint[_flag_] == B3)
			{
				TreaPoint[_flag_] = A63;
			}
		}
	}
	else if(nodesr.nowNode.nodenum == A62)
	{
		for(_flag_ = 0;_flag_ < 8;_flag_++)
		{
			if(TreaPoint[_flag_] == A61)
			{
				TreaPoint[_flag_] = A63;
			}
		}
	}
	else if(nodesr.nowNode.nodenum == B16)
	{
		for(_flag_ = 0;_flag_ < 8;_flag_++)
		{
			if(TreaPoint[_flag_] == B1)
			{
				TreaPoint[_flag_] = A63;
			}
		}
	}
	else if(nodesr.nowNode.nodenum == A61)
	{
		for(_flag_ = 0;_flag_ < 8;_flag_++)
		{
			if(TreaPoint[_flag_] == A62)
			{
				TreaPoint[_flag_] = A63;
			}
		}
	}
}

uint8_t SymPoint(uint8_t nodenum)
{
	if(nodenum == B3)
		return B14;
	else if(nodenum == B1)
		return B16;
	else if(nodenum == A61)
		return A62;
	else if(nodenum == B4)
		return B13;
	else if(nodenum == B5)
		return B12;
	else if(nodenum == B6)
		return B11;
	else if(nodenum == B7)
		return B10;
	else if(nodenum == B8)
		return B9;
	else if(nodenum == B9)
		return B8;
	else if(nodenum == B10)
		return B7;
	else if(nodenum == B11)
		return B6;
	else if(nodenum == B12)
		return B5;
	else if(nodenum == B13)
		return B4;
	else if(nodenum == B14)
		return B3;
	else if(nodenum == A62)
		return A61;
	else if(nodenum == B16)
		return B1;
}

