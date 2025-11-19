#include "scaner.h"
#include "map.h"
#include "math.h"
#include "turn.h"
#include "stdio.h"
#include "pid.h"
#include "speed_ctrl.h"
#include "bsp_linefollower.h"
#include "motor.h"

#define LINE_SPEED_MAX 50
#define Speed_Compensate   5
#define BLACK 0								//循黑线
#define WHLITE 1							//循白线

const float line_weight[8] = {-0.9,-0.5,-0.3,-0.1,0.1,0.3,0.5,0.9};//0.3
//从右到左
volatile struct Scaner_Set scaner_set = {0,0};

uint8_t ScanMode;
//float CatchsensorNum = 0;	//寻迹时的目标传感器位置，一般情况下为0，微微会用到
volatile SCANER Scaner;

uint32_t data_head = 0;
uint32_t data_tail = 0;

#define Line_color BLACK
//循迹板 1234578 87654321

void Go_Line(float speed){
	float Fspeed;				//经过PID运算后的结果
					
	line_pid_obj.measure = Scaner.error;  							//当前循迹板所在的位置，从左到右-7到0到0到7
	line_pid_obj.target = scaner_set.CatchsensorNum;		//目标
//	printf("error=%f\r\n",line_pid_obj.measure);
	Fspeed = positional_PID(&line_pid_obj, &line_pid_param); //进行位置PID运算
	
	if (Fspeed>=LINE_SPEED_MAX) 
		Fspeed = LINE_SPEED_MAX;
	else if (Fspeed<=-LINE_SPEED_MAX) 
		Fspeed = -LINE_SPEED_MAX;
	
	Fspeed *= fabsf(speed)/50;
//	printf("%f\r\n",Fspeed);
	
	
	motor_all.Lspeed = speed-Fspeed;
	motor_all.Rspeed = speed+Fspeed;
	
}
uint8_t getline_error()//获得更新不同巡线模式下的误差值
{
	get_detail();//获取巡线值
	if(Line_Scan(&Scaner, Lamp_Max, scaner_set.EdgeIgnore))
	{
		return 1;
	}
	else
	{
		return 0;
	}
//	printf("%d   %d   %d   %d\r\n",motor_L0.measure,motor_L1.measure,motor_R0.measure,motor_R1.measure);
}
void get_detail()
{
	//遇白亮灯返回0
		//左边的灯
	//从左往右   1 2 3 4 5 6 7 8
	if(Line_color)
	{
		data_head = 0xff;//白线
		data_tail = 0xff;
		Scaner.detail = 0xff;
		if(ScanMode == is_Front)
		{
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left1,GPIO_Left1_P)<<0);
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left2,GPIO_Left2_P)<<1);
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left3,GPIO_Left3_P)<<2);
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left4,GPIO_Left4_P)<<3);
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left5,GPIO_Left5_P)<<4);
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left6,GPIO_Left6_P)<<5);
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left7,GPIO_Left7_P)<<6);
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left8,GPIO_Left8_P)<<7);
		}
		else
		{
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right1,GPIO_Right1_P)<<7); //不同输出1.相同输出0
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right2,GPIO_Right2_P)<<6);
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right3,GPIO_Right3_P)<<5);
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right4,GPIO_Right4_P)<<4);
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right5,GPIO_Right5_P)<<3);
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right6,GPIO_Right6_P)<<2);
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right7,GPIO_Right7_P)<<1);
			Scaner.detail^=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right8,GPIO_Right8_P)<<0);
		}
	}
	else
	{
		Scaner.detail = 0x0;
		if(ScanMode == is_Front)
		{
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left1,GPIO_Left1_P)<<0);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left2,GPIO_Left2_P)<<1);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left3,GPIO_Left3_P)<<2);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left4,GPIO_Left4_P)<<3);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left5,GPIO_Left5_P)<<4);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left6,GPIO_Left6_P)<<5);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left7,GPIO_Left7_P)<<6);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Left8,GPIO_Left8_P)<<7);
		}	
		else if(ScanMode == is_Back)
		{
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right1,GPIO_Right1_P)<<7);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right2,GPIO_Right2_P)<<6);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right3,GPIO_Right3_P)<<5);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right4,GPIO_Right4_P)<<4);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right5,GPIO_Right5_P)<<3);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right6,GPIO_Right6_P)<<2);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right7,GPIO_Right7_P)<<1);
			Scaner.detail|=((uint8_t)HAL_GPIO_ReadPin(GPIO_Right8,GPIO_Right8_P)<<0);
		}
	}
	
//	if(ScanMode == is_Front)
//		Scaner.detail = data_head;
//	else if(ScanMode == is_Back)
//		Scaner.detail = data_tail;
}
//循迹扫描
uint8_t Line_Scan(SCANER *scaner, unsigned char sensorNum, int8_t edge_ignore)
{
	float error = 0;
	u8 linenum=0;//记录线的数目
	u8 lednum=0;
	int8_t lednum_tmp = 0;
							//获得二进制巡线值
	for(uint8_t i=0;i<sensorNum;i++) 		//从小车方向从左往右数亮灯数和引导线数
	{								//linenum用来记录有多少条线，line用来记录第几条线。
		if((scaner->detail&(0x1<<i))) 
		{
			lednum++;
			if(!(scaner->detail&(1<<(i+1)))) 
				++linenum;			//先读取亮灯数和引导线数，检测到从1变为0认为一条线
		}
	}
	scaner->lineNum = linenum;			
	scaner->ledNum = lednum;
////		for (uint8_t i=0; i<sensorNum; i++)
////		{
////			lednum_tmp += (scaner->detail>>(sensorNum-i-1))&0X01;//记录点灯数
////			error += ((scaner->detail>>(sensorNum-i-1))&0X01) * line_weight[i];
////		}
//	if ((nodesr.nowNode.flag & LEFT_LINE) == LEFT_LINE)  //左循线    
//	{
//		for (uint8_t i=0; i<sensorNum; i++)
//		{
//			lednum_tmp += (scaner->detail>>(sensorNum-i-1))&0X01;//记录点灯数
//			error += ((scaner->detail>>(sensorNum-i-1))&0X01) * line_weight[i];
//			if ((scaner->detail>>(sensorNum-i-1)) & 0X01)		//如果是白线
//				if (!((scaner->detail>>((sensorNum-i-2)))&0x01))		//下一个灯不是白
//					break;		//退出
//		}
//	}
//	else if ((nodesr.nowNode.flag & RIGHT_LINE) == RIGHT_LINE)  //右循线
//	{
//		for (uint8_t i=0; i<sensorNum; i++)
//		{
//			lednum_tmp += (scaner->detail>>i)&0X01;
//			error += ((scaner->detail>>i)&0X01) * line_weight[sensorNum-i-1];
//			if ((scaner->detail>>i) & 0X01)
//				if (!((scaner->detail>>(i+1))&0x01))
//					break;
//		}
//	}
//	else if((nodesr.nowNode.flag&LiuShui)==LiuShui)
//	{
//		uint8_t flag=0;
//		float error1=0;
//		float error2=0;
//		uint8_t lednum_temp1=0;
//		uint8_t lednum_temp2=0;
//		for(uint8_t i=0;i<sensorNum;i++)
//		{
//			if(flag==0)//右边线
//			{
//				error1+= ((scaner->detail>>i)&0X01) * line_weight[sensorNum-i-1];
//				lednum_temp1+= (scaner->detail>>i)&0X01;
//			}
//			else if(flag==1)//左边线
//			{
//				error2+= ((scaner->detail>>i)&0X01) * line_weight[sensorNum-i-1];
//				lednum_temp2+= (scaner->detail>>i)&0X01;
//			}
//			if ((scaner->detail>>i) & 0X01)
//			{
//				if (!((scaner->detail>>(i+1))&0x01))
//				{
//					flag=1;
//				}
//			}
//		}
//		if(error1!=0)
//			error1/=lednum_temp1;
//		else
//			error1=0;
//		if(lednum_temp2==0)//只有一条线
//			error2=0;
//		else
//			error2/=lednum_temp2;
//		if(lednum>=6)
//		{
//			Scaner.error = 0;
//			return 0;
//		}
//		else
//		{
//			if(fabs(error2)>fabs(error1)|lednum_temp2==0)
//			{
//				Scaner.error = error1;
//				return 0;
//			}
//			else if((lednum_temp2!=0)&(fabs(error2)<fabs(error1)))
//			{
//				Scaner.error = error2;
//				return 0;
//			}
//		}
//		
//	}
//	else//后面再完善
//	{
		for(uint8_t i= edge_ignore; i<sensorNum - edge_ignore; i++) 
		{
				lednum_tmp += (scaner->detail>>(sensorNum-1-i))&0X01;
				error += ((scaner->detail>>(sensorNum-1-i))&0X01) * line_weight[i];
		}
		if(lednum_tmp>=3)
		{
			Scaner.error=0;
			return 0;
		}       		
//	}
//	if(lednum==0|lednum_tmp==0)
//	{
//		error=0;
//	}
//	else
//	{
//		error/=(float)lednum_tmp;		//取平均
//	}
	Scaner.error = error;
	return 0;
}

