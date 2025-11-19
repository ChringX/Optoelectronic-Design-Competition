#include "encoder.h"
#include  <stdlib.h>
#include <string.h>
#include "uart.h"
#include "speed_ctrl.h"
wheel wheel_1,wheel_2,wheel_3,wheel_4;
uint32_t capture_Buf_before[4] = {0},capture_Buf_latter[4] = {0};   //存放计数时间
uint8_t  capture_Cnt[4] = {0};    			//状态标志位（用于切换模式）
uint32_t high_time[4] = {0};        			//高电平时间
uint32_t real_time[4] = {0};    					//CCR真实计时
int  direction[4] = {0,0,0,0};                //正反转识别
uint8_t  level_before[4]={0},level_latter[4] = {0};         //毛刺判断
int  pulse_num[4] = {0};                  //脉冲个数
int  pulse_out[4] = {0};                     //记录溢出个数
uint8_t frist_flag[4]={0};								//第一次收集八个数据的标志
uint8_t temp_i[4]={0};
uint32_t temp_time[4][8] = {0};        			//高电平暂存时间

uint8_t dog[4] = {0};   //电机狗

//定义小车的轮子输入捕获属于哪个定时器
void Encoder_init(void){
	//左前
	wheel_1.TIM = htim3;  //A相定时器
	wheel_1.GPIO_2 = GPIOB;        //通道2--B相
	wheel_1.GPIO_PORT_2 = GPIO_PIN_8;
	wheel_1.channelA=TIM_CHANNEL_4;//A相通道
	HAL_TIM_IC_Start_IT(&(wheel_1.TIM),wheel_1.channelA);	  //启动输入捕获
	__HAL_TIM_SET_CAPTUREPOLARITY(&wheel_1.TIM,wheel_1.channelA, TIM_INPUTCHANNELPOLARITY_RISING);  //开启上升沿捕获
		//左后
	wheel_2.TIM = htim3;
	wheel_2.GPIO_2 = GPIOB;        //通道2--B相
	wheel_2.GPIO_PORT_2 = GPIO_PIN_9;
	wheel_2.channelA=TIM_CHANNEL_3;//A相通道
	HAL_TIM_IC_Start_IT(&(wheel_2.TIM), wheel_2.channelA);	  //启动输入捕获
	__HAL_TIM_SET_CAPTUREPOLARITY(&wheel_2.TIM, wheel_2.channelA, TIM_INPUTCHANNELPOLARITY_RISING);  //开启上升沿捕获
	//右前
	wheel_3.TIM = htim3;
	wheel_3.GPIO_2 = GPIOE;        //通道2--B相
	wheel_3.GPIO_PORT_2 = GPIO_PIN_1;
	wheel_3.channelA=TIM_CHANNEL_1;//A相通道
	HAL_TIM_IC_Start_IT(&(wheel_3.TIM), wheel_3.channelA);	  //启动输入捕获
	__HAL_TIM_SET_CAPTUREPOLARITY(&wheel_3.TIM, wheel_3.channelA, TIM_INPUTCHANNELPOLARITY_RISING);  //开启上升沿捕获
	//右后
	wheel_4.TIM = htim3;
	wheel_4.GPIO_2 = GPIOE;        //通道1--B相
	wheel_4.GPIO_PORT_2 = GPIO_PIN_0;
	wheel_4.channelA=TIM_CHANNEL_2;//A相通道
	HAL_TIM_IC_Start_IT(&(wheel_4.TIM), wheel_4.channelA);	  //启动输入捕获
	__HAL_TIM_SET_CAPTUREPOLARITY(&wheel_4.TIM, wheel_4.channelA, TIM_INPUTCHANNELPOLARITY_RISING);  //开启上升沿捕获
	
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	//左上轮
	if( htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4 )
	{
		dog[0]=0;
		switch(capture_Cnt[0]){
			
			case 0:   //模式1   --------   捕获到上升沿 开始计时
				capture_Buf_before[0] = (wheel_1.TIM.Instance) ->CCR4;//获取当前的捕获值
				__HAL_TIM_SET_CAPTUREPOLARITY(&wheel_1.TIM,wheel_1.channelA,TIM_ICPOLARITY_FALLING);  //设置为下降沿捕获
				capture_Cnt[0]++; //从模式一切换到模式二
			  if(( level_before[0]= HAL_GPIO_ReadPin(wheel_1.GPIO_2,wheel_1.GPIO_PORT_2) )==RESET){   //判断正反转	           
					           direction[0] = BACKWARD;   //反转
				 // printf("反转!\r\n");
				}
				else{direction[0] = FORWARD;   //正转
				//	printf("ZHENG----------------------------------------转!\r\n");
				}
				break;
			
			  case 1:   //  模式二  --------   捕获到下降沿 关闭计时 
				capture_Buf_latter[0] = (wheel_1.TIM.Instance) ->CCR4;//获取当前的捕获值
				HAL_TIM_IC_Stop_IT(&wheel_1.TIM,wheel_1.channelA); //停止捕获  或者: __HAL_TIM_DISABLE(&htim5);
			  level_latter[0]=HAL_GPIO_ReadPin(wheel_1.GPIO_2,wheel_1.GPIO_PORT_2);//再次读取B相
			  if(level_before[0] != level_latter[0]){   //判断是否是毛刺
					if(capture_Buf_latter[0]<capture_Buf_before[0]){       //溢出计算
				    real_time[0] = (65535-capture_Buf_before[0])+capture_Buf_latter[0] +1;
			      }
					else{
						real_time[0] =  capture_Buf_latter[0]- capture_Buf_before[0] ;  //没有溢出
			      }
					if(!frist_flag[0]){    //前八次要先收集起来
						temp_time[0][temp_i[0]] = real_time[0];
						temp_i[0]++;
						if(temp_i[0]==8){
							for(uint8_t i =0; i<8;i++){
								real_time[0]+= temp_time[0][i];
							}
							high_time[0] = real_time[0]/8;
							frist_flag[0]=1;
						}
					}
					else{
						high_time[0] = 0;
						memmove(&temp_time[0], &temp_time[0][1], sizeof(uint32_t) * 7);
						temp_time[0][7] = real_time[0];
						for(uint8_t i =0; i<8;i++){
								real_time[0]+= temp_time[0][i];	
							}
						high_time[0] = real_time[0]/8;
					}
					pulse_num[0]+= direction[0];     //计算脉冲个数用于计算位移
							if(pulse_num[0]>=MAX_pulse){
						pulse_out[0]++;
						pulse_num[0]=0;
					}
					else if(pulse_num[0]<=-MAX_pulse){
						pulse_out[0]--;
						pulse_num[0]=0;
					}
				}
			  //printf("high_time = %5d us   direction = %d\r\n",high_time[0],direction[0]);     //发高电平时间
			  capture_Cnt[0] = 0;  //清空标志
					HAL_TIM_IC_Start_IT(&(wheel_1.TIM), wheel_1.channelA);	  //启动输入捕获
				__HAL_TIM_SET_CAPTUREPOLARITY(&wheel_1.TIM, wheel_1.channelA, TIM_INPUTCHANNELPOLARITY_RISING);  //开启上升沿捕获
			  break; 
		 }
	}
	
	//左后
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	{
		dog[1]=0;
		switch(capture_Cnt[1]){
			
			case 0:   //模式1   --------   捕获到上升沿 开始计时
				capture_Buf_before[1] = (wheel_2.TIM.Instance) ->CCR3;//获取当前的捕获值
				__HAL_TIM_SET_CAPTUREPOLARITY(&wheel_2.TIM,wheel_2.channelA,TIM_ICPOLARITY_FALLING);  //设置为下降沿捕获
				capture_Cnt[1]++; //从模式一切换到模式二
			  if(( level_before[1]=HAL_GPIO_ReadPin(wheel_2.GPIO_2,wheel_2.GPIO_PORT_2) )==RESET){   //判断正反转	           
					direction[1] = BACKWARD;   //反转
				//   printf("反转!\r\n");
				}
				else{direction[1] = FORWARD;   //正转
				//printf("-------------------------------------------------\r\n");
				}
				break;
			
			  case 1:   //  模式二  --------   捕获到下降沿 关闭计时 
				capture_Buf_latter[1] = (wheel_2.TIM.Instance) ->CCR3;//获取当前的捕获值
				HAL_TIM_IC_Stop_IT(&wheel_2.TIM,wheel_2.channelA); //停止捕获  或者: __HAL_TIM_DISABLE(&htim5);
			  level_latter[1]=HAL_GPIO_ReadPin(wheel_2.GPIO_2,wheel_2.GPIO_PORT_2);//再次读取B相
			  if(level_before[1] != level_latter[1]){   //判断是否是毛刺
					if(capture_Buf_latter[1]<capture_Buf_before[1]){       //溢出计算
				    real_time[1] = (65535-capture_Buf_before[1])+capture_Buf_latter[1] + 1;
			      }
					else{
						real_time[1] =  capture_Buf_latter[1]- capture_Buf_before[1] ;  //没有溢出
			      }
					if(!frist_flag[1]){    //前八次要先收集起来
						temp_time[1][temp_i[1]] = real_time[1];
						temp_i[1]++;
						if(temp_i[1]==8){
							for(uint8_t i =0; i<8;i++){
								real_time[1]+= temp_time[1][i];
							}
							high_time[1] = real_time[1]/8;
							frist_flag[1]=1;
						}
					}
					else{
						high_time[1] = 0;
						memmove(&temp_time[1], &temp_time[1][1], sizeof(uint32_t) * 7);
						temp_time[1][7] = real_time[1];
						for(uint8_t i =0; i<8;i++){
								real_time[1]+= temp_time[1][i];	
							}
						high_time[1] = real_time[1]/8;
					}
					pulse_num[1]+= direction[1] ;     //计算脉冲个数用于计算位移
							if(pulse_num[1]>=MAX_pulse){
						pulse_out[1]++;
						pulse_num[1]=0;
					}
					else if(pulse_num[1]<=-MAX_pulse){
						pulse_out[1]--;
						pulse_num[1]=0;
					}
				}
			  //printf("high_time = %5d us   direction = %d\r\n",high_time[0],direction[0]);     //发高电平时间
			  capture_Cnt[1] = 0;  //清空标志
					HAL_TIM_IC_Start_IT(&(wheel_2.TIM), wheel_2.channelA);	  //启动输入捕获
				__HAL_TIM_SET_CAPTUREPOLARITY(&wheel_2.TIM, wheel_2.channelA, TIM_INPUTCHANNELPOLARITY_RISING);  //开启上升沿捕获
			  break; 
		 }
	}
	//printf("%d\r\n",(int)(TIM3->SR));
	//右前
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		dog[2]=0;
		switch(capture_Cnt[2]){
			
			case 0:   //模式1   --------   捕获到上升沿 开始计时
				capture_Buf_before[2] = (wheel_3.TIM.Instance) ->CCR1;//获取当前的捕获值
				__HAL_TIM_SET_CAPTUREPOLARITY(&wheel_3.TIM,wheel_3.channelA,TIM_ICPOLARITY_FALLING);  //设置为下降沿捕获
				capture_Cnt[2]++; //从模式一切换到模式二
			  if(( level_before[2]=HAL_GPIO_ReadPin(wheel_3.GPIO_2,wheel_3.GPIO_PORT_2) )==RESET){   //判断正反转	           
					           direction[2] = BACKWARD;   //反转
//								printf("反转!\r\n");
				}
				else{direction[2] = FORWARD;   //正转
//					printf("正转!\r\n");
				}
				break;
			
			  case 1:   //  模式二  --------   捕获到下降沿 关闭计时 
				capture_Buf_latter[2] = (wheel_3.TIM.Instance) ->CCR1;//获取当前的捕获值
				HAL_TIM_IC_Stop_IT(&wheel_3.TIM,wheel_3.channelA); //停止捕获  或者: __HAL_TIM_DISABLE(&htim5);
			  level_latter[2]=HAL_GPIO_ReadPin(wheel_3.GPIO_2,wheel_3.GPIO_PORT_2);//再次读取B相
			  if(level_before[2] != level_latter[2]){   //判断是否是毛刺
					if(capture_Buf_latter[2]<capture_Buf_before[2]){       //溢出计算
				    real_time[2] = (65535-capture_Buf_before[2])+capture_Buf_latter[2] + 1;
			      }
					else{
						real_time[2] =  capture_Buf_latter[2]- capture_Buf_before[2] ;  //没有溢出
			      }
					if(!frist_flag[2]){    //前八次要先收集起来
						temp_time[2][temp_i[2]] = real_time[2];
						temp_i[2]++;
						if(temp_i[2]==8){
							for(uint8_t i =0; i<8;i++){
								real_time[2]+= temp_time[2][i];
							}
							high_time[2] = real_time[2]/8;
							frist_flag[2]=1;
						}
					}
					else{
						high_time[2] = 0;
						memmove(&temp_time[2], &temp_time[2][1], sizeof(uint32_t) * 7);
						temp_time[2][7] = real_time[2];
						for(uint8_t i =0; i<8;i++){
								real_time[2]+= temp_time[2][i];	
							}
						high_time[2] = real_time[2]/8;
					}
					pulse_num[2]+= direction[2] ;     //计算脉冲个数用于计算位移
							if(pulse_num[2]>=MAX_pulse){
						pulse_out[2]++;
						pulse_num[2]=0;
					}
					else if(pulse_num[0]<=-MAX_pulse){
						pulse_out[2]--;
						pulse_num[2]=0;
					}
				}
			  //printf("high_time = %5d us   direction = %d\r\n",high_time[0],direction[0]);     //发高电平时间
			  capture_Cnt[2] = 0;  //清空标志
					HAL_TIM_IC_Start_IT(&(wheel_3.TIM), wheel_3.channelA);	  //启动输入捕获
				__HAL_TIM_SET_CAPTUREPOLARITY(&wheel_3.TIM,wheel_3.channelA, TIM_INPUTCHANNELPOLARITY_RISING);  //开启上升沿捕获
			  break; 
		 }
	}
	
	//右后
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2 )
	{
		dog[3]=0;
		switch(capture_Cnt[3]){
			
			case 0:   //模式1   --------   捕获到上升沿 开始计时
				capture_Buf_before[3] = (wheel_4.TIM.Instance) ->CCR2;//获取当前的捕获值
				__HAL_TIM_SET_CAPTUREPOLARITY(&wheel_4.TIM,wheel_4.channelA,TIM_ICPOLARITY_FALLING);  //设置为下降沿捕获
				capture_Cnt[3]++; //从模式一切换到模式二
			  if(( level_before[3]=HAL_GPIO_ReadPin(wheel_4.GPIO_2,wheel_4.GPIO_PORT_2) )==RESET){   //判断正反转	           
					           direction[3] = BACKWARD;   //反转
				  				//	printf("GHHJWSDHDCHJHJ转!\r\n");
				}
				else{direction[3] = FORWARD;   //正转
									//	printf("正转!\r\n");
				}
				break;
			
			  case 1:   //  模式二  --------   捕获到下降沿 关闭计时 
				capture_Buf_latter[3] = (wheel_4.TIM.Instance) ->CCR2;//获取当前的捕获值
				HAL_TIM_IC_Stop_IT(&wheel_4.TIM,wheel_4.channelA); //停止捕获  或者: __HAL_TIM_DISABLE(&htim5);
			  level_latter[3]=HAL_GPIO_ReadPin(wheel_4.GPIO_2,wheel_4.GPIO_PORT_2);//再次读取B相
			  if(level_before[3] != level_latter[3]){   //判断是否是毛刺
					if(capture_Buf_latter[3]<capture_Buf_before[3]){       //溢出计算
				    real_time[3] = (65535-capture_Buf_before[3])+capture_Buf_latter[3] + 1 ;
			      }
					else{
						real_time[3] =  capture_Buf_latter[3]- capture_Buf_before[3] ;  //没有溢出
			      }
					if(!frist_flag[3]){    //前八次要先收集起来
						temp_time[3][temp_i[3]] = real_time[3];
						temp_i[3]++;
						if(temp_i[3]==8){
							for(uint8_t i =0; i<8;i++){
								real_time[3]+= temp_time[3][i];
							}
							high_time[3] = real_time[3]/8;
							frist_flag[3]=1;
						}
					}
					else{
						high_time[3] = 0;
						memmove(&temp_time[3], &temp_time[3][1], sizeof(uint32_t) * 7);
						temp_time[3][7] = real_time[3];
						for(uint8_t i =0; i<8;i++){
								real_time[3]+= temp_time[3][i];	
							}
						high_time[3] = real_time[3]/8;
					}
					pulse_num[3]+= direction[3] ;     //计算脉冲个数用于计算位移
							if(pulse_num[3]>=MAX_pulse){
						pulse_out[3]++;
						pulse_num[3]=0;
					}
					else if(pulse_num[3]<=-MAX_pulse){
						pulse_out[3]--;
						pulse_num[3]=0;
					}
				}
			  //printf("high_time = %5d us   direction = %d\r\n",high_time[0],direction[0]);     //发高电平时间
			  capture_Cnt[3] = 0;  //清空标志
					HAL_TIM_IC_Start_IT(&(wheel_4.TIM), wheel_4.channelA);	  //启动输入捕获
				__HAL_TIM_SET_CAPTUREPOLARITY(&wheel_4.TIM,wheel_4.channelA, TIM_INPUTCHANNELPOLARITY_RISING);  //开启上升沿捕获
			  break; 
		 }
	}
}
//重新计算脉冲
void encoder_clear(void)
{
	HAL_TIM_IC_Stop_IT(&wheel_1.TIM,wheel_1.channelA);
	HAL_TIM_IC_Stop_IT(&wheel_2.TIM,wheel_2.channelA);
	HAL_TIM_IC_Stop_IT(&wheel_3.TIM,wheel_3.channelA);
	HAL_TIM_IC_Stop_IT(&wheel_4.TIM,wheel_4.channelA);
	memset((void * )pulse_num,(int)0,(unsigned int )sizeof(pulse_num));//覆盖
	memset((void * )pulse_out,(int)0,(unsigned int )sizeof(pulse_out));//覆盖
	motor_all.Distance = 0;
	HAL_TIM_IC_Start_IT(&(wheel_1.TIM), wheel_1.channelA);	  //启动输入捕获
	HAL_TIM_IC_Start_IT(&(wheel_2.TIM), wheel_2.channelA);	  //启动输入捕获
	HAL_TIM_IC_Start_IT(&(wheel_3.TIM), wheel_3.channelA);	  //启动输入捕获
	HAL_TIM_IC_Start_IT(&(wheel_4.TIM), wheel_4.channelA);	  //启动输入捕获
}
