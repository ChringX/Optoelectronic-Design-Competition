#include "openmv.h"
#include "usart.h"
#include "math.h"
#include "barrier.h"
#include "stdio.h"
#include "Scaner.h"
#include "QR.h"
#include "string.h"
//color
//1 蓝队真宝藏
//2 红队真宝藏
//3 蓝队伪宝藏
//4 红队伪宝藏
//5 单独蓝色块
//6 单独红色块

UART_HandleTypeDef mv;//UART句柄
UART_HandleTypeDef mv_R;//UART句柄
uint8_t color;

void mv_init(uint32_t bound)
{	
	//UART 初始化设置
	mv.Instance=UART7;					  
	mv.Init.BaudRate=bound;				    //波特率
	mv.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	mv.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	mv.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	mv.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	mv.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&mv);					    //HAL_UART_Init()会使能UART3
	__HAL_UART_ENABLE_IT(&mv, UART_IT_RXNE);
	close_mv();
}

void mvR_init(uint32_t bound)
{	
	//UART 初始化设置
	mv_R.Instance=USART6;					  
	mv_R.Init.BaudRate=bound;				    //波特率
	mv_R.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	mv_R.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	mv_R.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	mv_R.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	mv_R.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&mv_R);					    //HAL_UART_Init()会使能UART3
	__HAL_UART_ENABLE_IT(&mv_R, UART_IT_RXNE);
	close_mvR();
}

void open_mv()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
}

void close_mv()
{
	color = 0;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
}

void open_mvR()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
}

void close_mvR()
{
	color = 0;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
}

void UART7_IRQHandler(void)
{
	static uint8_t temp=0;
	static uint8_t flag=0;
//	if(__HAL_UART_GET_FLAG(&mv,UART_FLAG_RXNE)!=0){
		temp=mv.Instance->RDR;
		
		if(flag==1)
		{
			if(temp == 1)
				color = 1;
			else if(temp == 2)
				color = 2;
			else if(temp == 3)
				color = 3;
			else if(temp == 4)
				color = 4;
			else
				color = 0;
			flag=0;
		}
		if(temp==0xFF && flag==0)
		{
			flag=1;
		}
//	}

	mv.Instance->ISR = 0;   //清除SR标志位
}

void USART6_IRQHandler(void)
{
	static uint8_t temp=0;
//	static uint8_t flag=0;
//	uint8_t temp_color=0;
	
	if(__HAL_UART_GET_FLAG(&mv_R,UART_FLAG_RXNE)!=0){
		temp = mv_R.Instance->RDR;
		
//		if(temp == 0xff && flag == 0)
//		{
//			flag = 1;
//		}
//		if(flag ==1 && temp != 0)
//		{
//			TreaPoint[0] = temp;
//			flag = 2;
//		}if(flag ==2 && temp != 0)
//		{
//			TreaPoint[1] = temp;
//			flag = 3;
//		}if(flag ==3 && temp != 0)
//		{
//			TreaPoint[2] = temp;
//			flag = 4;
//		}if(flag ==4 && temp != 0)
//		{
//			TreaPoint[3] = temp;
//			flag = 5;
//		}if(flag ==5 && temp != 0)
//		{
//			TreaPoint[4] = temp;
//			flag = 6;
//		}if(flag ==6 && temp != 0)
//		{
//			TreaPoint[5] = temp;
//			flag = 7;
//		}if(flag ==7 && temp != 0)
//		{
//			TreaPoint[6] = temp;
//			flag = 8;
//		}if(flag ==8 && temp != 0)
//		{
//			TreaPoint[7] = temp;
//		}
//		if(temp == 0)
//		{
//			for(uint8_t i = 0;i < 8;i++)
//				TreaPoint[i] = 0;
//			
//			flag = 0;
//		}
	}
	mv_R.Instance->ISR = 0;   //清除SR标志位
}
/*****************************************************************************
函数名： bofang()
函数功能：播放歌曲
形参：无
*******************************************************************************/
void bofang()
{
	  uint8_t data[5]={0x7e,0x03,0x01,0x02,0xef};
	  HAL_UART_Transmit(&huart6,data,5,0xFFFF);
}
/*****************************************************************************
函数名： stop_bofang()
函数功能：停止播放歌曲
形参：无
*******************************************************************************/
void stop_bofang()
{
	uint8_t data[5]={0x7e,0x03,0x02,0x01,0xef};
	HAL_UART_Transmit(&huart6,data,5,0xFFFF);
}
//播放指定歌曲
void bofang_zhiding(int shou)
{
	//7E 05 41 00(歌曲高位) 01(歌曲低位) 45(校验和) EF
	  uint8_t data[7]={0x7e,0x05,0x41,0x00,0x00,0x00,0xef};
	  data[4]=shou;
	  uint8_t sum=data[1]^data[2]^data[3]^data[4];
	  data[5]=sum;
//	   HAL_UART_Transmit(&huart3,data,7,0xFFFF);
//	   while(__HAL_UART_GET_FLAG(&huart3,USART_ISR_TXE)==0)
//	   {
//		   vTaskDelay(2);
//	   }
	  for(uint8_t i=0;i<7;i++)
	  {
		  HAL_UART_Transmit(&huart6,&data[i],1,0xFFFF);
	  }
	  
}
