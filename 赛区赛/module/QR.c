#include "QR.h"
#include "map.h"
#include "usart.h"
#include "math.h"
#include "stdio.h"
#include "string.h"
UART_HandleTypeDef  QR;//UART句柄
uint8_t BW_add=0;
#define QRBUFFER_SIZE 10
uint8_t QR_rx_buf[QRBUFFER_SIZE] = {0};
uint8_t QR_rx_len = 0;
uint8_t TreaPoint[8] = {0};
void QR_init(uint32_t bound)
{	
	//UART 初始化设置
	QR.Instance=USART2;					  
	QR.Init.BaudRate=bound;				    //波特率
	QR.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	QR.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	QR.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	QR.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	QR.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&QR);					    //HAL_UART_Init()会使能UART3
}

void QR_receive_init(void)
{
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart2,QR_rx_buf,QRBUFFER_SIZE);
}
void USART2_IRQHandler(void)
{
	uint8_t i;
	uint32_t flag_idle = 0;
	flag_idle = __HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE); 
	if((flag_idle != RESET))
	{ 	
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);
		HAL_UART_DMAStop(&huart2); 
		uint32_t temp = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);   
		QR_rx_len = QRBUFFER_SIZE - temp; 
		if(QR_rx_buf[0] == 0xff && QR_rx_buf[9] == 0xaa)
		{
			if(QR_rx_buf[1]!=0 && QR_rx_buf[2]!=0 && QR_rx_buf[3]!=0 && QR_rx_buf[4]!=0 
				&& QR_rx_buf[5]!=0 && QR_rx_buf[6]!=0 && QR_rx_buf[7]!=0 && QR_rx_buf[8]!=0)
			{
				for(i = 0;i < 8;i++)
				{
					TreaPoint[i] = QR_rx_buf[i + 1];
				}
			}
		}
		memset(QR_rx_buf,0,QR_rx_len);
		QR_rx_len = 0;
	}
	
	HAL_UART_Receive_DMA(&huart2,QR_rx_buf,QRBUFFER_SIZE);
	HAL_UART_IRQHandler(&huart2);
}

