#include "sin_generate.h"
#include "math.h"

#define PI 3.1415926
TIM_HandleTypeDef TIM5_Handler;      //定时器句柄 
struct sin_param sin1={0,0,1000,0.15};
float sin_generator(struct sin_param *param)
{
	float output;
	
	param->actual_t = param->time * param->angular_velocity;
	
	output = param->gain * sin(param->actual_t * PI/180);
	
	++param->time;
	
	if (param->actual_t >= 360)
		param->time = 0;
	
	return output;
}
void Timer5_Init()
{	//定时器5 1ms
   __HAL_RCC_TIM5_CLK_ENABLE();
     
    TIM5_Handler.Instance=TIM5;                          //通用定时器4
    TIM5_Handler.Init.Prescaler=215;                     //分频
    TIM5_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    TIM5_Handler.Init.Period=999;                        //自动装载值
    TIM5_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&TIM5_Handler);
		HAL_NVIC_SetPriority(TIM5_IRQn,0,0);    //设置中断优先级，抢占优先级3，子优先级3
    HAL_NVIC_EnableIRQ(TIM5_IRQn);          //开启ITM4中断  
    HAL_TIM_Base_Start_IT(&TIM5_Handler); //使能定时器4和定时器4中断 
}

void TIM5_IRQHandler(void)
{ 	
    if(__HAL_TIM_GET_IT_SOURCE(&TIM5_Handler,TIM_IT_UPDATE)==SET)//溢出中断
    {
		sin1.time++;
    }
    __HAL_TIM_CLEAR_IT(&TIM5_Handler, TIM_IT_UPDATE);//清除中断标志位
}
