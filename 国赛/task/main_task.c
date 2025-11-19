#include "main_task.h"
#include "rudder_control.h"
#include "uart.h"
#include "imu_task.h"
#include "uart.h"
#include "turn.h"
#include "map.h"
#include "barrier.h"
#include "bsp_buzzer.h"
#include "bsp_linefollower.h"
#include "scaner.h"
#include "speed_ctrl.h"
#include "encoder.h"
#include "barrier.h"
#include "motor_task.h"
#include "openmv.h"
#include "math.h"
#include "barrier.h"
void main_task(void *pvParameters){
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();   //获取系统节拍
	zhunbei();


	while(1)
	{
		Cross();
		if(map.routetime==1)
		{
			CarBrake();
			while(1)
			{
				vTaskDelay(2);
			}
		}
	
		vTaskDelayUntil(&xLastWakeTime, (1/portTICK_RATE_MS));//绝对休眠5ms // INCLUDE_vTaskDelayUntil 1
	}
}
