#ifndef __BSP_LINEFOLLOWER_H
#define __BSP_LINEFOLLOWER_H

#include "sys.h"

#define LFB_SENSOR_NUM 8   //循迹板传感器数量
#define LFB_HALF_SENSOR_NUM 8

#define Infrared_ahead_LEFT  (uint8_t)!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11)


#endif

