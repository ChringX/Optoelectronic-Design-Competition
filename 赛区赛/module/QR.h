#ifndef __QR_h__
#define __QR_h__
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "sys.h"

extern uint8_t TreaPoint[8];
#define QRBUFFER_SIZE 10
extern uint8_t QR_rx_buf[QRBUFFER_SIZE];
extern uint8_t QR_rx_len;
void QR_receive_init(void);
void QR_init(uint32_t bound);
#endif
