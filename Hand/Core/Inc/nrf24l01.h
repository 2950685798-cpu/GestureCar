#ifndef __NRF24L01_H
#define __NRF24L01_H

#include "main.h"

/* 外部可访问的发送/接收缓冲区 */
extern uint8_t NRF24L01_TxPacket[4];
extern uint8_t NRF24L01_RxPacket[4];

/* 函数声明 */
void NRF24L01_Init(void);
uint8_t NRF24L01_Send(void);
uint8_t NRF24L01_Receive(void);

#endif
