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
uint8_t NRF24L01_SelfCheck(void);	/* 上电自检：寄存器写入-回读，返回值 0x5A=正常 */
uint8_t NRF24L01_GetConfig(void);	/* 读 CONFIG 寄存器（诊断用，正常应=0x0B） */

#endif
