/**
  ******************************************************************************
  * @file    motor.h
  * @brief   车端电机控制层接口（2 块 TB6612 驱动 4 个麦轮）
  *
  *          硬件结构：
  *            左 TB6612（A 路 -> 左前轮，B 路 -> 左后轮）
  *              PWMA=PA0  AIN1=PB0  AIN2=PB1
  *              PWMB=PA1  BIN1=PB10 BIN2=PB11
  *            右 TB6612（A 路 -> 右前轮，B 路 -> 右后轮）
  *              PWMA=PA2  AIN1=PB14 AIN2=PB15
  *              PWMB=PA3  BIN1=PB13 BIN2=PC15（2026-09-14 原 PB12 烧坏挪来）
  *            两块 TB6612 的 STBY 均接 3.3V 常使能
  ******************************************************************************
  */
#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

/*==================================== 用户可调参数（实车标定区） ====================================*/
/* 恒定运行速度：0~1000（占空比千分比），所有动作统一使用该速度
   2026-09-13 用户自行改 999（近满占空比）。注意：最早 1000 曾把电源
   拉垮导致 MCU 复位，软启动只挡起步冲击；动作中若整屏闪回 LOCKED
   = 电源不够，需降回 500 以下 */
 #define CAR_SPEED			850    

/* 软启动每拍加速量（主循环 60Hz 调用：从 0 爬到 500 约 17 拍 ≈ 280ms）
   2026-09-12 加：防四轮同时起步的电流冲击把电源拉垮 */
#define MOTOR_RAMP_STEP		30

/* 原地旋转 360° 的持续时间（毫秒），定时开环方案，必须实车标定
   2026-10-08 用户要求改为 2000ms */
#define ROT_360_TIME_MS		2000

/*==================================== 引脚定义（接线改变时只改这里） ====================================*/
/* 左前轮 LF */
#define LF_PWM_CH			TIM_CHANNEL_1			/* PA0 -> 左 TB6612 PWMA */
#define LF_AIN1_PORT		GPIOB
#define LF_AIN1_PIN			GPIO_PIN_0				/* PB0 -> 左 TB6612 AIN1 */
#define LF_AIN2_PORT		GPIOB
#define LF_AIN2_PIN			GPIO_PIN_1				/* PB1 -> 左 TB6612 AIN2 */

/* 左后轮 LR */
#define LR_PWM_CH			TIM_CHANNEL_2			/* PA1 -> 左 TB6612 PWMB */
#define LR_AIN1_PORT		GPIOB
#define LR_AIN1_PIN			GPIO_PIN_10				/* PB10 -> 左 TB6612 BIN1 */
#define LR_AIN2_PORT		GPIOB
#define LR_AIN2_PIN			GPIO_PIN_11				/* PB11 -> 左 TB6612 BIN2 */

/* 右前轮 RF */
#define RF_PWM_CH			TIM_CHANNEL_3			/* PA2 -> 右 TB6612 PWMA */
#define RF_AIN1_PORT		GPIOB
#define RF_AIN1_PIN			GPIO_PIN_14				/* PB14 -> 右 TB6612 AIN1 */
#define RF_AIN2_PORT		GPIOB
#define RF_AIN2_PIN			GPIO_PIN_15				/* PB15 -> 右 TB6612 AIN2 */

/* 右后轮 RR */
#define RR_PWM_CH			TIM_CHANNEL_4			/* PA3 -> 右 TB6612 PWMB */
#define RR_AIN1_PORT		GPIOB
#define RR_AIN1_PIN			GPIO_PIN_13				/* PB13 -> 右 TB6612 BIN1 */
#define RR_AIN2_PORT		GPIOC
#define RR_AIN2_PIN			GPIO_PIN_15				/* PC15 -> 右 TB6612 BIN2（2026-09-14 原 PB12 烧坏：高电平输出 0V/2.5V 跳动，对地漏电） */

/*==================================== 类型定义 ====================================*/
/* 轮子编号（与 motor.c 里 MotorInfo 数组下标一致） */
typedef enum
{
	WHEEL_LF = 0,			/* 左前轮 */
	WHEEL_LR,				/* 左后轮 */
	WHEEL_RF,				/* 右前轮 */
	WHEEL_RR				/* 右后轮 */
} Wheel_t;

/* 整车动作 */
typedef enum
{
	CAR_STOP = 0,			/* 停止 */
	CAR_FWD,				/* 前进 */
	CAR_BACK,				/* 后退 */
	CAR_LEFT,				/* 左平移 */
	CAR_RIGHT,				/* 右平移 */
	CAR_ROT_CCW,			/* 原地逆时针旋转 */
	CAR_ROT_CW				/* 原地顺时针旋转 */
} CarDir_t;

/*==================================== 接口函数 ====================================*/
void Motor_Init(void);								/* 上电初始化：启动 PWM 并刹车 */
void Motor_SetWheel(Wheel_t wheel, int16_t speed);	/* 单轮控制：-1000~+1000，0=刹车 */
void Car_Move(CarDir_t dir);						/* 整车动作（按麦轮方向表） */
void Car_Stop(void);								/* 整车刹车停止 */
void Car_Rotate360(CarDir_t dir);					/* 原地旋转 360° 后自动停（阻塞） */

#endif /* __MOTOR_H */
