/**
  ******************************************************************************
  * @file    motor.c
  * @brief   车端电机控制层实现（2 块 TB6612 驱动 4 个麦轮）
  *
  *          控制原理：
  *            每个轮子 = 1 路 PWM（转速）+ 2 个方向脚（转向）
  *            方向脚真值表（TB6612）：
  *              AIN1=0 AIN2=0  停（滑行）
  *              AIN1=1 AIN2=0  正转
  *              AIN1=0 AIN2=1  反转
  *              AIN1=1 AIN2=1  刹车（短接制动，本工程停车用这个）
  ******************************************************************************
  */
#include "motor.h"
#include "tim.h"

/*==================================== 麦轮方向表 ====================================*/
/* 每个动作下四个轮子的转向：1=正转（车前进方向）  -1=反转  0=刹车
 * 注意：平移两行的方向取决于麦轮滚子朝向（A/B 轮安装方式），
 *       实车测试若发现平移方向反了，把 CAR_LEFT 与 CAR_RIGHT 两行互换即可，
 *       其他代码不用动。 */
static const int8_t MecanumDir[CAR_ROT_CW + 1][4] =
{
	/*                 LF   LR   RF   RR */
	/* CAR_STOP    */ { 0,   0,   0,   0},
	/* CAR_FWD     */ {-1,  -1,  -1,  -1},
	/* CAR_BACK    */ { 1,   1,   1,   1},
	/* CAR_LEFT    */ { 1,  -1,  -1,   1},
	/* CAR_RIGHT   */ {-1,   1,   1,  -1},
	/* CAR_ROT_CCW */ { 1,   1,  -1,  -1},
	/* CAR_ROT_CW  */ {-1,  -1,   1,   1},
	/* 2026-09-13 用户实测整车行进方向与手势相反：平移四行整体取反
	   （FWD↔BACK、LEFT↔RIGHT 互换）；
	   2026-09-13 晚实测原地旋转方向也反：旋转两行同样互换（CCW↔CW） */
};

/* 每个轮子的硬件信息：PWM 通道 + 两个方向脚 */
typedef struct
{
	uint32_t      pwmCh;		/* TIM 通道，如 TIM_CHANNEL_1 */
	GPIO_TypeDef *ain1Port;		/* AIN1 端口 */
	uint16_t      ain1Pin;		/* AIN1 引脚 */
	GPIO_TypeDef *ain2Port;		/* AIN2 端口 */
	uint16_t      ain2Pin;		/* AIN2 引脚 */
} MotorInfo_t;

static const MotorInfo_t MotorInfo[4] =
{
	/* WHEEL_LF 左前轮：PA0 接左 TB6612 的 PWMA，PB0/PB1 接 AIN1/AIN2 */
	{ LF_PWM_CH, LF_AIN1_PORT, LF_AIN1_PIN, LF_AIN2_PORT, LF_AIN2_PIN },
	/* WHEEL_LR 左后轮：PA1 接左 TB6612 的 PWMB，PB10/PB11 接 BIN1/BIN2 */
	{ LR_PWM_CH, LR_AIN1_PORT, LR_AIN1_PIN, LR_AIN2_PORT, LR_AIN2_PIN },
	/* WHEEL_RF 右前轮：PA2 接右 TB6612 的 PWMA，PB14/PB15 接 AIN1/AIN2 */
	{ RF_PWM_CH, RF_AIN1_PORT, RF_AIN1_PIN, RF_AIN2_PORT, RF_AIN2_PIN },
	/* WHEEL_RR 右后轮：PA3 接右 TB6612 的 PWMB，PB13/PC15 接 BIN1/BIN2
	   （2026-09-14 原 PB12 烧坏：高电平输出对地漏电，BIN2 挪到 PC15） */
	{ RR_PWM_CH, RR_AIN1_PORT, RR_AIN1_PIN, RR_AIN2_PORT, RR_AIN2_PIN },
};

/**
  * @brief  设置单个轮子的转向
  * @param  wheel 轮子编号（WHEEL_LF ~ WHEEL_RR）
  * @param  dir   1=正转  0=刹车  -1=反转
  * @retval 无
  * @note   刹车 = AIN1、AIN2 同时置 1（TB6612 真值表：1/1 为短接制动）
  */
static void Motor_SetDir(Wheel_t wheel, int8_t dir)
{
	const MotorInfo_t *info = &MotorInfo[wheel];

	if (dir > 0)						/* 正转：AIN1=1 AIN2=0 */
	{
		HAL_GPIO_WritePin(info->ain1Port, info->ain1Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(info->ain2Port, info->ain2Pin, GPIO_PIN_RESET);
	}
	else if (dir < 0)					/* 反转：AIN1=0 AIN2=1 */
	{
		HAL_GPIO_WritePin(info->ain1Port, info->ain1Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(info->ain2Port, info->ain2Pin, GPIO_PIN_SET);
	}
	else								/* 刹车：AIN1=1 AIN2=1 */
	{
		HAL_GPIO_WritePin(info->ain1Port, info->ain1Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(info->ain2Port, info->ain2Pin, GPIO_PIN_SET);
	}
}

/**
  * @brief  电机上电初始化：启动 4 路 PWM 并全部刹车
  * @param  无
  * @retval 无
  */
void Motor_Init(void)
{
	/* 启动 4 路 PWM（初始占空比为 0） */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	/* 上电默认刹车：车轮锁死，防止上电后溜车 */
	Car_Stop();
}

/* 每个轮子当前的输出速度（软启动爬升用） */
static int16_t s_curSpeed[4] = {0, 0, 0, 0};

/**
  * @brief  设置单个轮子的转速
  * @param  wheel 轮子编号（WHEEL_LF ~ WHEEL_RR）
  * @param  speed 转速：-1000~+1000（千分比占空比），正=前进方向，负=反转，0=刹车
  * @retval 无
  */
void Motor_SetWheel(Wheel_t wheel, int16_t speed)
{
	uint32_t period;
	uint32_t cmp;

	if (wheel > WHEEL_RR)
	{
		return;
	}

	/* 限幅到 -1000~+1000 */
	if (speed > 1000)
	{
		speed = 1000;
	}
	if (speed < -1000)
	{
		speed = -1000;
	}

	/* 软启动：起步和同向加/减速限斜率（每拍最多 ±MOTOR_RAMP_STEP），
	   缓慢爬升给电源响应时间；刹车/换向立即生效（急停安全优先）。
	   2026-09-12 加：四轮同时满速起步的电流冲击曾把降压模块拉垮
	   （3.3V 崩溃 → MCU 复位 → 屏幕上跳回 LOCKED） */
	if (speed == 0 || (s_curSpeed[wheel] != 0 && (speed > 0) != (s_curSpeed[wheel] > 0)))
	{
		s_curSpeed[wheel] = speed;	/* 刹车/换向：瞬间执行 */
	}
	else if (speed > s_curSpeed[wheel])
	{
		s_curSpeed[wheel] += MOTOR_RAMP_STEP;
		if (s_curSpeed[wheel] > speed)
		{
			s_curSpeed[wheel] = speed;
		}
	}
	else if (speed < s_curSpeed[wheel])
	{
		s_curSpeed[wheel] -= MOTOR_RAMP_STEP;
		if (s_curSpeed[wheel] < speed)
		{
			s_curSpeed[wheel] = speed;
		}
	}
	speed = s_curSpeed[wheel];

	/* 设置转向：正转 / 反转 / 刹车 */
	Motor_SetDir(wheel, (speed > 0) ? 1 : (speed < 0) ? -1 : 0);

	/* 占空比 = |speed| / 1000 * 周期（周期读 TIM2 实际配置，改 ARR 不用改这里） */
	period = htim2.Init.Period + 1;
	cmp = (uint32_t)(speed > 0 ? speed : -speed) * period / 1000;

	/* 重要：CCR 是 16 位寄存器，最大只能写到 ARR（Period）。
	   1000（100%）算出来是 65536，直接写会溢出归零（输出 0V，电机不动），
	   所以这里限幅到 Period（实际占空比 99.998%，效果就是满速） */
	if (cmp > htim2.Init.Period)
	{
		cmp = htim2.Init.Period;
	}
	__HAL_TIM_SET_COMPARE(&htim2, MotorInfo[wheel].pwmCh, cmp);
}

/**
  * @brief  整车动作（按麦轮方向表同时驱动四个轮子）
  * @param  dir 动作：CAR_FWD / CAR_BACK / CAR_LEFT / CAR_RIGHT /
  *             CAR_ROT_CCW / CAR_ROT_CW / CAR_STOP
  * @retval 无
  */
void Car_Move(CarDir_t dir)
{
	Wheel_t wheel;

	if (dir > CAR_ROT_CW)
	{
		return;
	}

	for (wheel = WHEEL_LF; wheel <= WHEEL_RR; wheel++)
	{
		Motor_SetWheel(wheel, (int16_t)MecanumDir[dir][wheel] * CAR_SPEED);
	}
}

/**
  * @brief  整车停止（四个轮子全部刹车，不是滑行）
  * @param  无
  * @retval 无
  */
void Car_Stop(void)
{
	Wheel_t wheel;

	for (wheel = WHEEL_LF; wheel <= WHEEL_RR; wheel++)
	{
		Motor_SetWheel(wheel, 0);
	}
}

/**
  * @brief  原地旋转 360° 后自动停止（定时开环方案）
  * @param  dir 旋转方向：CAR_ROT_CCW（逆时针）或 CAR_ROT_CW（顺时针）
  * @retval 无
  * @note   旋转时长 ROT_360_TIME_MS 需要实车标定；
  *         本函数为阻塞式（旋转期间不响应其他指令），后续 NRF 阶段可改为非阻塞
  */
void Car_Rotate360(CarDir_t dir)
{
	if (dir != CAR_ROT_CCW && dir != CAR_ROT_CW)
	{
		return;
	}

	Car_Move(dir);
	HAL_Delay(ROT_360_TIME_MS);
	Car_Stop();
}
