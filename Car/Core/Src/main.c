/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "OLED.h"
#include "nrf24l01.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 命令字（与手端 main.c 的 GES_* 编号一致，手端直接把手势编号当命令发） */
#define CMD_STOP    0	/* 放平：停车 */
#define CMD_FRONT   1	/* 前倾：前进 */
#define CMD_BACK    2	/* 后倾：后退 */
#define CMD_LEFT    3	/* 左倾：左平移 */
#define CMD_RIGHT   4	/* 右倾：右平移 */
#define CMD_ROTL    5	/* 左拧：原地逆时针旋转 360° */
#define CMD_ROTR    6	/* 右拧：原地顺时针旋转 360° */
#define CMD_LOCKTOG 7	/* 手端 PA6 按下：解锁/锁定切换（2026-09-14 加，遥控解锁） */

/* 失联保护（自动重连，2026-09-26 改）：
   收不到有效包超过 LINK_TIMEOUT_MS → 立即停车（安全第一），但不再上锁；
   之后连续稳定收到 LINK_GOOD_FRAMES 个有效包（约 0.3s @30Hz）自动恢复响应，
   全程不用按键。上锁只剩两种途径：上电默认锁定 + 手端 PA6（命令 7）
   电机 EMI 会压死车端 NRF 接收（车一停链路就恢复），自动重连后车能自己继续走 */
#define LINK_TIMEOUT_MS     500
#define LINK_GOOD_FRAMES    10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 主循环状态变量都在 USER CODE 3 里用 static 定义（和手端风格一致） */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();				/* OLED 初始化（内部自带上电延时） */
  Motor_Init();				/* 电机初始化（上电刹车，车轮锁死） */
  /* 注意顺序：Motor_Init 必须在 OLED_Init 之后，因为 OLED 驱动内部会重跑
     MX_GPIO_Init，把方向脚清成低电平；后初始化电机才能保证最终状态是刹车 */
  NRF24L01_Init();			/* NRF 初始化（默认进入接收模式） */
  /* 开机界面：上电默认锁定，不响应手势 */
  OLED_Clear();
  OLED_ShowString(1, 1, "SYS: LOCKED  ");
  OLED_ShowString(2, 1, "ACT: STOP    ");
  OLED_ShowString(3, 1, "LINK: LOST  ");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* ---- 主循环：60Hz 节奏，NRF 接收 + 命令执行 + OLED 显示 ---- */
    static uint32_t lastTick = 0;
    static uint32_t lastCmdTick = 0;        /* 最后一次收到有效包的时刻 */
    static uint8_t  carLocked  = 1;         /* 整车开关：上电默认锁定（仅手端 PA6 可切换） */
    static uint8_t  lastCmd    = CMD_STOP;  /* 最近一次有效命令 */
    static uint32_t rotUntil   = 0;         /* 旋转结束时刻（0=不在旋转） */
    static uint8_t  rotDisp    = CMD_STOP;  /* 旋转中 OLED 显示的动作 */
    static uint8_t  lastSYS    = 0xFF;      /* OLED 三行显示缓存（0xFF=强制重画） */
    static uint8_t  lastACT    = 0xFF;
    static uint8_t  lastLINK   = 0xFF;
    static uint8_t  linkEver   = 0;         /* 收到过有效包才显示 LINK OK */
    static uint8_t  lockCmdSeen = 0;        /* 命令 7 上升沿去重：同一次连发只切换一次 */
    static uint16_t linkGoodCnt = 0;        /* 连续有效包计数：失联后攒够 LINK_GOOD_FRAMES 才恢复执行 */
    static uint16_t redrawCnt  = 0;         /* 每 0.5s 全刷防软 I2C 偶发花屏 */
    uint32_t now;

    if (HAL_GetTick() - lastTick >= 16)
    {
        lastTick = HAL_GetTick();
        now = HAL_GetTick();

        /* ---- 1. NRF 接收（锁定状态下也接收，维持链路检测） ---- */
        uint8_t rx = NRF24L01_Receive();
        if (rx == 1)
        {
            uint8_t hdr = NRF24L01_RxPacket[0];
            uint8_t cmd = NRF24L01_RxPacket[1];
            if (hdr == 0xA5 && NRF24L01_RxPacket[3] == (uint8_t)(hdr + cmd + NRF24L01_RxPacket[2]))
            {
                lastCmdTick = now;          /* 有效包：喂链路狗 */
                linkEver = 1;
                if (linkGoodCnt < LINK_GOOD_FRAMES)
                    linkGoodCnt++;          /* 连续有效包计数（失联后攒够才恢复执行） */
                if (cmd == CMD_LOCKTOG)     /* 手端 PA6：上电启动/锁车切换（锁定期间也生效） */
                {
                    /* 手端按一次会连发一串命令 7（400ms ≈ 12 帧），
                       只在"7 的上升沿"（前一个是手势帧）切换一次，重复帧忽略 */
                    if (lockCmdSeen == 0)
                    {
                        lockCmdSeen = 1;
                        carLocked = !carLocked;
                        Car_Stop();
                        rotUntil = 0;
                        lastCmd = CMD_STOP; /* 解锁后停在原地，等新手势 */
                        lastSYS = 0xFF;     /* 强制刷新 SYS 行 */
                        lastACT = 0xFF;
                    }
                }
                else
                {
                    lockCmdSeen = 0;        /* 手势帧复位上升沿检测 */
                    if (!carLocked)
                    {
                        lastCmd = cmd;      /* 锁定期间忽略命令 */
                    }
                }
            }
            /* rx==2/3：驱动内部已自动重启初始化，忽略即可 */
        }

        /* ---- 2. 失联保护：只停车不上锁，链路稳定恢复后自动继续（自动重连） ---- */
        if (!carLocked && (now - lastCmdTick) > LINK_TIMEOUT_MS)
        {
            if (linkGoodCnt != 0)
            {
                linkGoodCnt = 0;            /* 进入失联：立即停车（安全第一），重连从零攒稳定包 */
                Car_Stop();
                rotUntil = 0;
                lastCmd = CMD_STOP;
                lastACT = 0xFF;
            }
        }

        /* ---- 3. 命令执行（解锁 + 链路就绪；失联重连攒够稳定包才恢复） ---- */
        if (!carLocked && (linkGoodCnt >= LINK_GOOD_FRAMES))
        {
            if (rotUntil != 0)              /* 旋转进行中：优先转完，忽略其他命令 */
            {
                if ((int32_t)(now - rotUntil) >= 0)
                {
                    Car_Stop();             /* 转满一圈，停车 */
                    rotUntil = 0;
                    lastCmd = CMD_STOP;     /* 防积压的旋转帧再次触发 */
                    while (NRF24L01_Receive() != 0);    /* 丢弃旋转期间积压的旧帧 */
                    lastACT = 0xFF;
                }
                else
                {
                    /* 旋转进行中也要每拍喂一次 Car_Move：软启动每拍只爬一步，
                       只在触发拍调用一次的话，整个旋转会停在约 6% 占空比上
                       （2026-09-12 实测现象：旋转又慢又像只有单边在动） */
                    Car_Move((rotDisp == CMD_ROTL) ? CAR_ROT_CCW : CAR_ROT_CW);
                }
            }
            else
            {
                switch (lastCmd)
                {
                    case CMD_STOP:  Car_Stop(); break;   /* 放平：停车（重复调用无害） */
                    case CMD_FRONT: Car_Move(CAR_FWD);  break;
                    case CMD_BACK:  Car_Move(CAR_BACK); break;
                    case CMD_LEFT:  Car_Move(CAR_LEFT); break;
                    case CMD_RIGHT: Car_Move(CAR_RIGHT); break;
                    case CMD_ROTL:                      /* 逆时针旋转 360°（定时开环） */
                        Car_Move(CAR_ROT_CCW);
                        rotUntil = now + ROT_360_TIME_MS;
                        rotDisp  = CMD_ROTL;
                        lastACT = 0xFF;
                        break;
                    case CMD_ROTR:                      /* 顺时针旋转 360° */
                        Car_Move(CAR_ROT_CW);
                        rotUntil = now + ROT_360_TIME_MS;
                        rotDisp  = CMD_ROTR;
                        lastACT = 0xFF;
                        break;
                    default: break;
                }
            }
        }

        /* ---- 4. OLED 三行显示（变化时重画 + 每 0.5s 全刷防花屏） ---- */
        uint8_t dispSYS  = carLocked ? 0 : 1;
        uint8_t dispACT  = (rotUntil != 0) ? rotDisp
                         : ((linkGoodCnt < LINK_GOOD_FRAMES) ? CMD_STOP : lastCmd);
        uint8_t dispLINK = (linkEver && ((now - lastCmdTick) <= LINK_TIMEOUT_MS)) ? 1 : 0;

        if (++redrawCnt >= 32)              /* 约 0.5s 全刷一遍 */
        {
            redrawCnt = 0;
            lastSYS = 0xFF;
            lastACT = 0xFF;
            lastLINK = 0xFF;
        }

        if (dispSYS != lastSYS)
        {
            lastSYS = dispSYS;
            if (dispSYS) OLED_ShowString(1, 1, "SYS: UNLOCKED");
            else         OLED_ShowString(1, 1, "SYS: LOCKED  ");
        }
        if (dispACT != lastACT)
        {
            lastACT = dispACT;
            switch (dispACT)
            {
                case CMD_FRONT: OLED_ShowString(2, 1, "ACT: FRONT   "); break;
                case CMD_BACK:  OLED_ShowString(2, 1, "ACT: BACK    "); break;
                case CMD_LEFT:  OLED_ShowString(2, 1, "ACT: LEFT    "); break;
                case CMD_RIGHT: OLED_ShowString(2, 1, "ACT: RIGHT   "); break;
                case CMD_ROTL:  OLED_ShowString(2, 1, "ACT: ROTATE L"); break;
                case CMD_ROTR:  OLED_ShowString(2, 1, "ACT: ROTATE R"); break;
                default:        OLED_ShowString(2, 1, "ACT: STOP    "); break;
            }
        }
        if (dispLINK != lastLINK)
        {
            lastLINK = dispLINK;
            if (dispLINK) OLED_ShowString(3, 1, "LINK: OK    ");
            else          OLED_ShowString(3, 1, "LINK: LOST  ");
        }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
