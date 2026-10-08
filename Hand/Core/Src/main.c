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
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "mpu6500.h"
#include "nrf24l01.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 手势编号（后续 NRF 发送阶段将直接用作命令字） */
#define GES_STOP   0	/* 放平 */
#define GES_FRONT  1	/* 前倾 */
#define GES_BACK   2	/* 后倾 */
#define GES_LEFT   3	/* 左倾 */
#define GES_RIGHT  4	/* 右倾 */
#define GES_ROTL   5	/* 左拧（原地逆时针） */
#define GES_ROTR   6	/* 右拧（原地顺时针） */
#define GES_LOCKTOG 7	/* 解锁/锁定车端（PA6 按下时连发，2026-09-14 加） */

/* PA6 解锁命令连发时长：按下后把命令 7 连发约 12 帧（30Hz × 400ms），
   即使中途丢几帧也能送到车端；车端按命令 7 的上升沿去重，只切换一次 */
#define LOCK_BURST_MS   400

/* 拧腕阈值：陀螺仪 Z 轴原始值。±100°/s × 131 LSB = ±13100（2026-09-12 用户实测定） */
#define TWIST_RAW_THRESHOLD   13100

/* 拧腕锁定时间：一次拧腕结束后，这段时间内不再识别任何旋转（防回正误判成反向旋转） */
#define TWIST_LOCK_MS   1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

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
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();				/* OLED 初始化（内部会重跑 MX_GPIO_Init，必须最先） */
  NRF24L01_Init();			/* NRF 模块配置好待命，发送受总开关控制 */
  /* 上电默认总开关关：只显示关机提示，按 PA0 才开机（采样/显示/NRF 发送全开） */
  OLED_Clear();
  OLED_ShowString(1, 1, "SENSOR OFF");
  OLED_ShowString(3, 1, "Press key to");
  OLED_ShowString(4, 1, "start");
  if (MPU6500_Init(&hi2c2) != HAL_OK)	/* 传感器自检（读 WHO_AM_I） */
  {
      OLED_ShowString(1, 1, "MPU6500 ERROR");	/* 覆盖关机提示第一行 */
      OLED_ShowString(2, 1, "Check wire/AD0 ");
      while (1) {}			/* 传感器没响应，停在这里排查接线 */
  }
  /* 旋转指示输出脚初始拉低（顺时针=PA3，识别到才置高；PA6 已改做解锁按键） */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 60Hz 节奏：每 16ms 处理一次按键 + 采样一次传感器（16ms ≈ 62Hz） */
    static uint32_t lastTick = 0;
    static uint8_t  gSysOn     = 0;         /* 总开关：0=整机停，1=工作（PA0 按一下切换） */
    static uint8_t  lastGesture = GES_STOP; /* 上次显示的手势，变化时才重画 */
    static uint8_t  btnState   = 0;         /* 防抖后的按键稳定电平（按下=1） */
    static uint8_t  btnRawLast = 0;
    static uint32_t btnDebCnt  = 0;
    static uint16_t redrawCnt  = 0;         /* 定期整屏重刷计数（修软 I2C 偶发花屏） */
    static uint32_t twistLockUntil = 0;     /* 拧腕锁定截止时刻：到点前不识别新旋转 */
    static int8_t   twistDir = 0;           /* 持续中的拧腕方向：-1 顺 / 0 无 / +1 逆 */
    static uint8_t  txGesture = GES_STOP;   /* 要发送的手势（MPU 读失败时=STOP） */
    static uint8_t  lockBtnState   = 0;     /* PA6 解锁按键：防抖后的稳定电平（按下=1） */
    static uint8_t  lockBtnRawLast = 0;
    static uint32_t lockBtnDebCnt  = 0;
    static uint32_t lockBurstUntil = 0;     /* 命令 7 连发截止时刻（0=不在连发） */
    int16_t ax, ay, az;
    int16_t gx, gy, gz = 0;                 /* gz = 陀螺仪 Z 轴，拧腕旋转用 */
    float pitch, roll;

    if (HAL_GetTick() - lastTick >= 16)
    {
        lastTick = HAL_GetTick();

        /* ---- 按键消抖：PA0 一端接 3.3V、另一端接引脚（内部下拉，按下读 1） ---- */
        uint8_t btnRaw = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
        if (btnRaw != btnRawLast)
        {
            btnRawLast = btnRaw;            /* 电平变化，重新累计稳定时间 */
            btnDebCnt = 0;
        }
        else if (++btnDebCnt >= 3)          /* 稳定 3 个采样周期（约 48ms）才认 */
        {
            btnDebCnt = 3;                  /* 封顶防溢出 */
            if (btnRaw != btnState)
            {
                btnState = btnRaw;
                if (btnState == 1)          /* 按下边沿：总开关 开/关 切换 */
                {
                    gSysOn = !gSysOn;
                    if (gSysOn)
                    {
                        /* 开机：清屏，只显示当前动作（第 2 行居中） */
                        OLED_Clear();
                        OLED_ShowString(2, 5, "STOP   ");
                        lastGesture = GES_STOP;
                        txGesture = GES_STOP;   /* 清掉关机前的旧手势 */
                    }
                    else
                    {
                        /* 关机：采样、显示、NRF 发送全部停止，只剩提示 */
                        OLED_Clear();
                        OLED_ShowString(1, 1, "SENSOR OFF");
                        OLED_ShowString(3, 1, "Press key to");
                        OLED_ShowString(4, 1, "start");
                        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);	/* 旋转指示清零 */
                    }
                }
            }
        }

        /* ---- PA6 解锁按键消抖：接法和 PA0 完全一样（一端接 3.3V、另一端接引脚，
               内部下拉，按下读 1）。按下边沿 → 启动命令 7 连发 400ms，
               车端收到后切换 解锁/锁定。
               2026-09-14 加：车端断联自动上锁后，不用再弯腰按车上的 PA15 ---- */
        uint8_t lockBtnRaw = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
        if (lockBtnRaw != lockBtnRawLast)
        {
            lockBtnRawLast = lockBtnRaw;    /* 电平变化，重新累计稳定时间 */
            lockBtnDebCnt = 0;
        }
        else if (++lockBtnDebCnt >= 3)      /* 稳定 3 个采样周期（约 48ms）才认 */
        {
            lockBtnDebCnt = 3;
            if (lockBtnRaw != lockBtnState)
            {
                lockBtnState = lockBtnRaw;
                if (lockBtnState == 1)      /* 按下边沿：启动解锁命令连发 */
                {
                    lockBurstUntil = HAL_GetTick() + LOCK_BURST_MS;
                }
            }
        }

        /* ---- 每 0.5s 重刷一遍静态内容（不清屏避免闪烁）：软 I2C 偶发丢字节
               弄花屏幕时，半秒内自动修复 ---- */
        if (++redrawCnt >= 32)          /* 32 × 16ms ≈ 0.5s */
        {
            redrawCnt = 0;
            if (gSysOn)
            {
                lastGesture = 0xFF;     /* 手势文字下一拍强制重画 */
            }
            else
            {
                OLED_ShowString(1, 1, "SENSOR OFF");
                OLED_ShowString(3, 1, "Press key to");
                OLED_ShowString(4, 1, "start");
            }
        }

        /* ---- 总开关打开时才干活：采样 + 显示 +（后续）NRF 发送 ---- */
        if (gSysOn && MPU6500_ReadAccel(&hi2c2, &ax, &ay, &az) == HAL_OK)
        {
            MPU6500_CalcAngles(ax, ay, az, &pitch, &roll);
            MPU6500_ReadGyro(&hi2c2, &gx, &gy, &gz);	/* 读陀螺仪（拧腕看 gz） */

            /* 手势分类：拧腕优先（动作最明确），再判倾斜（±20° 死区）；
               文字只在分类变化时重画 */
            uint8_t gesture = GES_STOP;

            /* 拧腕状态机：
               - 空闲且 1 秒锁定期已过才识别新拧腕，方向看 gz 符号
               - 同方向持续拧动时保持该手势显示
               - 角速度回到死区 = 本次拧腕结束，启动 1 秒锁定，
                 防止回正的反向角速度被误判成反向旋转 */
            if (gz > TWIST_RAW_THRESHOLD)
            {
                if (twistDir == 0 && (int32_t)(HAL_GetTick() - twistLockUntil) >= 0)
                    twistDir = 1;               /* 新识别：左拧/逆时针 */
                if (twistDir == 1)
                    gesture = GES_ROTL;
            }
            else if (gz < -TWIST_RAW_THRESHOLD)
            {
                if (twistDir == 0 && (int32_t)(HAL_GetTick() - twistLockUntil) >= 0)
                    twistDir = -1;              /* 新识别：右拧/顺时针 */
                if (twistDir == -1)
                    gesture = GES_ROTR;
            }
            else if (twistDir != 0)
            {
                twistDir = 0;                   /* 拧腕结束，开始 1 秒锁定 */
                twistLockUntil = HAL_GetTick() + TWIST_LOCK_MS;
            }

            if (gesture == GES_STOP)            /* 没有拧腕（或锁定期内）才判倾斜 */
            {
                if (pitch > 20.0f)          gesture = GES_FRONT; /* 前倾 */
                else if (pitch < -20.0f)    gesture = GES_BACK;  /* 后倾 */
                else if (roll > 20.0f)      gesture = GES_LEFT;  /* 左倾（按当前安装方向） */
                else if (roll < -20.0f)     gesture = GES_RIGHT; /* 右倾 */
            }

            if (gesture != lastGesture)
            {
                lastGesture = gesture;
                switch (gesture)
                {
                    case GES_FRONT:  OLED_ShowString(2, 5, "FRONT  "); break;
                    case GES_BACK:   OLED_ShowString(2, 5, "BACK   "); break;
                    case GES_LEFT:   OLED_ShowString(2, 5, "LEFT   "); break;
                    case GES_RIGHT:  OLED_ShowString(2, 5, "RIGHT  "); break;
                    case GES_ROTL:   OLED_ShowString(2, 5, "ROT L  "); break;
                    case GES_ROTR:   OLED_ShowString(2, 5, "ROT R  "); break;
                    default:         OLED_ShowString(2, 5, "STOP   "); break;
                }
            }

            /* 旋转指示输出：顺时针→PA3 高电平（电平实时跟随识别结果；
               PA6 已改做解锁按键输入，逆时针指示取消） */
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, (gesture == GES_ROTR) ? GPIO_PIN_SET : GPIO_PIN_RESET);

            txGesture = gesture;        /* 记录本拍识别结果，供下方发送块使用 */
        }
        else if (gSysOn)
        {
            /* MPU 读失败（I2C 瞬断）：本拍按放平处理——发 STOP 让车安全停车，
               同时链路不断，车端不会误判失联重新上锁 */
            txGesture = GES_STOP;
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        }

        /* ---- NRF 发送：每 2 拍发一包（约 30Hz）。独立于采样块：
               只要总开关开就发，MPU 读失败时也照发 STOP 保链；
               2026-09-12 由 15Hz 提到 30Hz：电机 EMI 压接收时，
               帧流越密，漏进干扰间隙的概率越高 ---- */
        static uint8_t sendCnt = 0;
        static uint8_t seq = 0;
        if (gSysOn && ++sendCnt >= 2)
        {
            sendCnt = 0;
            NRF24L01_TxPacket[0] = 0xA5;                    /* 帧头 */
            /* 命令字：PA6 刚按下后的连发窗口内优先发 7（解锁/锁定切换），
               其余时间发手势编号 0~6 */
            NRF24L01_TxPacket[1] = ((int32_t)(HAL_GetTick() - lockBurstUntil) < 0)
                                 ? GES_LOCKTOG : txGesture;
            NRF24L01_TxPacket[2] = seq++;                   /* 序号 */
            NRF24L01_TxPacket[3] = NRF24L01_TxPacket[0] + NRF24L01_TxPacket[1]
                                 + NRF24L01_TxPacket[2];    /* 校验和（前三字节之和） */
            NRF24L01_Send();                /* 发送失败不处理：车端有失联保护 */
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
