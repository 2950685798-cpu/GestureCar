#include "mpu6500.h"
#include <math.h>

/* 超时时间（毫秒），避免 I2C 故障时死锁 */
#define I2C_TIMEOUT  100

/* ---- 写一个字节到 MPU6500 寄存器 ---- */
static HAL_StatusTypeDef MPU6500_WriteReg(I2C_HandleTypeDef *hi2c,
                                          uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, &value, 1, I2C_TIMEOUT);
}

/* ---- 从 MPU6500 寄存器读取多个字节 ---- */
static HAL_StatusTypeDef MPU6500_ReadRegs(I2C_HandleTypeDef *hi2c,
                                          uint8_t reg, uint8_t *data, uint8_t len)
{
    return HAL_I2C_Mem_Read(hi2c, MPU6500_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);
}

/**
  * @brief  初始化 MPU6500
  * @param  hi2c: I2C 句柄指针（&hi2c1）
  * @retval HAL_OK: 成功, HAL_ERROR: 传感器未找到或 I2C 错误
  */
HAL_StatusTypeDef MPU6500_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t whoami = 0;
    HAL_StatusTypeDef status;

    /* 1. 验证芯片：读取 WHO_AM_I 寄存器
     *    MPU6500 = 0x70, MPU6050 = 0x68（市面模块常混用，兼容两种） */
    status = MPU6500_ReadRegs(hi2c, MPU6500_REG_WHO_AM_I, &whoami, 1);
    if (status != HAL_OK || (whoami != 0x70 && whoami != 0x68))
    {
        return HAL_ERROR;
    }

    /* 2. 唤醒 MPU6500（上电默认睡眠，bit6=1，写 0x00 清零） */
    MPU6500_WriteReg(hi2c, MPU6500_REG_PWR_MGMT_1, 0x00);
    HAL_Delay(50);  /* 等内部振荡器稳定（至少 35ms，取 50ms 留余量） */

    /* 3. 采样率分频 = 0 → 内部采样率 1kHz */
    MPU6500_WriteReg(hi2c, MPU6500_REG_SMPLRT_DIV, 0x00);

    /* 4. DLPF = 0 → 加速度带宽 260Hz，延迟最低 */
    MPU6500_WriteReg(hi2c, MPU6500_REG_CONFIG, 0x00);

    /* 5. 加速度计量程 ±2g（精度 16384 LSB/g，最适合倾角检测） */
    MPU6500_WriteReg(hi2c, MPU6500_REG_ACCEL_CONFIG, 0x00);

    /* 6. 陀螺仪量程 ±250°/s（寄存器默认值，显式写出便于以后改：
       0x00=±250、0x08=±500、0x10=±1000、0x18=±2000°/s） */
    MPU6500_WriteReg(hi2c, MPU6500_REG_GYRO_CONFIG, 0x00);

    HAL_Delay(10);  /* 等配置生效 */
    return HAL_OK;
}

/**
  * @brief  读取 MPU6500 三轴加速度原始值
  * @param  hi2c: I2C 句柄指针
  * @param  ax/ay/az: 输出指针，原始值（±2g → ±16384）
  * @retval HAL_OK: 成功
  */
HAL_StatusTypeDef MPU6500_ReadAccel(I2C_HandleTypeDef *hi2c,
                                    int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t raw[6];  /* ACCEL_XOUT_H 开始的 6 字节 */
    HAL_StatusTypeDef status;

    status = MPU6500_ReadRegs(hi2c, MPU6500_REG_ACCEL_XOUT_H, raw, 6);
    if (status != HAL_OK)
    {
        return status;
    }

    /* 大端字节序 → int16_t */
    *ax = (int16_t)((raw[0] << 8) | raw[1]);
    *ay = (int16_t)((raw[2] << 8) | raw[3]);
    *az = (int16_t)((raw[4] << 8) | raw[5]);

    return HAL_OK;
}

/**
  * @brief  根据加速度原始值计算俯仰角和翻滚角
  * @param  ax/ay/az: 三轴加速度原始值
  * @param  pitch: 输出指针，前后倾角（°），正=前倾
  * @param  roll:  输出指针，左右倾角（°），正=左倾（按当前安装方向，实车如反了改 main.c 判定符号）
  * @note   使用 atan2 直接算比值，LSB 单位自动约掉，无需转为 g 值
  */
void MPU6500_CalcAngles(int16_t ax, int16_t ay, int16_t az,
                        float *pitch, float *roll)
{
    /* 180/PI ≈ 57.29578，弧度转角度 */
    *pitch = atan2f((float)ax, (float)az) * 57.29578f;
    *roll  = atan2f((float)ay, (float)az) * 57.29578f;
}

/**
  * @brief  读取 MPU6500 三轴陀螺仪原始值
  * @param  hi2c: I2C 句柄指针
  * @param  gx/gy/gz: 输出指针，原始值（±250°/s → ±32768，131 LSB/°/s）
  * @retval HAL_OK: 成功
  * @note   拧腕旋转检测用 gz（Z 轴角速度），正负号代表旋转方向
  */
HAL_StatusTypeDef MPU6500_ReadGyro(I2C_HandleTypeDef *hi2c,
                                   int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t raw[6];  /* GYRO_XOUT_H 开始的 6 字节 */
    HAL_StatusTypeDef status;

    status = MPU6500_ReadRegs(hi2c, MPU6500_REG_GYRO_XOUT_H, raw, 6);
    if (status != HAL_OK)
    {
        return status;
    }

    /* 大端字节序 → int16_t */
    *gx = (int16_t)((raw[0] << 8) | raw[1]);
    *gy = (int16_t)((raw[2] << 8) | raw[3]);
    *gz = (int16_t)((raw[4] << 8) | raw[5]);

    return HAL_OK;
}
