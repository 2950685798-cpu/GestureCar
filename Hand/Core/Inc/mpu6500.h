#ifndef __MPU6500_H
#define __MPU6500_H

#include "main.h"

/* MPU6500 I2C 从机地址（7位地址 0x68 左移1位，适配 HAL 库）*/
#define MPU6500_ADDR            0xD0

/* MPU6500 寄存器地址 */
#define MPU6500_REG_WHO_AM_I    0x75    /* 设备ID寄存器，MPU6500返回0x70 */
#define MPU6500_REG_PWR_MGMT_1  0x6B    /* 电源管理，bit6=1时处于睡眠 */
#define MPU6500_REG_SMPLRT_DIV  0x19    /* 采样率分频器 */
#define MPU6500_REG_CONFIG      0x1A    /* 低通滤波器配置 */
#define MPU6500_REG_GYRO_CONFIG 0x1B    /* 陀螺仪量程配置 */
#define MPU6500_REG_ACCEL_CONFIG 0x1C   /* 加速度计量程配置 */
#define MPU6500_REG_ACCEL_XOUT_H 0x3B   /* 加速度X高字节（共6字节：X/Y/Z各2字节）*/

#define MPU6500_REG_GYRO_XOUT_H  0x43   /* 陀螺仪X高字节（共6字节：X/Y/Z各2字节）*/

/* 函数声明 */
HAL_StatusTypeDef MPU6500_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6500_ReadAccel(I2C_HandleTypeDef *hi2c,
                                    int16_t *ax, int16_t *ay, int16_t *az);

/* 陀螺仪读取 */
HAL_StatusTypeDef MPU6500_ReadGyro(I2C_HandleTypeDef *hi2c,
                                   int16_t *gx, int16_t *gy, int16_t *gz);
void MPU6500_CalcAngles(int16_t ax, int16_t ay, int16_t az,
                        float *pitch, float *roll);

#endif /* __MPU6500_H */
