/**
 * nRF24L01 无线通信模块驱动 — STM32 HAL 库版
 * 基于江协科技 V1.0 参考代码移植，寄存器参数完全一致
 * 
 * 引脚配置（由 CubeMX MX_GPIO_Init 完成）:
 *   CE  → PA4    GPIO_OUTPUT_PP
 *   CSN → PA5    GPIO_OUTPUT_PP
 *   SCK → PA6    GPIO_OUTPUT_PP
 *   MOSI→ PA7    GPIO_OUTPUT_PP
 *   MISO→ PA12   GPIO_INPUT_PU
 * 2026-09-13 车端 NRF 模块烧坏时把 PA8/PA9 引脚烧成对地短路（PA8 恒低、
 *   PA9 恒低），信号脚整体从 PA8~PA11 挪到 PA4~PA7，MISO 留在 PA12（实测好）。
 *   手端板引脚未坏，仍用 PA8~PA12，两端驱动文件不同步属正常。
 */

#include "nrf24l01.h"
#include "gpio.h"

/* ==================== 寄存器地址 ==================== */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define RX_ADDR_P0  0x0A
#define TX_ADDR     0x10
#define RX_PW_P0    0x11

/* ==================== 指令码 ==================== */
#define R_REGISTER      0x00
#define W_REGISTER      0x20
#define R_RX_PAYLOAD    0x61
#define W_TX_PAYLOAD    0xA0
#define FLUSH_TX        0xE1
#define FLUSH_RX        0xE2
#define NOP             0xFF

/* ==================== 引脚宏（HAL 写法） ==================== */
/* 2026-09-13 PA8/PA9 烧坏后整体挪到 PA4~PA7（见文件头注释） */
#define NRF_CE(x)       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, x)
#define NRF_CSN(x)      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, x)
#define NRF_SCK(x)      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, x)
#define NRF_MOSI(x)     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, x)
#define NRF_MISO()      HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)

/* ==================== 全局变量 ==================== */
uint8_t NRF24L01_TxPacket[4];   // 发送缓冲区
uint8_t NRF24L01_RxPacket[4];   // 接收缓冲区

static const uint8_t NRF24L01_TxAddress[5] = {0x11, 0x22, 0x33, 0x44, 0x55};
static const uint8_t NRF24L01_RxAddress[5] = {0x11, 0x22, 0x33, 0x44, 0x55};

/* ==================== 底层 SPI 操作 ==================== */

/**
 * SPI 交换一个字节（软件模拟，模式 0：CPOL=0, CPHA=0）
 */
static uint8_t SPI_SwapByte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        /* 高位先行，输出 MOSI */
        NRF_MOSI((byte & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        byte <<= 1;

        /* SCK 上升沿 → 从机采样 */
        NRF_SCK(GPIO_PIN_SET);

        /* 读取 MISO */
        if (NRF_MISO() == GPIO_PIN_SET)
            byte |= 0x01;

        /* SCK 下降沿 */
        NRF_SCK(GPIO_PIN_RESET);
    }
    return byte;
}

/* ==================== 寄存器读写 ==================== */

static void WriteReg(uint8_t reg, uint8_t data)
{
    NRF_CSN(GPIO_PIN_RESET);
    SPI_SwapByte(W_REGISTER | reg);
    SPI_SwapByte(data);
    NRF_CSN(GPIO_PIN_SET);
}

static uint8_t ReadReg(uint8_t reg)
{
    NRF_CSN(GPIO_PIN_RESET);
    SPI_SwapByte(R_REGISTER | reg);
    uint8_t val = SPI_SwapByte(NOP);
    NRF_CSN(GPIO_PIN_SET);
    return val;
}

static void WriteRegs(uint8_t reg, const uint8_t *data, uint8_t len)
{
    NRF_CSN(GPIO_PIN_RESET);
    SPI_SwapByte(W_REGISTER | reg);
    for (uint8_t i = 0; i < len; i++)
        SPI_SwapByte(data[i]);
    NRF_CSN(GPIO_PIN_SET);
}

static uint8_t ReadStatus(void)
{
    NRF_CSN(GPIO_PIN_RESET);
    uint8_t status = SPI_SwapByte(NOP);
    NRF_CSN(GPIO_PIN_SET);
    return status;
}

/* ==================== TX/RX 有效载荷 ==================== */

static void WriteTxPayload(const uint8_t *data, uint8_t len)
{
    NRF_CSN(GPIO_PIN_RESET);
    SPI_SwapByte(W_TX_PAYLOAD);
    for (uint8_t i = 0; i < len; i++)
        SPI_SwapByte(data[i]);
    NRF_CSN(GPIO_PIN_SET);
}

static void ReadRxPayload(uint8_t *buf, uint8_t len)
{
    NRF_CSN(GPIO_PIN_RESET);
    SPI_SwapByte(R_RX_PAYLOAD);
    for (uint8_t i = 0; i < len; i++)
        buf[i] = SPI_SwapByte(NOP);
    NRF_CSN(GPIO_PIN_SET);
}

/* ==================== 模式切换 ==================== */

static void EnterRxMode(void)
{
    NRF_CE(GPIO_PIN_RESET);
    uint8_t cfg = ReadReg(CONFIG);
    if (cfg == 0xFF) return;
    WriteReg(CONFIG, cfg | 0x03);   // PWR_UP=1, PRIM_RX=1
    NRF_CE(GPIO_PIN_SET);
}

static void EnterTxMode(void)
{
    NRF_CE(GPIO_PIN_RESET);
    uint8_t cfg = ReadReg(CONFIG);
    if (cfg == 0xFF) return;
    WriteReg(CONFIG, (cfg | 0x02) & ~0x01);  // PWR_UP=1, PRIM_RX=0
    NRF_CE(GPIO_PIN_SET);
}

/* ==================== 初始化 ==================== */

void NRF24L01_Init(void)
{
    /* MX_GPIO_Init() 已在 main 中调用，此处不再重复初始化引脚 */

    /* 配置寄存器（和参考代码完全一致） */
    WriteReg(CONFIG,     0x08);   // CRC使能(1字节), PWR_UP=0
    WriteReg(EN_AA,      0x3F);   // 全通道自动应答
    WriteReg(EN_RXADDR,  0x01);   // 只开接收通道0
    WriteReg(SETUP_AW,   0x03);   // 地址宽度5字节
    WriteReg(SETUP_RETR, 0x03);   // 重发间隔250us, 重试3次
    WriteReg(RF_CH,      0x02);   // 射频通道 2402MHz
    WriteReg(RF_SETUP,   0x0E);   // 速率2Mbps, 功率0dBm

    /* 接收通道0数据包宽度 */
    WriteReg(RX_PW_P0, 4);

    /* 接收通道0地址 */
    WriteRegs(RX_ADDR_P0, NRF24L01_RxAddress, 5);

    /* 清空 FIFO */
    NRF_CSN(GPIO_PIN_RESET); SPI_SwapByte(FLUSH_TX); NRF_CSN(GPIO_PIN_SET);
    NRF_CSN(GPIO_PIN_RESET); SPI_SwapByte(FLUSH_RX); NRF_CSN(GPIO_PIN_SET);

    /* 清状态标志 */
    WriteReg(STATUS, 0x70);

    /* 默认进入接收模式 */
    EnterRxMode();
}

/**
 * 上电自检：对 RF_CH 寄存器「写入-回读-恢复」，判断模块与 SPI 链路是否正常
 * @return 回读值：0x5A=自检通过；0xFF=模块没响应（接线/供电/模块坏）；
 *         0x00=数据线短路；其他值=写命令没生效
 * @note   2026-09-13 车端 NRF 烧坏换新后 LINK 一直 LOST，加此诊断；
 *         只读写 RF_CH，不影响收发配置，调用后无需重新初始化
 */
uint8_t NRF24L01_SelfCheck(void)
{
    uint8_t orig = ReadReg(RF_CH);
    uint8_t back;

    WriteReg(RF_CH, 0x5A);          /* 写测试值 */
    back = ReadReg(RF_CH);          /* 回读验证 */
    WriteReg(RF_CH, orig);          /* 恢复原值（0x02），不影响正常收发 */

    return back;
}

/**
 * 读取 CONFIG 寄存器（诊断用）
 * @return 正常应为 0x0B：CRC 1 字节(bit3) + 上电(bit1) + 接收模式(bit0)
 * @note   2026-09-13 链路排查：模块 SPI 通但不回 ACK 时，
 *         用它判断模块是否真的进入了接收状态
 */
uint8_t NRF24L01_GetConfig(void)
{
    return ReadReg(CONFIG);
}

/* ==================== 发送 ==================== */

/**
 * 发送数据包
 * @return 1=成功, 2=无ACK, 3=状态异常, 4=超时
 */
uint8_t NRF24L01_Send(void)
{
    uint8_t status;
    uint8_t flag = 0;
    uint32_t timeout = 10000;

    /* 设置发送地址（接收通道0也设成发送地址用于收ACK） */
    WriteRegs(TX_ADDR, NRF24L01_TxAddress, 5);
    WriteRegs(RX_ADDR_P0, NRF24L01_TxAddress, 5);

    /* 写Tx有效载荷 */
    WriteTxPayload(NRF24L01_TxPacket, 4);

    /* 进入发送模式 */
    EnterTxMode();

    /* 等待发送完成 */
    while (1)
    {
        status = ReadStatus();
        timeout--;

        if (timeout == 0)
        {
            flag = 4;               // 超时
            NRF24L01_Init();
            break;
        }

        if ((status & 0x30) == 0x30)     // MAX_RT + TX_DS 同时为1
        {
            flag = 3;                    // 状态异常
            NRF24L01_Init();
            break;
        }
        else if (status & 0x10)          // MAX_RT
        {
            flag = 2;                    // 最大重传，无ACK
            NRF24L01_Init();
            break;
        }
        else if (status & 0x20)          // TX_DS
        {
            flag = 1;                    // 发送成功
            break;
        }
    }

    /* 清状态 + 清FIFO */
    WriteReg(STATUS, 0x30);
    NRF_CSN(GPIO_PIN_RESET); SPI_SwapByte(FLUSH_TX); NRF_CSN(GPIO_PIN_SET);

    /* 恢复接收地址，回到接收模式 */
    WriteRegs(RX_ADDR_P0, NRF24L01_RxAddress, 5);
    EnterRxMode();

    return flag;
}

/* ==================== 接收 ==================== */

/**
 * 接收数据包（非阻塞查询）
 * @return 0=无数据, 1=成功, 2=状态异常, 3=掉电
 */
uint8_t NRF24L01_Receive(void)
{
    uint8_t status = ReadStatus();
    uint8_t config = ReadReg(CONFIG);

    if ((config & 0x02) == 0)            // PWR_UP = 0
    {
        NRF24L01_Init();
        return 3;
    }

    if ((status & 0x30) == 0x30)         // 状态异常
    {
        NRF24L01_Init();
        return 2;
    }

    if (status & 0x40)                   // RX_DR = 1
    {
        ReadRxPayload(NRF24L01_RxPacket, 4);

        /* 清标志 + 清FIFO */
        WriteReg(STATUS, 0x40);
        NRF_CSN(GPIO_PIN_RESET); SPI_SwapByte(FLUSH_RX); NRF_CSN(GPIO_PIN_SET);

        return 1;
    }

    return 0;
}
