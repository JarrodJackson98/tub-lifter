/*
 * I2C-to-CAN Bridge Firmware for Sequent Microsystems MEGA-IND V2 Hat
 * STM32F072V8T6 - Replaces stock firmware with a CAN-only bridge.
 *
 * Hardware (from schematic):
 *   CAN_TX  = PB9  (AF4) -> MCP2551 TxD via R26
 *   CAN_RX  = PB8  (AF4) -> MCP2551 RxD via R14
 *   I2C_SCL = PB6  (AF1) -> RPi SCL (GPIO3)
 *   I2C_SDA = PB7  (AF1) -> RPi SDA (GPIO2)
 *   ID1     = PB12 (input, active low via jumper J101)
 *   ID2     = PB13
 *   ID3     = PB14
 *   LED     = PA0  (active high, accent LED)
 *
 * I2C slave address: 0x48 + jumper offset (0-7)
 *
 * Register map (smbus block read/write):
 * -----------------------------------------------
 * WRITE registers (RPi -> STM32 -> CAN):
 *   0x00  TX_SEND: Write 14 bytes to send a CAN frame immediately:
 *         [ID3 ID2 ID1 ID0] [FLAGS] [DLC] [D0..D7]
 *         FLAGS bit 0: 1=extended 29-bit ID, 0=standard 11-bit
 *
 * READ registers (CAN -> STM32 -> RPi):
 *   0x10  RX_FRAME: Read 15 bytes of last received CAN frame:
 *         [STATUS] [ID3 ID2 ID1 ID0] [FLAGS] [DLC] [D0..D7]
 *         STATUS bit 0: frame valid, bit 1: overrun
 *
 *   0x20  INFO: Read 2 bytes: [FW_VERSION] [CAN_STATUS]
 *         CAN_STATUS bit 0: bus-off, bit 1: error passive
 *
 * WRITE registers (special):
 *   0xF0  ENTER_BOOTLOADER: Write magic byte 0xBE to jump to the STM32
 *         ROM bootloader (I2C address changes from 0x48 to 0x56).
 *         No BOOT0 bridge or power cycle needed.
 */

#include "stm32f0xx_hal.h"
#include <string.h>

#define FW_VERSION       0x02
#define I2C_BASE_ADDR    0x48
#define CAN_BITRATE_500K 1
#define BOOTLOADER_MAGIC 0xBE  /* Magic byte to trigger bootloader entry */
#define SYSTEM_MEMORY    0x1FFFC800  /* STM32F072 ROM bootloader base */

/* ---- Register file shared between I2C ISR and main loop ---- */

/* TX buffer: written by I2C master, consumed by main loop */
static volatile struct {
    uint32_t id;
    uint8_t  flags;
    uint8_t  dlc;
    uint8_t  data[8];
    uint8_t  pending;  /* set by I2C ISR, cleared by main after CAN send */
} tx_buf;

/* RX buffer: written by CAN RX ISR, read by I2C master */
static volatile struct {
    uint32_t id;
    uint8_t  flags;
    uint8_t  dlc;
    uint8_t  data[8];
    uint8_t  valid;
    uint8_t  overrun;
} rx_buf;

/* Flag to enter bootloader from main loop */
static volatile uint8_t enter_bootloader = 0;

/* ---- Peripheral handles ---- */
static CAN_HandleTypeDef hcan;
static I2C_HandleTypeDef hi2c1;

/* I2C transfer state */
static uint8_t i2c_reg_addr;
static uint8_t i2c_rx_buf[16];
static uint8_t i2c_tx_buf[16];

/* ---- Forward declarations ---- */
static void SystemClock_Config(void);
static void GPIO_Init(void);
static void CAN_Init(void);
static void I2C_Init(void);
static uint8_t ReadIDJumpers(void);
static void LED_Set(int on);
static void CAN_SendFrame(void);
static void PrepareRxResponse(void);
static void JumpToBootloader(void);

/* ================================================================== */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    CAN_Init();
    I2C_Init();

    /* Start listening for I2C transactions */
    HAL_I2C_EnableListen_IT(&hi2c1);

    /* Start CAN reception (FIFO0, interrupt) */
    CAN_FilterTypeDef filter = {0};
    filter.FilterMode       = CAN_FILTERMODE_IDMASK;
    filter.FilterScale      = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh     = 0;
    filter.FilterIdLow      = 0;
    filter.FilterMaskIdHigh = 0;
    filter.FilterMaskIdLow  = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.FilterBank       = 0;
    HAL_CAN_ConfigFilter(&hcan, &filter);
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    LED_Set(1);  /* LED on = firmware running */

    while (1) {
        /* Jump to ROM bootloader if requested via I2C register 0xF0 */
        if (enter_bootloader) {
            JumpToBootloader();
        }

        /* Send CAN frame if I2C master wrote one */
        if (tx_buf.pending) {
            CAN_SendFrame();
            tx_buf.pending = 0;
        }
    }
}

/* ---- Clock: HSI48 -> 48 MHz system clock ---- */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    osc.HSI48State     = RCC_HSI48_ON;
    HAL_RCC_OscConfig(&osc);

    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                          RCC_CLOCKTYPE_PCLK1;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI48;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1);
}

/* ---- GPIO ---- */
static void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* LED on PA0 (active high) */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_0;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* ID jumpers PB12, PB13, PB14 (active low, external pull-up) */
    gpio.Pin  = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static uint8_t ReadIDJumpers(void)
{
    uint8_t id = 0;
    /* Jumpers short pin to GND when installed (active low) */
    if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)) id |= 1;
    if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) id |= 2;
    if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)) id |= 4;
    return id;
}

static void LED_Set(int on)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ---- CAN: 500 kbit/s on PB8 (RX) / PB9 (TX) ---- */
static void CAN_Init(void)
{
    __HAL_RCC_CAN1_CLK_ENABLE();

    /* Configure PB8 = CAN_RX, PB9 = CAN_TX (AF4) */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF4_CAN;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* CAN peripheral config
     * APB1 = 48 MHz
     * Prescaler=6, BS1=13, BS2=2 -> 48/(6*(1+13+2)) = 500 kbit/s
     */
    hcan.Instance = CAN;
    hcan.Init.Prescaler     = 6;
    hcan.Init.Mode          = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1      = CAN_BS1_13TQ;
    hcan.Init.TimeSeg2      = CAN_BS2_2TQ;
    hcan.Init.TimeTriggeredMode   = DISABLE;
    hcan.Init.AutoBusOff          = ENABLE;
    hcan.Init.AutoWakeUp          = ENABLE;
    hcan.Init.AutoRetransmission  = ENABLE;
    hcan.Init.ReceiveFifoLocked   = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    HAL_CAN_Init(&hcan);

    /* Enable CAN RX interrupt */
    HAL_NVIC_SetPriority(CEC_CAN_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CEC_CAN_IRQn);
}

/* Send the frame currently in tx_buf */
static void CAN_SendFrame(void)
{
    CAN_TxHeaderTypeDef header = {0};

    if (tx_buf.flags & 0x01) {
        header.IDE   = CAN_ID_EXT;
        header.ExtId = tx_buf.id;
    } else {
        header.IDE   = CAN_ID_STD;
        header.StdId = tx_buf.id & 0x7FF;
    }
    header.RTR = CAN_RTR_DATA;
    header.DLC = tx_buf.dlc;

    uint8_t data[8];
    memcpy(data, (const void *)tx_buf.data, 8);

    uint32_t mailbox;
    HAL_CAN_AddTxMessage(&hcan, &header, data, &mailbox);

    /* Toggle LED on each TX */
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
}

/* ---- Jump to ROM bootloader (no BOOT0 bridge needed) ---- */
static void JumpToBootloader(void)
{
    /* Disable all interrupts */
    __disable_irq();

    /* Deinit all peripherals to reset them to default state */
    HAL_CAN_Stop(&hcan);
    HAL_CAN_DeInit(&hcan);
    HAL_I2C_DeInit(&hi2c1);
    HAL_DeInit();

    /* Disable SysTick */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    /* Clear all pending interrupts (Cortex-M0 has 1 ICER/ICPR register) */
    NVIC->ICER[0] = 0xFFFFFFFF;
    NVIC->ICPR[0] = 0xFFFFFFFF;

    /* Remap system memory to address 0x00000000 */
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

    /* Set MSP to the bootloader's stack pointer (first word of system memory) */
    __set_MSP(*(volatile uint32_t *)SYSTEM_MEMORY);

    /* Jump to the bootloader reset handler (second word of system memory) */
    void (*bootloader)(void) = (void (*)(void))(*(volatile uint32_t *)(SYSTEM_MEMORY + 4));

    __enable_irq();
    bootloader();

    /* Should never reach here */
    while (1);
}

/* ---- CAN RX interrupt callback ---- */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if (HAL_CAN_GetRxMessage(h, CAN_RX_FIFO0, &header, data) == HAL_OK) {
        if (rx_buf.valid) {
            rx_buf.overrun = 1;
        }
        if (header.IDE == CAN_ID_EXT) {
            rx_buf.id    = header.ExtId;
            rx_buf.flags = 0x01;
        } else {
            rx_buf.id    = header.StdId;
            rx_buf.flags = 0x00;
        }
        rx_buf.dlc = header.DLC;
        memcpy((void *)rx_buf.data, data, 8);
        rx_buf.valid = 1;
    }
}

void CEC_CAN_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan);
}

/* ---- I2C slave: address 0x48 + jumper offset ---- */
static void I2C_Init(void)
{
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* Configure PB6 = SCL, PB7 = SDA (AF1, open-drain) */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode      = GPIO_MODE_AF_OD;
    gpio.Pull      = GPIO_NOPULL;  /* External pull-ups on the hat */
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF1_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);

    uint8_t addr = I2C_BASE_ADDR + ReadIDJumpers();

    hi2c1.Instance             = I2C1;
    hi2c1.Init.Timing          = 0x20303E5D;  /* 400 kHz fast mode @ 48 MHz */
    hi2c1.Init.OwnAddress1     = addr << 1;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);

    HAL_NVIC_SetPriority(I2C1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_IRQn);
}

/* ---- I2C slave callbacks ---- */

/* Called when master addresses us - determine direction */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t dir, uint16_t addrMatchCode)
{
    (void)addrMatchCode;

    if (dir == I2C_DIRECTION_TRANSMIT) {
        /* Master is writing to us: first byte = register address */
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, i2c_rx_buf, 1, I2C_FIRST_FRAME);
    } else {
        /* Master is reading: respond based on last written register address */
        PrepareRxResponse();
        HAL_I2C_Slave_Seq_Transmit_IT(hi2c, i2c_tx_buf, sizeof(i2c_tx_buf),
                                       I2C_LAST_FRAME);
    }
}

/* Called when we received data from master */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    static uint8_t phase = 0;  /* 0 = got reg addr, 1 = got data */

    if (phase == 0) {
        /* First byte received = register address */
        i2c_reg_addr = i2c_rx_buf[0];
        phase = 1;
        /* Continue receiving the rest of the write (up to 14 bytes for TX_SEND) */
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, i2c_rx_buf + 1, 14, I2C_LAST_FRAME);
    } else {
        /* Data received - process based on register address */
        phase = 0;

        if (i2c_reg_addr == 0x00) {
            /* TX_SEND: [ID3 ID2 ID1 ID0] [FLAGS] [DLC] [D0..D7] */
            tx_buf.id = ((uint32_t)i2c_rx_buf[1] << 24) |
                        ((uint32_t)i2c_rx_buf[2] << 16) |
                        ((uint32_t)i2c_rx_buf[3] << 8)  |
                        (uint32_t)i2c_rx_buf[4];
            tx_buf.flags = i2c_rx_buf[5];
            tx_buf.dlc   = i2c_rx_buf[6];
            if (tx_buf.dlc > 8) tx_buf.dlc = 8;
            memcpy((void *)tx_buf.data, (const void *)&i2c_rx_buf[7], 8);
            tx_buf.pending = 1;
        } else if (i2c_reg_addr == 0xF0 && i2c_rx_buf[1] == BOOTLOADER_MAGIC) {
            /* ENTER_BOOTLOADER: jump to ROM bootloader (handled in main loop) */
            enter_bootloader = 1;
        }
    }
}

/* Called when master finished reading from us */
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    (void)hi2c;
    /* If master read the RX frame, clear valid flag */
    if (i2c_reg_addr == 0x10) {
        rx_buf.valid   = 0;
        rx_buf.overrun = 0;
    }
}

/* Called on STOP condition */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    /* Re-enable listen mode for next transaction */
    HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    /* Re-enable listen mode on error */
    HAL_I2C_EnableListen_IT(hi2c);
}

/* Prepare response buffer for read transactions */
static void PrepareRxResponse(void)
{
    memset(i2c_tx_buf, 0, sizeof(i2c_tx_buf));

    if (i2c_reg_addr == 0x10) {
        /* RX_FRAME: [STATUS] [ID3..ID0] [FLAGS] [DLC] [D0..D7] */
        i2c_tx_buf[0] = (rx_buf.valid ? 0x01 : 0x00) |
                         (rx_buf.overrun ? 0x02 : 0x00);
        i2c_tx_buf[1] = (rx_buf.id >> 24) & 0xFF;
        i2c_tx_buf[2] = (rx_buf.id >> 16) & 0xFF;
        i2c_tx_buf[3] = (rx_buf.id >> 8)  & 0xFF;
        i2c_tx_buf[4] = rx_buf.id & 0xFF;
        i2c_tx_buf[5] = rx_buf.flags;
        i2c_tx_buf[6] = rx_buf.dlc;
        memcpy(&i2c_tx_buf[7], (const void *)rx_buf.data, 8);

    } else if (i2c_reg_addr == 0x20) {
        /* INFO: [FW_VERSION] [CAN_STATUS] */
        uint32_t esr = hcan.Instance->ESR;
        i2c_tx_buf[0] = FW_VERSION;
        i2c_tx_buf[1] = ((esr & CAN_ESR_BOFF) ? 0x01 : 0x00) |
                         ((esr & CAN_ESR_EPVF) ? 0x02 : 0x00);
    }
}

/* I2C1 interrupt handler */
void I2C1_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&hi2c1);
    HAL_I2C_ER_IRQHandler(&hi2c1);
}

/* Required by HAL */
void SysTick_Handler(void)
{
    HAL_IncTick();
}
