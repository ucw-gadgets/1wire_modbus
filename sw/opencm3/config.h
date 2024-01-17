#ifndef CONFIG_H
#define CONFIG_H

#define CPU_CLOCK_MHZ 16

//#define DEBUG_USART USART2

//#define DS_DEBUG
//#define DS_DEBUG2

#define DS_TIMER TIM3
#define DS_GPIO GPIOA
#define DS_PIN GPIO7
#define DS_DMA DMA1
#define DS_DMA_CH 6

#define MODBUS_STATUS_REG	1
#define MODBUS_UPTIME_REG	2
#define MODBUS_VIN_REG	5
#define MODBUS_IIN_REG	6
#define MODBUS_3V3_REG	7
#define MODBUS_TEMPINT_REG	8
#define MODBUS_SHT_TEMP_REG	10
#define MODBUS_SHT_HUM_REG	11

#define MODBUS_OW_REGS_PER_THERM 4
#define MODBUS_OW_ADDR1_BASE_REG	20
#define MODBUS_OW_ADDR2_BASE_REG	21
#define MODBUS_OW_ADDR3_BASE_REG	22
#define MODBUS_OW_TEMP_BASE_REG	23


// USART (pins are expected to be configured by the caller)
#define MODBUS_USART USART2
#define MODBUS_NVIC_USART_IRQ NVIC_USART2_LPUART2_IRQ
#define MODBUS_USART_ISR usart2_lpuart2_isr

// GPIO pin for transmitter enable (pins is expected to be configured by the caller)
#define MODBUS_TXEN_GPIO_PORT GPIOA
#define MODBUS_TXEN_GPIO_PIN GPIO1
#define MODBUS_RXEN_GPIO_PORT GPIOA
#define MODBUS_RXEN_GPIO_PIN GPIO0

// Timer TODO zmenit na existujici
#define MODBUS_TIMER TIM2
#define MODBUS_NVIC_TIMER_IRQ NVIC_TIM2_IRQ
#define MODBUS_TIMER_ISR tim2_isr

// Slave address we are responding at
#define MODBUS_OUR_ADDRESS 42

// Maximum number of supported sensors
#define DS_NUM_SENSORS 8

#endif