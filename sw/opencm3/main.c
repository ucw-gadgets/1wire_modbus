#include "libopencm3/cm3/common.h"
#include "libopencm3/stm32/g0/gpio.h"
#include "libopencm3/stm32/g0/rcc.h"
#include "libopencm3/cm3/systick.h"
#include "libopencm3/stm32/usart.h"
#include "libopencm3/stm32/i2c.h"
#include "libopencm3/cm3/nvic.h"
#include "libopencm3/stm32/adc.h"
#include "libopencm3/stm32/dma.h"
#include "libopencm3/stm32/dmamux.h"
#include "libopencm3/stm32/memorymap.h"

#include "config.h"

#include "stm32lib/lib/util.h"
#include "stm32lib/lib/ds18b20.h"

#include "sht30.h"
#include "modbus.h"
#include "my_adc.h"
#include "onewire.h"

/*** System ticks ***/

volatile u32 ms_ticks;

unsigned int ow_n_therm;
uint16_t ow_addr1[DS_NUM_SENSORS];
uint16_t ow_addr2[DS_NUM_SENSORS];
uint16_t ow_addr3[DS_NUM_SENSORS];
uint16_t ow_temp[DS_NUM_SENSORS];

volatile uint16_t adc_res[ADC_NCHANNELS];
volatile uint32_t n_dmaint;

void sys_tick_handler(void)
{
	ms_ticks++;
}

static void tick_init(void)
{
	systick_set_frequency(1000, CPU_CLOCK_MHZ * 1000000);
	systick_counter_enable();
	systick_interrupt_enable();
}

static void delay_ms(uint ms)
{
	u32 start_ticks = ms_ticks;
	while (ms_ticks - start_ticks < ms)
		;
}

void adc_setup(void)
{
	gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO7);

	adc_power_off(ADC1);
	adc_set_clk_prescale(ADC1, ADC_CCR_PRESC_DIV2);
	adc_set_single_conversion_mode(ADC1);
	adc_set_right_aligned(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_160DOT5);
	adc_enable_temperature_sensor();
	adc_enable_vrefint();

	adc_calibrate(ADC1);

	uint8_t channel_array[ADC_NCHANNELS] = {0};
	channel_array[0] = 11;
	channel_array[1] = ADC_CHANNEL_TEMP;
	channel_array[2] = ADC_CHANNEL_VREF;
	adc_set_regular_sequence(ADC1, ADC_NCHANNELS, channel_array);
	adc_enable_dma_circular_mode(ADC1);
	adc_set_resolution(ADC1, ADC_CFGR1_RES_12_BIT);

	// ADC DMA start
	adc_set_continuous_conversion_mode(ADC1);
	// adc_set_operation_mode(ADC1,ADC_MODE_SCAN_INFINITE);

	//adc_disable_discontinuous_mode(ADC1);
	adc_enable_eoc_interrupt(ADC1);
	nvic_enable_irq(NVIC_ADC_COMP_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);

	adc_enable_dma_circular_mode(ADC1);
	adc_enable_dma(ADC1);
	// ADC DMA end

	adc_power_on(ADC1);
	delay_ms(10);
	// ADC DMA start
	adc_start_conversion_regular(ADC1);
	return;
}
void adc_comp_isr() {
	ADC_ISR(ADC1) = ADC_ISR_EOC;
	n_dmaint++;
}

static void dma_setup(void)
{
	adc_power_off(ADC1);
	dma_channel_reset(DMA1, DMA_CHANNEL1);

	dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&ADC_DR(ADC1));
	dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)&adc_res);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
	dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);
	dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);
	dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_LOW);

	dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);
	dma_set_number_of_data(DMA1, DMA_CHANNEL1, ADC_NCHANNELS);
	dma_enable_circular_mode(DMA1, DMA_CHANNEL1);
	dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);

	//	dma_enable_mem2mem_mode(DMA1, DMA_CHANNEL1);
	//	dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL1);

	dma_enable_channel(DMA1, DMA_CHANNEL1);
	//	adc_enable_dma(DMA1);

	dmamux_reset_dma_channel(DMAMUX1, DMA_CHANNEL1);
	dmamux_set_dma_channel_request(DMAMUX1, DMA_CHANNEL1, DMAMUX_CxCR_DMAREQ_ID_ADC);
}

void dma1_channel1_isr(void)
{
	dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_IFCR_CGIF1);
	//n_dmaint++;
}
static uint16_t calculate_temperature(uint16_t vcc, uint16_t temp_val){
	int32_t tmp;
	tmp = (uint32_t)temp_val*vcc/4096;	//temp val in mV
	temp_val = (tmp - (ST_TSENSE_CAL1_30C*3000/4096))*2/5+30;
	return temp_val;
	//return tmp;
}

static uint16_t calculate_voltage(uint16_t adc_val){

	//uint16_t vrefint_cal;                        // VREFINT calibration value
	//vrefint_cal= ST_VREFINT_CAL; // read VREFINT_CAL_ADDR memory location
	
	uint16_t voltage;
	voltage = 3000 * ST_VREFINT_CAL/adc_val;
	return voltage;
}



void usart_setup(void)
{
	/* Setup GPIO pin GPIO_USART3_TX and GPIO_USART3_RX. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO2 | GPIO3);

	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);

	/* Setup UART parameters. */
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART2);
}

void i2c_setup(void)
{
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_LOW, GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF6, GPIO11 | GPIO12);

	// rcc_set_i2c_clock_hsi(I2C2);

	rcc_periph_reset_pulse(RST_I2C2);
	i2c_peripheral_disable(I2C2);
	// configure ANFOFF DNF[3:0] in CR1
	i2c_enable_analog_filter(I2C2);
	i2c_set_digital_filter(I2C2, 0);
	/* HSI is at 8Mhz */
	i2c_set_speed(I2C2, i2c_speed_sm_100k, 16);
	// configure No-Stretch CR1 (only relevant in slave mode)
	i2c_disable_stretching(I2C2);
	// addressing mode
	i2c_set_7bit_addr_mode(I2C2);
	i2c_peripheral_enable(I2C2);
}

bool modbus_check_discrete_input(u16 addr)
{
	(void)addr;
	return false;
}
bool modbus_get_discrete_input(u16 addr)
{
	// iwdg_reset();
	(void)addr;
	return false;
}

bool modbus_check_coil(u16 addr)
{
	switch (addr)
	{
	default:
		return false;
	}
}

bool modbus_get_coil(u16 addr)
{
	// iwdg_reset();
	switch (addr)
	{
	default:
		return false;
	}
	return false;
}
void modbus_set_coil(u16 addr, bool value)
{
	// iwdg_reset();
	switch (addr)
	{
	default:
		return;
	}
}

bool modbus_check_input_register(u16 addr)
{
	for (unsigned int i = 0; i < ow_n_therm; i++)
	{
		if (addr == MODBUS_OW_ADDR1_BASE_REG + i * MODBUS_OW_REGS_PER_THERM)
		{
			return true;
		}
		if (addr == MODBUS_OW_ADDR2_BASE_REG + i * MODBUS_OW_REGS_PER_THERM)
		{
			return true;
		}
		if (addr == MODBUS_OW_ADDR3_BASE_REG + i * MODBUS_OW_REGS_PER_THERM)
		{
			return true;
		}
		if (addr == MODBUS_OW_TEMP_BASE_REG + i * MODBUS_OW_REGS_PER_THERM)
		{
			return true;
		}
	}
	switch (addr)
	{
	case MODBUS_VIN_REG:
	case MODBUS_IIN_REG:
	case MODBUS_3V3_REG:
	case MODBUS_TEMPINT_REG:
	case MODBUS_SHT_TEMP_REG:
	case MODBUS_SHT_HUM_REG:
		return true;
	default:
		return false;
	}
}

u16 modbus_get_input_register(u16 addr)
{
	// iwdg_reset();
	// printf("Get input registers: %d\n",addr);
	for (unsigned int i = 0; i < ow_n_therm; i++)
	{
		if (addr == MODBUS_OW_ADDR1_BASE_REG + i * MODBUS_OW_REGS_PER_THERM)
		{
			return ow_addr1[i];
		}
		if (addr == MODBUS_OW_ADDR2_BASE_REG + i * MODBUS_OW_REGS_PER_THERM)
		{
			return ow_addr2[i];
		}
		if (addr == MODBUS_OW_ADDR3_BASE_REG + i * MODBUS_OW_REGS_PER_THERM)
		{
			return ow_addr3[i];
		}
		if (addr == MODBUS_OW_TEMP_BASE_REG + i * MODBUS_OW_REGS_PER_THERM)
		{
			return (uint16_t)ow_temp[i];
		}
	}
	switch (addr)
	{
	case MODBUS_VIN_REG:
		return adc_vin_v;
	case MODBUS_IIN_REG:
		return UINT16_MAX;
	case MODBUS_3V3_REG:
		return adc_3V3_v;
	case MODBUS_TEMPINT_REG:
		return adc_temp_int;
	case MODBUS_SHT_TEMP_REG:
		return sht_temp;
	case MODBUS_SHT_HUM_REG:
		return sht_hum;
	default:
		return UINT16_MAX;
	}
}

bool modbus_check_holding_register(u16 addr)
{
	// printf("Checking holding registers %d\n",addr);
	switch (addr)
	{
	case MODBUS_STATUS_REG:
	case MODBUS_UPTIME_REG:
		return true;
	default:
		return false;
	}
}

u16 modbus_get_holding_register(u16 addr)
{
	// iwdg_reset();
	// debug_printf("Get holding registers %d\n",addr);
	switch (addr)
	{
	case MODBUS_STATUS_REG:
		return UINT16_MAX; // FIXME
	case MODBUS_UPTIME_REG:
		return ms_ticks / 1000;
	default:
		return 0;
	}
}
void modbus_set_holding_register(u16 addr, u16 value)
{
	// iwdg_reset();
	// debug_printf("Set holding reg %d to %d\n",addr,value);
	switch (addr)
	{
	case MODBUS_STATUS_REG:
		break;
	case MODBUS_UPTIME_REG:
		break;
	default:
		break;
	}
}

void modbus_ready_hook(void) {}
void modbus_frame_start_hook(void) {}

const char *const modbus_id_strings[] = {
    "Jethro",
    "1Wire_ModBus",
    "0.1"};

int main(void)
{
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_I2C2);
	rcc_periph_clock_enable(RCC_TIM3);
	rcc_periph_clock_enable(RCC_TIM2);
	rcc_periph_clock_enable(RCC_TIM1);
	rcc_periph_clock_enable(RCC_DMA1);
	rcc_periph_clock_enable(RCC_ADC);

	rcc_periph_reset_pulse(RST_GPIOA);
	rcc_periph_reset_pulse(RST_GPIOB);
	rcc_periph_reset_pulse(RST_USART2);
	rcc_periph_reset_pulse(RST_I2C2);
	rcc_periph_reset_pulse(RST_TIM1);
	rcc_periph_reset_pulse(RST_ADC);
	rcc_periph_reset_pulse(RST_DMA1);

	// gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, 0);

	tick_init();

	// usart_setup();
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO2 | GPIO3);

	i2c_setup();
	sht_init();

	dma_setup();
	adc_setup();

	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3);

	modbus_init();

	uint32_t last_blink;
	last_blink = ms_ticks;

	uint32_t last_sht_read;
	last_sht_read = ms_ticks;

	uint32_t last_ow_read;
	last_ow_read = ms_ticks;

	uint8_t ow_addr[4][8];
	uint8_t n_addr_total;
	n_addr_total = 0;
	uint8_t n_addr;
	n_addr = 0;

	int16_t temperature;

	ow_init();
	ow_init_pin(DS1_GPIO, DS1_PIN);
	ow_init_pin(DS2_GPIO, DS2_PIN);

	while (1)
	{
		adc_3V3_v = calculate_voltage(adc_res[2]);
		adc_temp_int = calculate_temperature(adc_3V3_v, adc_res[1]);
		adc_vin_v = adc_res[0]*adc_3V3_v/4096*(470+47)/47;
		if (ms_ticks - last_blink > 500)
		{
			gpio_toggle(GPIOB, GPIO3);
			last_blink = ms_ticks;
		}
		if (ms_ticks - last_sht_read > 2000)
		{
			sht_read();
			debug_printf("temp: %d, hum: %d \n", sht_temp, sht_hum);
			last_sht_read = ms_ticks;
		}
		if (ms_ticks - last_ow_read > 1000)
		{
			n_addr = 0;
			n_addr_total = 0;
			ow_set_pin(DS1_GPIO, DS1_PIN);
			ow_reset_search();
			while (ow_search(ow_addr[n_addr], 1))
			{
				n_addr++;
			}
			for (int i = 0; i < n_addr; i++)
			{

				ow_set_precision(ow_addr[i], (1 << 5) | (1 << 6));
				ow_start_conversion(ow_addr[i]);
			}
			if (n_addr > 0)
			{
				uint32_t start_conversion_time = ms_ticks;
				while (ms_ticks - start_conversion_time < 1000)
				{
					modbus_loop();
				}
			}
			for (int i = 0; i < n_addr; i++)
			{
				temperature = ow_read_temperature(ow_addr[i]);
				if (temperature < -12000)
				{
					temperature = ow_read_temperature(ow_addr[i]);
				}
				ow_addr1[i] = ow_addr[i][1] << 8 | ow_addr[i][2];
				ow_addr2[i] = ow_addr[i][3] << 8 | ow_addr[i][4];
				ow_addr3[i] = ow_addr[i][5] << 8 | ow_addr[i][6];
				ow_temp[i] = temperature;

				// printf("Temperature: %f\n", temperature);
				delay_ms(1);
			}
			n_addr_total = n_addr;

			n_addr = 0;
			for (int i = 0; i < 8; i++)
			{
				ow_addr[0][i] = 0xEE;
				temperature = 0;
			}
			ow_set_pin(DS2_GPIO, DS2_PIN);
			ow_reset_search();
			while (ow_search(ow_addr[n_addr], 1))
			{
				n_addr++;
			}
			for (int i = 0; i < n_addr; i++)
			{

				ow_set_precision(ow_addr[i], (1 << 5) | (1 << 6));
				ow_start_conversion(ow_addr[i]);
			}
			if (n_addr > 0)
			{
				uint32_t start_conversion_time = ms_ticks;
				while (ms_ticks - start_conversion_time < 1000)
				{
					modbus_loop();
				}
			}
			for (int i = 0; i < n_addr; i++)
			{
				temperature = ow_read_temperature(ow_addr[i]);
				if (temperature < -12000)
				{
					temperature = ow_read_temperature(ow_addr[i]);
				}
				ow_addr1[n_addr_total + i] = ow_addr[i][1] << 8 | ow_addr[i][2];
				ow_addr2[n_addr_total + i] = ow_addr[i][3] << 8 | ow_addr[i][4];
				ow_addr3[n_addr_total + i] = ow_addr[i][5] << 8 | ow_addr[i][6];
				ow_temp[n_addr_total + i] = temperature;

				// printf("Temperature: %f\n", temperature);
				delay_ms(1);
			}
			n_addr_total += n_addr;
			ow_n_therm = n_addr_total;

			last_ow_read = ms_ticks;
		}

		modbus_loop();
	}
}
