STM32LIB=stm32lib
OPENCM3_DIR=libopencm3
BINARY=1wire_modbus
OBJS=main.o
LIB_OBJS=util-debug.o sht30.o modbus.o my_adc.o onewire.o
#LIB_OBJS=util-debug.o modbus.o

#WITH_BOOT_LOADER=0
#WITH_MODBUS_FLASH=0

MODBUS_FLASH_ARGS=--rtu /dev/ttyUSB2 --baud 19200 --addr 42

include $(STM32LIB)/mk/g030.mk
