DIR := $(dir $(lastword $(MAKEFILE_LIST)))

OBJS += \
	$(DIR)/src/usart.o \
	$(DIR)/src/printf.o \
	$(DIR)/src/packet.o \
	$(DIR)/src/crc.o \
	$(DIR)/src/i2cbb.o \
	$(DIR)/src/fix32_div.o \
	$(DIR)/src/fft.o
	
INCS += -I$(DIR)/include
