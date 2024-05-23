CC=arm-none-eabi-gcc

SOURCES=$(wildcard ./*.c) $(wildcard ./Drivers/Src/*.c)
OBJECTS=$(SOURCES:.c=.o)
LDFLAGS=-mcpu=cortex-m4 -T./linker-script.ld --specs=nosys.specs \
	-static --specs=nano.specs \
	-mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb \
	-u _printf_float -lc -lm -fsingle-precision-constant
CFLAGS=-mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F401xC -c \
       -I. -I./Drivers/Inc/Legacy -I./Drivers/Inc \
       -I./Drivers/CMSIS/Device/ST/STM32F4xx/Include \
       -I./Drivers/CMSIS/Include -Os -Wall --specs=nano.specs \
       -mfpu=fpv4-sp-d16 -mfloat-abi=hard -u _printf_float -mthumb -lm \
       -fsingle-precision-constant -Wdouble-promotion
SFLAGS=-mcpu=cortex-m4 -c -x assembler-with-cpp --specs=nano.specs \
       -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb \
       -fsingle-precision-constant

all: load

load: prog.elf
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
		-c "program prog.elf verify reset exit"

prog.elf: $(OBJECTS) ./startup.o
	$(CC) $(LDFLAGS) $(OBJECTS) ./startup.o -o $@

.s.o:
	$(CC) $(SFLAGS) $< -o $@

.c.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm $(OBJECTS) ./startup.o
