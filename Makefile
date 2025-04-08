CC=arm-none-eabi-gcc

SOURCES=$(wildcard ./*.c) $(wildcard ./devices/*.c) \
	$(wildcard ./Drivers/Src/*.c)
OBJECTS=$(SOURCES:.c=.o)
LDFLAGS=-mcpu=cortex-m4 -T./linker-script.ld --specs=nosys.specs \
	-Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm \
	-Wl,--end-group -fsingle-precision-constant -u _printf_float \
	-Wl,-Map=memmap.map
CFLAGS=-mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F405xx -c \
       -I. -I./Drivers/Inc/Legacy -I./Drivers/Inc \
       -I./Drivers/CMSIS/Device/ST/STM32F4xx/Include \
       -I./Drivers/CMSIS/Include -I./devices \
       -Os -Wall -ffunction-sections -fdata-sections -Wall \
	--specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb \
	-lm -fsingle-precision-constant -Wdouble-promotion \
	-u _printf_float

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
