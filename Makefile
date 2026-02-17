CC=arm-none-eabi-gcc

F4FLIGHT: FC = F4FLIGHT
F4FLIGHT: DEVICE = STM32F4xx
F4FLIGHT: MCU = STM32F405xx
F4FLIGHT: FPU = fpv4-sp-d16
F4FLIGHT: CPU = cortex-m4
F4FLIGHT: OCDTARGET = stm32f4x

H7FLIGHT: FC = H7FLIGHT
H7FLIGHT: DEVICE = STM32H7xx
H7FLIGHT: MCU = STM32H723xx
H7FLIGHT: FPU = fpv5-d16
H7FLIGHT: CPU = cortex-m7
H7FLIGHT: OCDTARGET = stm32h7x

BUILDDIR=build
SOURCES=$(wildcard ./*.c) $(wildcard ./devices/*.c) \
	$(wildcard ./Drivers/$(DEVICE)_HAL_Driver/Src/*.c) \
	$(wildcard ./$(DEVICE)/*.c)
STARTUP=$(wildcard ./$(DEVICE)/*.s)
OBJECTS=$(SOURCES:%.c=$(BUILDDIR)/%.o) $(STARTUP:%.s=$(BUILDDIR)/%.o)
LDFLAGS=-mcpu=$(CPU) -T./$(DEVICE)/linker-script.ld \
	--specs=nosys.specs -Wl,--gc-sections -static \
	--specs=nano.specs -mfpu=$(FPU) -mfloat-abi=hard -mthumb \
	-Wl,--start-group -lc -lm -Wl,--end-group \
	-fsingle-precision-constant -u _printf_float \
	-Wl,-Map=$(BUILDDIR)/memmap.map
CFLAGS=-mcpu=$(CPU) -std=gnu11 -DUSE_HAL_DRIVER -D$(MCU) -c \
       -I. -I./Drivers/$(DEVICE)_HAL_Driver/Inc/Legacy \
       -I./Drivers/$(DEVICE)_HAL_Driver/Inc \
       -I./Drivers/CMSIS/Device/ST/$(DEVICE)/Include \
       -I./Drivers/CMSIS/Include -I./devices \
       -I./$(DEVICE) -D$(FC) -D$(DEVICE)\
       -Os -Wall -ffunction-sections -fdata-sections -Wall \
	--specs=nano.specs -mfpu=$(FPU) -mfloat-abi=hard -mthumb \
	-lm -fsingle-precision-constant -Wdouble-promotion \
	-u _printf_float

SFLAGS=-mcpu=$(CPU) -c -x assembler-with-cpp --specs=nano.specs \
       -mfpu=$(FPU) -mfloat-abi=hard -mthumb \
       -fsingle-precision-constant

all: F4FLIGHT

F4FLIGHT: load
H7FLIGHT: load

load: $(BUILDDIR)/prog.elf
	openocd -f interface/stlink.cfg -f target/$(OCDTARGET).cfg \
		-c "program $(BUILDDIR)/prog.elf verify reset exit"

.SECONDEXPANSION:
$(BUILDDIR)/prog.elf: $$(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

$(BUILDDIR)/%.o: %.s
	@mkdir -p $(@D)
	$(CC) $(SFLAGS) $< -o $@

$(BUILDDIR)/%.o: %.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -r build
