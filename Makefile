CC = arm-none-eabi-g++
LD = $(CC)

ifeq ($(OS),Windows_NT)
    SHELL := CMD
endif

SPECS = nosys.specs

INCLUDES = -I.. -I. -ILibraries/ -IDrivers/ -IStartup/ 
ARGS = -O0 -ffunction-sections -fdata-sections -fno-exceptions -c -fmessage-length=0 -Wno-psabi -specs=$(SPECS)
CFLAGS =  -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mfloat-abi=soft -mthumb -g3 -gdwarf-2 -gstrict-dwarf

AUTOCPP := $(wildcard ../*.cpp)
AUTOH := $(wildcard ../*.h)

AUTOOBJECTS := $(patsubst %.cpp,%.o,$(AUTOCPP))
OBJECTS := $(AUTOOBJECTS) Startup/startup_mk60d10.o Libraries/FEHMotor.o Drivers/mcg.o Drivers/i2c.o Drivers/spi.o Drivers/uart.o Drivers/ff.o Drivers/SDHC.o Drivers/lptmr.o FEHProteus.o Drivers/FEHPropeller.o Libraries/FEHUtility.o Libraries/FEHIO.o Drivers/adc16.o Libraries/FEHBuzzer.o Libraries/FEHServo.o Libraries/FEHLCD.o Libraries/FEHAccel.o Libraries/FEHBattery.o Drivers/FEHXBee.o Libraries/FEHRPS.o Libraries/FEHSD.o Libraries/FEHRandom.o

ifeq ($(OS), Windows_NT)
DELOBJECTS := $(subst /,\, $(OBJECTS))
endif

all: $(TARGET).elf $(TARGET).s19

clean:
ifeq ($(OS),Windows_NT)
	del $(DELOBJECTS) ..\$(TARGET).elf ..\$(TARGET).s19 ..\$(TARGET).map $(DELOBJECTS:%.o=%.d)
else
	rm -f $(OBJECTS) ../$(TARGET).elf ../$(TARGET).s19 ../$(TARGET).map $(OBJECTS:%.o=%.d)
endif

%.o : %.c $(AUTOH)
	$(CC) $(INCLUDES) $(ARGS) $(CFLAGS) -c $< -o $@

%.o : %.cpp $(AUTOH)
	$(CC) $(INCLUDES) $(ARGS) $(CFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJECTS)
	$(LD) $(OBJECTS) -u _printf_float -u _scanf_float -TLinker/MK60DN512Z_flash.ld -Xlinker --gc-sections -Wl,-Map,../$(TARGET).map -n -specs=$(SPECS) -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -g3 -gdwarf-2 -gstrict-dwarf -g -o ../$(TARGET).elf

$(TARGET).s19: $(TARGET).elf
	arm-none-eabi-objcopy  -O srec --srec-len=40 --srec-forceS3 ../$(TARGET).elf ../$(TARGET).s19

deploy:
ifeq ($(OS),Windows_NT)
	@tools\copyS19.bat
else
	@tools/copyS19
endif

run:
	@arm-none-eabi-size  ../$(TARGET).elf > ../size.txt
	@echo ----------------------------------------------
ifeq ($(OS),Windows_NT)
	@tools\sizeGet.bat
else
	@tools/sizeGet
endif
	@echo ----------------------------------------------
	@echo Download Successful. Please eject the SD card. >&2
	@echo ----------------------------------------------
