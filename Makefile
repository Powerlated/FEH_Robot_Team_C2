# RM := rm -rf

CC = arm-none-eabi-g++
LD = $(CC)

SPECS = nosys.specs

INCLUDES = -I.. -I. -ILibraries/ -IDrivers/ -IStartup/ -I"$(FEHPROTEUSINSTALL)/GCC/arm-none-eabi/include" -I"$(FEHPROTEUSINSTALL)/GCC/lib/gcc/arm-none-eabi/9.2.1/include" -I"$(FEHPROTEUSINSTALL)/GCC/lib/gcc/arm-none-eabi/9.2.1/include-fixed"
ARGS = -O0 -ffunction-sections -fdata-sections -fno-exceptions -c -fmessage-length=0 -Wno-psabi -specs=$(SPECS)
CFLAGS =  -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mfloat-abi=soft -mthumb -g3 -gdwarf-2 -gstrict-dwarf

AUTOCPP := $(shell Tools/egrep cpp$$ ../$(TARGET).files)
AUTOH := $(shell Tools/egrep h$$ ../$(TARGET).files)

AUTOOBJECTS := $(patsubst %.cpp,../%.o,$(AUTOCPP))
AUTOH := $(patsubst %.h,../%.h,$(AUTOH))
OBJECTS := $(AUTOOBJECTS) Startup/startup_mk60d10.o Libraries/FEHMotor.o Drivers/mcg.o Drivers/i2c.o Drivers/spi.o Drivers/uart.o Drivers/ff.o Drivers/SDHC.o Drivers/lptmr.o FEHProteus.o Drivers/FEHPropeller.o Libraries/FEHUtility.o Libraries/FEHIO.o Drivers/adc16.o Libraries/FEHBuzzer.o Libraries/FEHServo.o Libraries/FEHLCD.o Libraries/FEHAccel.o Libraries/FEHBattery.o Drivers/FEHXBee.o Libraries/FEHRPS.o Libraries/FEHSD.o Libraries/FEHRandom.o

#OBJS=$(SRCS:.c=.o)

all: $(TARGET).elf $(TARGET).s19

clean:
	del $(OBJECTS) ..\$(TARGET).elf ..\$(TARGET).s19 ..\$(TARGET).map $(OBJECTS:%.o=%.d)

%.o : %.c $(AUTOH)
	$(CC) $(INCLUDES) $(ARGS) $(CFLAGS) -c $< -o $@

%.o : %.cpp $(AUTOH)
	$(CC) $(INCLUDES) $(ARGS) $(CFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJECTS)
	$(LD) $(OBJECTS) -u _printf_float -u _scanf_float -TLinker/MK60DN512Z_flash.ld -Xlinker --gc-sections -Wl,-Map,../$(TARGET).map -n -specs=$(SPECS) -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -g3 -gdwarf-2 -gstrict-dwarf -g -o ../$(TARGET).elf

$(TARGET).s19: $(TARGET).elf
	arm-none-eabi-objcopy  -O srec --srec-len=40 --srec-forceS3 ../$(TARGET).elf ../$(TARGET).s19
	
deploy:
	@tools\copyS19.bat
