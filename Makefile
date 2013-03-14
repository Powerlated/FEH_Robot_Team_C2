GITBINARY := git
FEHURL := feh.osu.edu
FIRMWAREREPO := FEHRobotControllerFirmware

CXX = arm-none-eabi-gcc
LD = $(CXX)

SPECS = "$(FEHPROTEUSINSTALL)/GCC/arm-none-eabi/lib/armv7e-m/ewl_c++.specs"
ARGS = -O0 -ffunction-sections -fdata-sections -Wall -fno-exceptions -c -fmessage-length=0 -specs=$(SPECS)

##
CFLAGS = -Wa,-adhlns="$@.lst" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -g3 -gdwarf-2 -gstrict-dwarf

INCLUDES = -I.. -I. -IDrivers/ -ILibraries/ -IStartup/  -I"$(FEHPROTEUSINSTALL)/EWL/EWL_C/include" -I"$(FEHPROTEUSINSTALL)/EWL/EWL_C++/include" -I"$(FEHPROTEUSINSTALL)/EWL/EWL_Runtime/include"
LIBS = -L"$(FEHPROTEUSINSTALL)/EWL/lib/armv7e-m"

#CPPFILES := $(shell cat $(TARGET).files | grep cpp | awk '{ printf "%s ", $$1 } END { printf "\n" }')
#OBJECTS := $(CPPFILES:.cpp=.o)
#AUTOOBJECTS := $(shell cat $(TARGET).files | grep cpp | "C:/Program Files (x86)/Git/bin/awk" '{ printf "%so ", substr( $$1, 1, match( $$1, "\." ) )  } END { printf "\n" }')

ifeq ($(OS),Windows_NT)
OBJECTS := ..\main.o Startup\__arm_start.o Startup\__arm_end.o Startup\kinetis_sysinit.o Libraries\FEHMotor.o Drivers\mcg.o Drivers\uart.o Drivers\lptmr.o FEHProteus.o Drivers\FEHPropeller.o Libraries\FEHUtility.o Libraries\FEHIO.o Drivers\adc16.o Libraries\FEHBuzzer.o Libraries\FEHServo.o
else
OBJECTS := ../main.o Startup/__arm_start.o Startup/__arm_end.o Startup/kinetis_sysinit.o Libraries/FEHMotor.o Drivers/mcg.o Drivers/uart.o Drivers/lptmr.o FEHProteus.o Drivers/FEHPropeller.o Libraries/FEHUtility.o Libraries/FEHIO.o Drivers/adc16.o Libraries/FehBuzzer.o Libraries/FEHServo.o
endif

all: $(TARGET).elf $(TARGET).s19

clean:
ifeq ($(OS),Windows_NT)
	del $(OBJECTS) ..\$(TARGET).elf ..\$(TARGET).s19 ..\$(TARGET).map $(OBJECTS:%.o=%.d) $(OBJECTS:%.o=%.o.lst)
else
	@rm -rf $(OBJECTS) ../$(TARGET).elf ../$(TARGET).s19 ../$(TARGET).map $(OBJECTS:%.o=%.d) $(OBJECTS:%.o=%.o.lst)
endif

%.o : %.cpp
	$(CXX) $(INCLUDES) $(ARGS) $(CFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJECTS)
	$(LD) $(OBJECTS) $(LIBS) -TLinker/MK60DN512Z_flash.ld -Xlinker --gc-sections -Wl,-Map,../$(TARGET).map -n -specs=$(SPECS) -Xlinker --undefined=__pformatter_ -Xlinker --defsym=__pformatter=__pformatter_ -Xlinker --undefined=__sformatter -Xlinker --defsym=__sformatter=__sformatter -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -g3 -gdwarf-2 -gstrict-dwarf -g -o ../$(TARGET).elf

$(TARGET).s19: $(TARGET).elf
	arm-none-eabi-objcopy  -O srec --srec-len=40 --srec-forceS3 ../$(TARGET).elf ../$(TARGET).s19

deploy:

run:


