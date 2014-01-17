GITBINARY := git
FEHURL := feh.osu.edu
FIRMWAREREPO := fehproteusfirmware

ifeq ($(FEHQTINSTALL),)
FEHQTINSTALL = C:\apps\FEHQt
endif

FEHPROTEUSINSTALL = $(FEHQTINSTALL)\Proteus

CXX = arm-none-eabi-g++
LD = $(CXX)

SPECS = "$(FEHPROTEUSINSTALL)/GCC/arm-none-eabi/lib/armv7e-m/ewl_c_noio.specs"
ARGS = -O0 -ffunction-sections -fdata-sections -fno-exceptions -c -fmessage-length=0 -specs=$(SPECS)

##-Wa,-adhlns="$@.lst"
CFLAGS =  -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -g3 -gdwarf-2 -gstrict-dwarf

INCLUDES = -I.. -I. -IDrivers/ -ILibraries/ -IStartup/  -I"$(FEHPROTEUSINSTALL)/EWL/EWL_C/include" -I"$(FEHPROTEUSINSTALL)/EWL/EWL_C++/include" -I"$(FEHPROTEUSINSTALL)/EWL/EWL_Runtime/include" -I..\Generated_Code -I..\Sources -I..\Startup_Code
LIBS = -L"$(FEHPROTEUSINSTALL)/EWL/lib/armv7e-m"

# AUTOCPP := $(shell cat ../$(TARGET).files | grep cpp | gawk '{ printf "..\\\%%s ", $$1 } END { printf "\n" }' )
# AUTOCPP := $(shell Tools/egrep cpp$$ ../$(TARGET).files)
# AUTOH := $(shell Tools/egrep h$$ ../$(TARGET).files)

ifeq ($(OS),Windows_NT)
# AUTOOBJECTS := $(patsubst %.cpp,..\\%.o,$(AUTOCPP))
# AUTOH := $(patsubst %.h,..\\%.h,$(AUTOH))
OBJECTS := ..\Generated_Code\PE_LDD.o ..\Generated_Code\wdt_kinetis.o ..\Generated_Code\USB1.o ..\Generated_Code\USB0.o ..\Generated_Code\usb_framework.o ..\Generated_Code\usb_driver.o ..\Generated_Code\usb_descriptor.o ..\Generated_Code\usb_dci_kinetis.o ..\Generated_Code\usb_class.o ..\Generated_Code\usb_cdc_pstn.o ..\Generated_Code\usb_cdc.o ..\Generated_Code\Tx1.o ..\Generated_Code\Rx1.o ..\Generated_Code\CDC1.o ..\Generated_Code\UTIL1.o ..\Generated_Code\WAIT1.o ..\Generated_Code\Vectors.o ..\Generated_Code\Cpu.o ..\Sources\Events.o ..\Sources\ProcessorExpert.o ..\Startup_Code\__arm_start.o ..\Startup_Code\__arm_end.o
# OBJECTS := $(AUTOOBJECTS) Startup\__arm_start.o Startup\__arm_end.o Startup\kinetis_sysinit.o Libraries\FEHMotor.o Drivers\mcg.o Drivers\uart.o Drivers\lptmr.o FEHProteus.o Drivers\FEHPropeller.o Libraries\FEHUtility.o Libraries\FEHIO.o Drivers\adc16.o Libraries\FEHBuzzer.o Libraries\FEHServo.o Libraries\FEHLCD.o Libraries\FEHBattery.o Drivers\FEHXBee.o Libraries\FEHMOM.o
else
AUTOOBJECTS := $(patsubst %.cpp,../%.o,$(AUTOCPP))
AUTOH := $(patsubst %.h,../%.h,$(AUTOH))
OBJECTS := $(AUTOOBJECTS) Startup/__arm_start.o Startup/__arm_end.o Startup/kinetis_sysinit.o Libraries/FEHMotor.o Drivers/mcg.o Drivers/uart.o Drivers/lptmr.o FEHProteus.o Drivers/FEHPropeller.o Libraries/FEHUtility.o Libraries/FEHIO.o Drivers/adc16.o Libraries/FehBuzzer.o Libraries/FEHServo.o Libraries/FEHLCD.o Libraries/FEHBattery.o Drivers/FEHXBee.o Libraries/FEHMOM.o
endif

all: $(TARGET).elf $(TARGET).s19

clean:
ifeq ($(OS),Windows_NT)
	del $(OBJECTS) ..\$(TARGET).elf ..\$(TARGET).s19 ..\$(TARGET).map $(OBJECTS:%.o=%.d)
else
	@rm -rf $(OBJECTS) ../$(TARGET).elf ../$(TARGET).s19 ../$(TARGET).map $(OBJECTS:%.o=%.d)
endif

%.o : %.c
	$(CXX) $(INCLUDES) $(ARGS) $(CFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJECTS)
	$(LD) $(OBJECTS) $(LIBS) -TLinker/MK60DN512Z_flash.ld -Xlinker --gc-sections -Wl,-Map,../$(TARGET).map -n -specs=$(SPECS) -Xlinker --undefined=__pformatter_ -Xlinker --defsym=__pformatter=__pformatter_ -Xlinker --undefined=__sformatter -Xlinker --defsym=__sformatter=__sformatter -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -g3 -gdwarf-2 -gstrict-dwarf -g -o ../$(TARGET).elf

$(TARGET).s19: $(TARGET).elf
	arm-none-eabi-objcopy  -O srec --srec-len=40 --srec-forceS3 ../$(TARGET).elf ../$(TARGET).s19

deploy:

run:
ifeq ($(OS),Windows_NT)
	-explorer ..\.
endif

