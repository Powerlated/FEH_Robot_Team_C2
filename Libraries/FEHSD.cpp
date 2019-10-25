#include "FEHSD.h"
#include <FEHLCD.h>
#include <stdarg.h>
#include <stdio.h>

#define SD_CD_LOC (1<<6)


FEHSD SD;
static FATFS FATFS_Obj;
FRESULT f_res;

FEHSD::FEHSD(){
	SD.isOpen = 0;
}

FEHFile FEHSD::FOpen(const TCHAR* str, const TCHAR* mode){
    int status;
    FEHFile File();
    BYTE FatFsMode;
    if(!SD.isOpen){
            SD.isOpen = 1;
            status = FEHSD::Initialize();
            if(status==0){
            	
                if(mode == "r") FatFsMode = FA_READ;
                if(mode == "r+") FatFsMode = FA_READ | FA_WRITE;
                if(mode == "w") FatFsMode = FA_CREATE_ALWAYS | FA_WRITE;
                if(mode == "w+") FatFsMode = FA_CREATE_ALWAYS | FA_WRITE | FA_READ;
                if(mode == "a") FatFsMode = FA_OPEN_APPEND | FA_WRITE;
                if(mode == "a+") FatFsMode = FA_OPEN_APPEND | FA_WRITE | FA_READ;
                if(mode == "wx") FatFsMode = FA_CREATE_NEW | FA_WRITE;
                if(mode == "w+x") FatFsMode = FA_CREATE_NEW | FA_WRITE | FA_READ;
				
		f_res = f_open(File.wrapper, str, FatFsMode); //I welcome thou's attempt to use files
		if(f_res != 0 || f_error(File.Wrapper) != 0){
			LCD.WriteLine("File failed to open");
		}

            } else if (status==-1){
                LCD.WriteLine("SD Card not detected!");
            } else {
                LCD.Write(status);
                LCD.WriteLine("SD Card Initialize failed!");
            }
    }
    else{
        LCD.WriteLine("File is already open already open");
    }
    
    return File;
}

void FEHSD::FClose(FEHFile fptr){
	if(SD.isOpen){
    	f_close(fptr.wrapper); //I banish thy memoryleaks
    	SD.isOpen = 0;
	}
}

int FEHSD::FPrintf(const FEHFile fptr,
    const TCHAR* str,	/* Pointer to the format string */
    ...
)
{
	va_list args;
	va_start(args, str);
	f_printf(fptr.wrapper, str, args);

	// Check file for errors and return
	return = f_error(fptr.wrapper);
}

int FEHSD::FScanf(const FEHFile fptr, const TCHAR* format, va_list args) {
	FIL file;

	// Check for end of file, return -1 if eof
	if(f_eof(fptr)){
		return -1;
	}

	// Create char buffer (buffer > 2048 will crash)
	int bufferSize = 2048;
	char buffer[bufferSize];

	// Get correct line and store in buffer
	f_gets(buffer, bufferSize, fptr.wrapper);

	// Scan line and store in args; also get number of args read
	int numRead = vsscanf(buffer, format, args);

	// Check file for errors
	//f_error(fptr.wrapper);

	// Return number of successful reads
	return numRead;
}

int FEHSD::Initialize(){

    PORTE_PCR6 = PORT_PCR_MUX(1);
    PTD_BASE_PTR->PDDR &= ~SD_CD_LOC;
    int opens = 0;
    if (SDCARD_GPIO_DETECT == 0)
    {
        Sleep(300);
        opens = disk_initialize(0);
        if (SDHC_Info.CARD == ESDHC_CARD_SD)
        {

            LCD.WriteLine("SD card initiallized");
        }
        if (SDHC_Info.CARD == ESDHC_CARD_SDHC)
        {

            LCD.WriteLine("SDHC card initialized !");
        }

        // Mount FAT File System
        f_res = f_mount((uint8)0, &FATFS_Obj);
        if (f_res == 0)
        {

            LCD.WriteLine("FAT filesystem mounted !");
            return opens;
        }else
        {

            LCD.WriteLine("Failed to mount SD card !");
            return opens;
        }
            return opens;
        opens = 1;

    } else{
        opens = -1;
        return opens;
    }
    Sleep(300);
    return opens;
}
