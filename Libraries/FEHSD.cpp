#include "FEHSD.h"
#include <FEHLCD.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <FEHUtility.h>
#include "stdlib.h"
#include "ctype.h"
#include "string.h"

#define SD_CD_LOC (1<<6)
using namespace std;

FEHSD SD;
static FATFS FATFS_Obj;
FRESULT f_res;
FEHFile *filePtrs[25];

int FEHFile::prevFileId = 0;

bool inAppendMode = false;

FEHSD::FEHSD(){
	SD.isInitialized = 0;
	SD.numberOfFiles = 0;
}

FEHFile *FEHSD::FOpen(const char* str, const char* mode){
    BYTE FatFsMode;
    FEHFile *File = new FEHFile();
    int status;
    if(!SD.isInitialized){
    	status = FEHSD::Initialize();
    }
    if(SD.isInitialized == 1){
		//Choosing the appropriate access mode
        if(strcmp(mode,"r") == 0){
        	FatFsMode = FA_READ;
        }else if(strcmp(mode,"r+") == 0){
        	FatFsMode = FA_READ | FA_WRITE;
        }else if(strcmp(mode,"w") == 0){
        	FatFsMode = FA_CREATE_ALWAYS | FA_WRITE;
        }else if(strcmp(mode,"w+") == 0){
        	FatFsMode = FA_CREATE_ALWAYS | FA_WRITE | FA_READ;
        }else if(strcmp(mode,"a") == 0){
        	FatFsMode = FA_OPEN_APPEND | FA_WRITE;
			inAppendMode = true;
        }else if(strcmp(mode,"a+") == 0){
        	FatFsMode = FA_OPEN_APPEND | FA_WRITE | FA_READ;
			inAppendMode = true;
        }else if(strcmp(mode,"wx") == 0){
        	FatFsMode = FA_CREATE_NEW | FA_WRITE;
        }else if(strcmp(mode,"w+x") == 0){
        	FatFsMode = FA_CREATE_NEW | FA_WRITE | FA_READ;
        }else{
        	FatFsMode = FA_CREATE_ALWAYS | FA_WRITE;
        }

        f_res = f_open(&(File->wrapper), str, FatFsMode); //I welcome thou's attempt to use files
		
        filePtrs[SD.numberOfFiles++] = File;

		if(f_res != 0 || f_error(&(File->wrapper)) != 0){
			LCD.WriteLine("File failed to open");
		}else if(inAppendMode){
			f_lseek(&(File->wrapper), f_size(&(File->wrapper))); //go to the end of the file if wanting to append
		}
		
    } else if (status == -1){
        LCD.WriteLine("SD Card not detected!");
    } else {
        LCD.Write(status);
        LCD.WriteLine("SD Card Initialize failed!");
    }
    return filePtrs[SD.numberOfFiles - 1] ;
}

int FEHSD::FClose(FEHFile *fptr){
	int i, j;
	if(SD.isInitialized && fptr != NULL){
        for (i = 0; i < SD.numberOfFiles; i++){
        	if(fptr->fileIdNum == (filePtrs[i])->fileIdNum){
				f_res = f_close(&(filePtrs[i]->wrapper));  //I banish thy memoryleaks
				//Shift all elements in array one over to the left
				SD.numberOfFiles--;
				for(j = i; j < SD.numberOfFiles; j++){
					filePtrs[j] = filePtrs[j+1];
				}
				filePtrs[SD.numberOfFiles] = NULL;
				break;
			}
		}
    }
    return f_res;
}

int FEHSD::FCloseAll(){
	int i;
	if(SD.isInitialized){
		for (i = 0; i < SD.numberOfFiles; i++){
        	if(filePtrs[i] != NULL){
				f_res = f_close(&(filePtrs[i]->wrapper));  //I banish thy memoryleaks
			}
		}
		SD.numberOfFiles = 0;
		SD.isInitialized = 0;
	}
	return f_res;
}

int FEHSD::FEof(FEHFile *fptr){
	return f_eof(&(fptr->wrapper));
}

int FEHSD::FPrintf(FEHFile *fptr,
    const char* str,	/* Pointer to the format string */
    ...
)
{
	va_list args;
	va_start(args, str);
	int numChars = f_printf(&(fptr->wrapper), str, args);
	if(f_error(&(fptr->wrapper)) != 0){
		LCD.WriteLine("Error printing to file");
		return -1;
	}
	va_end(args);
	// Return number of characters printed
	return numChars;
}

int FEHSD::FScanf(FEHFile *fptr, const char* format, ...) {

	va_list args;
	va_start(args, format);

	// Check for end of file, return -1 if eof
	if(f_eof(&(fptr->wrapper))){
		return -1;
	}

	// Create string buffer (buffer > 2048 will crash)
	int bufferSize = 2048;
	char buffer[bufferSize];

	// Get correct line and store in buffer
	f_gets(buffer, bufferSize, &(fptr->wrapper));

	if(f_error(&(fptr->wrapper)) != 0){
		LCD.WriteLine("Error reading from file");
		return -1;
	}

	// Scan line and store in args; also get number of args read
	int numRead = vsscanf(buffer, format, args);

	va_end(args);

	// Return number of successful reads
	return numRead;
}

//First draft of FSeek
// int FEHSD::FSeek(FEHFile *fptr, long int offset, int position){
// 	if(position == SEEK_SET){
// 		return f_lseek(&(fptr->wrapper), offset);
// 	}else if(position == SEEK_CUR){
// 		return f_lseek(&(fptr->wrapper), f_tell(&(fptr->wrapper)) + offset);
// 	}else if(position == SEEK_END){
// 		return f_lseek(&(fptr->wrapper), f_size(&(fptr->wrapper)) + offset);
// 	}else{
// 		return f_lseek(&(fptr->wrapper), 0);
// 	}
// }

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
            LCD.WriteLine("SD card initialized");
            SD.isInitialized = 1;
        }
        if (SDHC_Info.CARD == ESDHC_CARD_SDHC)
        {
            LCD.WriteLine("SDHC card initialized !");
			SD.isInitialized = 1;
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
    } else{
        opens = -1;
        return opens;
    }
    Sleep(300);
    return opens;
}
