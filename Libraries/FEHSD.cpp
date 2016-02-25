#include "FEHSD.h"
#include <FEHLCD.h>
#include <stdarg.h>

#define SD_CD_LOC (1<<6)


FEHSD SD;

FIL logfil;
static FATFS FATFS_Obj;
FRESULT f_res;

float x,y;

int counter = 0;

FEHSD::FEHSD(){
	SD.isOpen = 0;
}

void FEHSD::OpenLog(){
    int status;
    if(!SD.isOpen){
            SD.isOpen = 1;
            status = FEHSD::Initialize();
            if(status==0){
                char log_str[11] = "LOG000.TXT";
                uint16 log_no[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
                uint8 log_no_index;
                uint16 log_no_1;
                uint16 log_no_10;
                uint16 log_no_100;
                do
                {
                    if (log_no[log_no_index] > 999){
                        LCD.WriteLine("Too many logfiles created!");
                        return;
                    } //overflow

                    log_no_100 = log_no[log_no_index] / 100; //eliminate 1 and 10's place
                    log_no_10 = (log_no[log_no_index] - (log_no_100 * 100)) / 10; //subtract out 100's place, then eliminate 1's
                    log_no_1 = log_no[log_no_index] - (log_no_100 * 100) - (log_no_10 * 10);
                    log_str[5] = log_no_1 + '0';//1's
                    log_str[4] = log_no_10 + '0';//10's
                    log_str[3] = log_no_100 + '0';//100's

                    log_no[log_no_index]++;
                } while (f_open(&logfil, log_str, FA_CREATE_NEW | FA_WRITE) == FR_EXIST); //test to see if file exists, if it does, make a new one
            } else if (status==-1){
                LCD.WriteLine("SD Card not detetcted!");
            } else {
                LCD.Write(status);
                LCD.WriteLine("SD Card Initialize failed!");
            }
    }
    else{
        LCD.WriteLine("Logfile already open");
    }
}

void FEHSD::CloseLog(){
    f_close(&logfil);
    SD.isOpen = 0;
}

void FEHSD::Printf(
    const TCHAR* str,	/* Pointer to the format string */
    ...
)
{
    f_printf(&logfil, str);
}

int FEHSD::power(int base, int exp){
    int result = 1;
    int i = 0;
    for(i = 0; i < exp; i++){
        result *= base;
    }
    return result;
}

// val is the value to convert; precision is the number of decimal places
// to include, limit 10
char* FEHSD::FloatToString(float val, int precision){
    float val_copy = val;
    int digit = 0, len = 0, i = 0, j = 0;
    val_copy = val;

    //  if the value is negative, print a '-' char then make the float positive
    if(val < 0){
      str[j++] = '-';
        val = -val;
        val_copy = -val_copy;
    }
    // get length of argr
    // loop until int part of arg is 0
    while(int(val_copy) != 0){
        val_copy /= 10;
        len++;
    }
    //  put the integer part of the num
    for(i = len - 1; i != -1; i--){
        digit = (int) (val / power(10, i));
        digit %= 10;
        digit += '0';
        str[j++] = digit;
    }
    str[j++] = '.';
    // put the floating part of the num
    for(i = 1; i <= precision && i <= 10; i++){
        digit = (int) (val * power(10, i));
        digit %= 10;
        digit += '0';
        str[j++] = digit;
    }
    str[j++] = '\0';
    return str;
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
