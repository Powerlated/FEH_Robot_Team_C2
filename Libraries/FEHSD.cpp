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
int FEHFile::fileIdNum = 0;

// https://raw.githubusercontent.com/ShivanKaul/libslack/master/vsscanf.c
int our_vsscanf(const char *str, const char *format, va_list args)
{
    const char *f, *s;
    const char point = '.';
    int cnv = 0;

    for (s = str, f = format; *f; ++f)
    {
        if (*f == '%')
        {
            int size = 0;
            int width = 0;
            int do_cnv = 1;

            if (*++f == '*') {
                ++f, do_cnv = 0;
			}

            for (; isdigit((int)(unsigned int)*f); ++f){
                width *= 10, width += *f - '0';
			}

            if (*f == 'h' || *f == 'l' || *f == 'L')
                size = *f++;

            if (*f != '[' && *f != 'c' && *f != 'n')
                while (isspace((int)(unsigned int)*s))
                    ++s;

#define COPY                         *b++ = *s++, --width
#define MATCH(cond)                  if (width && (cond)) COPY;
#define MATCH_ACTION(cond, action)   if (width && (cond)) { COPY; action; }
#define MATCHES_ACTION(cond, action) while (width && (cond)) { COPY; action; }
#define FAIL                         (cnv) ? cnv : EOF
			switch (*f)
            {
                case 'd': case 'i': case 'o': case 'u': case 'x': case 'X':
                case 'p':
                {
                    static const char types[] = "diouxXp";
                    static const int bases[] = { 10, 0, 8, 10, 16, 16, 16 };
                    static const char digitset[] = "0123456789abcdefABCDEF";
                    static const int setsizes[] = { 10, 0, 0, 0, 0, 0, 0, 0, 8, 0, 10, 0, 0, 0, 0, 0, 22 };
                    int base = bases[strchr(types, *f) - types];
                    int setsize;
                    char buf[513];
                    char *b = buf;
                    int digit = 0;
                    if (width <= 0 || width > 512) width = 512;
                    MATCH(*s == '+' || *s == '-')
                    MATCH_ACTION(*s == '0',
                        digit = 1;
                        MATCH_ACTION((*s == 'x' || *s == 'X') && (base == 0 || base == 16), base = 16) else base = 8;
                    )
                    setsize = setsizes[base];
                    MATCHES_ACTION(memchr(digitset, *s, setsize), digit = 1)
                    if (!digit) return FAIL;
                    *b = '\0';
                    if (do_cnv)
                    {
                        if (*f == 'd' || *f == 'i')
                        {
                            long data = strtol(buf, NULL, base);
                            if (size == 'h')
                                *va_arg(args, short *) = (short)data;
                            else if (size == 'l')
                                *va_arg(args, long *) = data;
                            else
                                *va_arg(args, int *) = (int)data;
                        }
                        else
                        {
                            unsigned long data = strtoul(buf, NULL, base);
                            if (size == 'p')
                                *va_arg(args, void **) = (void *)data;
                            else if (size == 'h')
                                *va_arg(args, unsigned short *) = (unsigned short)data;
                            else if (size == 'l')
                                *va_arg(args, unsigned long *) = data;
                            else
                                *va_arg(args, unsigned int *) = (unsigned int)data;
                        }
                        ++cnv;
                    }
                    break;
                }

                case 'e': case 'E': case 'f': case 'g': case 'G':
                {
                    char buf[513];
                    char *b = buf;
                    int digit = 0;
                    if (width <= 0 || width > 512) width = 512;
                    MATCH(*s == '+' || *s == '-')
                    MATCHES_ACTION(isdigit((int)(unsigned int)*s), digit = 1)
                    MATCH(*s == point)
                    MATCHES_ACTION(isdigit((int)(unsigned int)*s), digit = 1)
                    MATCHES_ACTION(digit && (*s == 'e' || *s == 'E'),
                        MATCH(*s == '+' || *s == '-')
                        digit = 0;
                        MATCHES_ACTION(isdigit((int)(unsigned int)*s), digit = 1)
                    )
                    if (!digit) return FAIL;
                    *b = '\0';
                    if (do_cnv)
                    {
                        double data = strtod(buf, NULL);
                        if (size == 'l')
                            *va_arg(args, double *) = data;
                        else if (size == 'L')
                            *va_arg(args, long double *) = (long double)data;
                        else
                            *va_arg(args, float *) = (float)data;
                        ++cnv;
                    }
                    break;
                }

                case 's':
                {
                    char *arg = va_arg(args, char *);
                    // Changed from INT_MAX to 425000 b/c we limit the
                    // Max size of writing to be 2048 (see Fprintf)
                    if (width <= 0) width = 425000; // INT_MAX
                    while (width-- && *s && !isspace((int)(unsigned int)*s)){
                        if (do_cnv) *arg++ = *s++;
					}
                    if (do_cnv) *arg = '\0', ++cnv;
                    break;
                }

                case '[':
                {
                    char *arg = va_arg(args, char *);
                    int setcomp = 0;
                    size_t setsize;
                    const char *end;
                    if (width <= 0) width = 425000; // INT_MAX
                    if (*++f == '^') setcomp = 1, ++f;
                    end = strchr((*f == ']') ? f + 1 : f, ']');
                    if (!end) return FAIL; /* Could be cnv to match glibc-2.2 */
                    setsize = end - f;     /* But FAIL matches the C standard */
                    while (width-- && *s)
                    {
                        if (!setcomp && !memchr(f, *s, setsize)) break;
                        if (setcomp && memchr(f, *s, setsize)) break;
                        if (do_cnv) *arg++ = *s++;
                    }
                    if (do_cnv) *arg = '\0', ++cnv;
                    f = end;
                    break;
                }

                case 'c':
                {
                    char *arg = va_arg(args, char *);
                    if (width <= 0) width = 1;
                    while (width--)
                    {
                        if (!*s) return FAIL;
                        if (do_cnv) *arg++ = *s++;
                    }
                    if (do_cnv) ++cnv;
                    break;
                }

                case 'n':
                {
                    if (size == 'h')
                        *va_arg(args, short *) = (short)(s - str);
                    else if (size == 'l')
                        *va_arg(args, long *) = (long)(s - str);
                    else
                        *va_arg(args, int *) = (int)(s - str);
                    break;
                }

                case '%':
                {
                    if (*s++ != '%') return cnv;
                    break;
                }

                default:
                    return FAIL;
            }
        }
        else if (isspace((int)(unsigned int)*f))
        {
            while (isspace((int)(unsigned int)f[1]))
                ++f;
            while (isspace((int)(unsigned int)*s))
                ++s;
        }
        else
        {
            if (*s++ != *f)
                return cnv;
        }
    }

    return cnv;
}

FEHSD::FEHSD(){
	SD.isInitialized = 0;
	SD.numberOfFiles = 0;
}

FEHFile *FEHSD::FOpen(const TCHAR* str, const TCHAR* mode){
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
        	FatFsMode = FA_OPEN_ALWAYS | FA_WRITE;
        }else if(strcmp(mode,"a+") == 0){
        	FatFsMode = FA_OPEN_ALWAYS | FA_WRITE | FA_READ;
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
    const TCHAR* str,	/* Pointer to the format string */
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

int FEHSD::FScanf(FEHFile *fptr, const TCHAR* format, ...) {
	
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
	int numRead = our_vsscanf(buffer, format, args);
	
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

