#ifndef FEHSD_H
#define FEHSD_H
#include "SDHC.h"
#include "ff.h"
#include "ffconf.h"
#include "diskio.h"

#define STR_SIZE = 20

class FEHSD
{
public:
    FEHSD();
    void OpenLog();
    void CloseLog();
    void Printf(const TCHAR* str,	/* Pointer to the format string */
    ...	);
    char* FloatToString(float val, int precision);


private:
	int isOpen;
    int Initialize();
    int power(int base, int exponent);
    char str[STRING SIZE];
};

extern FEHSD SD;
#endif

