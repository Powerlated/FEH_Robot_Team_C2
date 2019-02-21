#ifndef FEHSD_H
#define FEHSD_H
#include "SDHC.h"
#include "ff.h"
#include "ffconf.h"
#include "diskio.h"

class FEHSD
{
public:
    FEHSD();
    void OpenLog();
    void CloseLog();
    void Printf(const TCHAR* str,	/* Pointer to the format string */
    ...	);
    void fscanf(const TCHAR* file_name, const TCHAR* format, ...);
    void fscanf(const TCHAR* file_name, int line, const TCHAR* format, ...);


private:
	  int isOpen;
    int Initialize();
    void fscanf(const TCHAR* file_name, int line, const TCHAR* format, va_list list);
};

extern FEHSD SD;
#endif
