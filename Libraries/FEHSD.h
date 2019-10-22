#ifndef FEHSD_H
#define FEHSD_H
#include "SDHC.h"
#include "ff.h"
#include "ffconf.h"
#include "diskio.h"
#include "FEHFile.h"
class FEHSD
{
public:
    FEHSD();
    FEHFile FOpen(const TCHAR* str, const TCHAR* mode);
    int FClose(FEHFile fptr);
    int FPrintf(const TCHAR* str,	/* Pointer to the format string */
    ...	);
    int FScanf(const FEHFile fptr, const TCHAR* format, ...);
	int FSeek(const FEHFile fptr, long int offset, int position)


private:
	int isOpen;
    int Initialize();
};

extern FEHSD SD;
#endif
