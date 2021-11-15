#ifndef FEHSD_H
#define FEHSD_H
#include "SDHC.h"
#include "ff.h"
#include "ffconf.h"
#include "diskio.h"
#include "FEHFile.h"

/**
 * @brief Access to the Proteus SD card
 * 
 * Max number of files allowed on SD card = TODO <br/>
 * SD card must be inserted in the Proteus for file access
 */
class FEHSD
{
public:
    /**
     * @brief Open a file on the SD card
     * 
     * File name must be at most 8 characters long (including file extension). <br/>
     * <br/>
     * Possible file access modes: <br/>
     * "r" = Opens a file for reading. The file must exist. <br/>
     * "w" = Creates an empty file for writing. If a file with the same name already exists, its content is erased and the file is considered as a new empty file. <br/>
     * "a" = Appends to a file. Writing operations, append data at the end of the file. The file is created if it does not exist. <br/>
     * "r+" = Opens a file to update both reading and writing. The file must exist. <br/>
     * "w+" = Creates an empty file for both reading and writing. <br/>
     * "a+" = Opens a file for reading and appending.
     * 
     * @param str Name of the file to be opened
     * @param mode File access mode used
     * @return FEHFile* Opened file pointer
     */
    FEHFile *FOpen(const char* str, const char* mode);

    /**
     * @brief Close an opened file
     * 
     * Close an opened file
     * 
     * @param fptr File to be closed
     * @return int 0 if successful
     */
    int FClose(FEHFile *fptr);

    /**
     * @brief Close all opened files
     * 
     * Close all opened files
     * 
     * @return int 0 if successful
     */
    int FCloseAll();

    /**
     * @brief Print to a file
     * 
     * Variables used in function call replace format specifiers in string from left to right <br/>
     * <br/>
     * Format specifiers: <br/>
     * "%i" or "%d" = Integer <br/>
     * "%f" = Float <br/>
     * "%p" = Pointer address <br/>
     * "%c" = Character <br/>
     * "%s" = String <br/>
     * "%%" = Percent symbol <br/>
     * "%e" = Scientific notation (3.92e+2) <br/>
     * Width and precision specifiers can be used (i.e. "%3.2f") <br/>
     * Shorts and longs can be read (i.e. "%hi" and "%li")
     * 
     * @param fptr File to be written to
     * @param format String containing desired format specifiers
     * @param ... Variables to be formatted into string
     * @return int Number of characters printed
     */
    int FPrintf(FEHFile *fptr,const char* format,
    		/* Pointer to the format string */ ...);

    /**
     * @brief Scan data from a file
     * 
     * Variables used in function call replace format specifiers in string from left to right <br/>
     * <br/>
     * Format specifiers: <br/>
     * "%i" or "%d" = Integer <br/>
     * "%f" = Float <br/>
     * "%c" = Character <br/>
     * "%s" = String <br/>
     * Shorts and longs can be read (i.e. "%hi" and "%li")
     * 
     * @param fptr File to be scanned from
     * @param format String containing desired format specifiers
     * @param ... Variables to be formatted into string
     * @return int Number of items successfully read
     */
    int FScanf(FEHFile *fptr, const char* format, ...);
    
    /**
     * @brief Check for end of file
     * 
     * End of file is true after failing a scan
     * 
     * @param fptr File to be checked
     * @return int Nonzero if at end of file
     */
    int FEof(FEHFile *fptr);
    //int FSeek(FEHFile *fptr, long int offset, int position);

    FEHSD();

private:
    int Initialize();
    int isInitialized;
	int numberOfFiles;
};

/**
 * @brief Global access to FEHSD class
 * 
 * Global access to FEHSD class
 * 
 */
extern FEHSD SD;
#endif
