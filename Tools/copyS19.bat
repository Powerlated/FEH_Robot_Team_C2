@echo off
setlocal
set PATH=C:\Windows\system32;%PATH%
for /f %%D in ('wmic volume get DriveLetter^, Label ^| find "FEHSD"') do set usb=%%D

if [%usb%] == [] goto ERROR2
if not exist %usb%\ goto ERROR2
del %usb%\*.S19

xcopy ..\*.s19 %usb%\
IF %ERRORLEVEL% NEQ 0 GOTO COPY_ERROR
echo S19 succesfully copied

move %usb%\*.s19 %usb%\CODE.S19
IF %ERRORLEVEL% NEQ 0 GOTO RENAME_ERROR
echo S19 succesfully renamed
echo ----------------------------------------------
echo Download Successful. Please eject the SD card.
echo ----------------------------------------------
GOTO QUIT

:COPY_ERROR
echo S19 file copy failed 1>&2
exit 1

:ERROR2
echo SD card is not inserted 1>&2
exit 1

:RENAME_ERROR
echo S19 file could not be renamed 1>&2
exit 1

:QUIT
echo