for /f %%D in ('wmic volume get DriveLetter^, Label ^| find "FEHSD"') do set usb=%%D
del %usb%\*.S19
xcopy ..\*.s19 %usb%\
move %usb%\*.s19 %usb%\CODE.S19