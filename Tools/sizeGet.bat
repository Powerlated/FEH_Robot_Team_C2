@for /f "tokens=1" %%a in (../size.txt) do @set text=%%a
@for /f "tokens=2" %%a in (../size.txt) do @set data=%%a
@for /f "tokens=3" %%a in (../size.txt) do @set bss=%%a

@set /a flash=%text%+%data%
@set /a ram=%data%+%bss%
@set flash_total=474096
@set ram_total=125952
@set /a flash_perc=(%flash%*100/%flash_total%)
@set /a ram_perc=(%ram%*100/%ram_total%)

@echo Flash Usage: %flash% / %flash_total% (%flash_perc%%%)
@echo RAM Usage: %ram% / %ram_total% (%ram_perc%%%)