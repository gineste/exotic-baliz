@ECHO OFF
CLS

ECHO.
ECHO /**********************************
ECHO  * CREATING HEX FILE (APP AND SD) *
ECHO  **********************************/
ECHO.

ECHO Change Directory...
CD Output

REM ---------------------------------------------------
REM --           Get Version of Application          --
REM ---------------------------------------------------
SETLOCAL
SET Line = ""
SET FirstQuote = 0

FOR /F "tokens=* delims=" %%X in ('type ".\..\Version.h"^| Find /i "#define FW_VERSION "') DO SET Line=%%X > NUL

SET SStrg="^""
SET StrgTemp=%Line%&SET Position=0
:loop
SET /a Position+=1
ECHO %StrgTemp%|FINDSTR /b /c:"%SStrg%" > NUL
IF ERRORLEVEL 1 (
SET StrgTemp=%StrgTemp:~1%
IF DEFINED StrgTemp GOTO loop
SET Position=0
)
SET /a FirstQuote=%Position%

ECHO.%Line%>SVersion
FOR %%a IN (SVersion) DO SET /a SVersion=%%~za -2
SET /a SVersion=%SVersion%-%FirstQuote%-1

CALL SET FirmVersion=%%Line:~%FirstQuote%,%SVersion%%%
DEL SVersion

REM ---------------------------------------------------
REM --         SD, BL and SD_BL Zip Generation       --
REM ---------------------------------------------------
mergehex --merge Softdevice\s132_nrf52_5.0.0_softdevice.hex  ..\Objects\HardwareTest.hex --output Hex\SD_APP_HT_valid_v%FirmVersion%.hex

