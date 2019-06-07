@ECHO OFF
CLS

ECHO.
ECHO /*********************************
ECHO  * CREATING PACKAGE FILE FOR DFU *
ECHO  *********************************/
ECHO.

ECHO Change Directory...
CD Output


REM ---------------------------------------------------
REM --           Get Version of Application          --
REM ---------------------------------------------------
REM Combine the softdevice with the Bootloader
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
REM Combine the softdevice with the Bootloader
mergehex --merge Softdevice\s132_nrf52_5.0.0_softdevice.hex  Bootloader\Bootloader.hex --output SD_BL.hex
COPY SD_BL.hex Hex\SD_Bootloader.hex

REM ---------------------------------------------------
REM --    Settings for HardwareTest and SF Generation       --
REM ---------------------------------------------------
REM Create the settings.hex file
CALL nrfutil settings generate --family NRF52 --application ..\Objects\HardwareTest.hex --application-version 1 --bootloader-version 1 --bl-settings-version 1 settings_HT.hex

REM Add the app to it
mergehex --merge SD_BL.hex ..\Objects\HardwareTest.hex --output SD_BL_APPHT.hex

REM Add the settings file to the Softdevice/bootloader/application hex
mergehex --merge SD_BL_APPHT.hex settings_HT.hex --output Hex\SD_BL_APP_HT_valid_v%FirmVersion%.hex

DEL SD_BL.hex
DEL settings_HT.hex
DEL SD_BL_APPHT.hex

