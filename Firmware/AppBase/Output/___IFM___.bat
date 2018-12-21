@ECHO OFF

SET JLINK_V=JLink_V
SET JLINK_VERSION=
SET SEGGER_Path=C:\Program Files (x86)\SEGGER

:START
CLS

ECHO /**********************************
ECHO  *   INTERFACE HFLGBTI MACHINE    *
ECHO  **********************************/
ECHO.
ECHO 1. Install Tools
ECHO 2. Programm uC
ECHO 3. Launch tests
ECHO 4. Start Bootloader
ECHO x. Exit
ECHO.
SET /p Cmd=Enter command : 
ECHO.

IF %Cmd% EQU 1 (
   GOTO INSTALL_TOOLS
) ELSE IF %Cmd% EQU 2 (
   GOTO PROGRAMM_HEX
) ELSE IF %Cmd% EQU 3 (
   GOTO JLINKRTT
) ELSE IF %Cmd% EQU 4 (
   GOTO BOOTLOADER
)ELSE GOTO END


:INSTALL_TOOLS
ECHO.
ECHO /**********************************
ECHO  *   INSTALLATION OF SOFTWARES    *
ECHO  **********************************/
ECHO.
REM IF NOT EXIST "C:\Program Files (x86)\SEGGER\JLink_V622g" (
REM    ECHO Installation of JLink in Progress, please follow steps...
REM    START /W Softwares/JLink_Windows_V622g.exe
REM ) ELSE (
REM    ECHO SEGGER JLink already installed !
REM )

IF NOT EXIST "C:\Program Files (x86)\Nordic Semiconductor\nrf5x\bin" (
   ECHO Installation of nRF Utilities in Progress, please follow steps...
   START /W Softwares/nRF5x-Command-Line-Tools_9_7_2_Installer.exe
) ELSE (
   ECHO nRF5x Utilities already installed !
)

IF NOT EXIST "%SEGGER_Path%" (
   ECHO Installation of JLink in Progress, please follow steps...
   START /W Softwares/JLink_Windows_V622g.exe
   SET JLINK_VERSION=JLink_V622g
   ECHO %JLINK_VERSION%
) ELSE (
   ECHO SEGGER JLink already installed !
)
PAUSE
GOTO START


:PROGRAMM_HEX
ECHO.
ECHO /**********************************
ECHO  *  PROGRAM GLOBAL HEX FOR TESTS  *
ECHO  **********************************/
ECHO.
CALL nrfjprog -f NRF52 --program GlobalHex\SD_APP.hex --verify --chiperase --reset
REM CALL nrfjprog -f NRF52 --program GlobalHex\SD_BL_APP_STG.hex --verify --chiperase --reset
PAUSE
GOTO START

:JLINKRTT
ECHO.
ECHO /**********************************
ECHO  *     OPEN JLINK RTT VIEWER      *
ECHO  **********************************/
ECHO.
REM Set local Path and variables
REM IF %JLINK_VERSION% EQU 1 (
   REM SET JLINKRTT_Path=C:\Program Files (x86)\SEGGER\JLink_V620i
REM ) ELSE IF %JLINK_VERSION% EQU 2 (
   REM SET JLINKRTT_Path=C:\Program Files (x86)\SEGGER\JLink_V622g
REM ) ELSE (
   REM ECHO JLink RTT Viewer Not Installed !
REM )
      
      
IF EXIST "C:\Program Files (x86)\SEGGER\JLink_V622g" (
   SET JLINK_VERSION=JLink_V622g
) ELSE IF EXIST "C:\Program Files (x86)\SEGGER\JLink_V620i" (
   SET JLINK_VERSION=JLink_V620i
) ELSE (
   ECHO Add Your version to the script
   PAUSE
   GOTO END
)   
ECHO %SEGGER_Path%\%JLINK_VERSION%\JLinkRTTViewer.exe
REM SET JLINKRTT_Path=C:\Program Files (x86)\SEGGER\JLink_V622g
REM SET JLINKRTT_Path=C:\Program Files (x86)\SEGGER\JLink_V620i
CALL nrfjprog -f NRF52 --reset
ECHO Running JLink RTT Viewer
REM CALL "%JLINKRTT_Path%\JLinkRTTViewer.exe" -s 9600 -a
CALL "%SEGGER_Path%\%JLINK_VERSION%\JLinkRTTViewer.exe" -s 9600 -a
GOTO END

:BOOTLOADER
ECHO Stay pressed on board's button and tape any key
PAUSE
CALL nrfjprog -f NRF52 --reset
PAUSE
GOTO END

:COMPARE
REM Compare with Global Hex
ECHO Comparing Hex Files...
FC /B ReadCode.hex GlobalHex\SD_BL_HT_STG.hex > NUL
IF ERRORLEVEL 0 (
   ECHO.
   ECHO Right Hex Already programmed !
   CALL nrfjprog -f NRF52 --reset
   GOTO JLINKRTT
) ELSE IF ERRORLEVEL 1 (
   ECHO Programming...
   GOTO PROG
) ELSE (
   ECHO Cannot find files !
   GOTO END
)
   
:END