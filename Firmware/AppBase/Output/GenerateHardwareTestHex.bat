REM Program with SoftDevice and Bootloader
REM Read Memory and Create Setting.hex
REM Merge Setting.hex with SoftDevice and Bootloader
REM CALL nrfutil pkg generate --hw-version 52 --bootloader-version 1 --bootloader Bootloader.hex --sd-req 0x9D --key-file private.key Bootloader.zip
REM mergehex --merge Bootloader.hex Settings.hex s132_nrf52_5.0.0_softdevice.hex --output BLStgSDv5_SDK14.hex

@ECHO OFF
CLS

ECHO.
ECHO /*********************************
ECHO  *  CREATING HEX FILE FOR TESTS  *
ECHO  *********************************/
ECHO.

ECHO Change Directory...
CD Output

REM REMOVE PREVIOUS Hex Files
IF EXIST *.hex (
   ECHO Deleting Hex Files...
   DEL *.hex
)

REM Create the settings.hex file
CALL nrfutil settings generate --family NRF52 --application ..\Objects\HardwareTest.hex --application-version 1 --bootloader-version 1 --bl-settings-version 1 Stg.hex

REM Combine the softdevice with the Bootloader
mergehex --merge SoftDevice\s132_nrf52_5.0.0_softdevice.hex Bootloader\Bootloader.hex --output SD_BL.hex

REM Add the app to it
mergehex --merge SD_BL.hex ..\Objects\HardwareTest.hex --output SD_BL_HT.hex

REM Add the settings file to the Softdevice/bootloader/application hex
mergehex --merge SD_BL_HT.hex Stg.hex --output GlobalHex\SD_BL_APP_STG.hex

IF EXIST Stg.hex (
   DEL Stg.hex
)
IF EXIST SD_BL.hex (
   DEL SD_BL.hex
)
IF EXIST SD_BL_HT.hex (
   DEL SD_BL_HT.hex
)
