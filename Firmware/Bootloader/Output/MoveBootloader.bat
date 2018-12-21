@ECHO OFF
CLS

CD Output 
COPY ..\Objects\Bootloader.hex Hex\Bootloader.hex

COPY Hex\Bootloader.hex ..\..\AppBase\Output\Bootloader
