@ECHO OFF 
cd ..
::for /d %%D in (*) do for /d %%D in (%%D) do echo %%~D
dir /a:d /b /s > ..\fileslist.txt
pause
