@ECHO OFF

FOR /F "tokens=*" %%I IN ('tasklist /FI "IMAGENAME EQ LabVIEW.exe"') DO SET TEMP=%%I
SET MATCHSTRING=%TEMP:~0,7%

IF "%MATCHSTRING%"=="LabVIEW" (
	ECHO Found one or more running LabVIEW processes
	ECHO Terminating processes...
	WMIC PROCESS WHERE NAME="LabVIEW.exe" CALL TERMINATE
)ELSE (
	ECHO No running LabVIEW processes found.
)

GOTO END 

:END
REM PAUSE