@echo on

rmdir /s /q build
md build
md build\x86
md build\x64

copy win7.iss .\build
copy license.txt .\build

xcopy /E /H /K /I /Y %1 .\build\x86
xcopy /E /H /K /I /Y %2 .\build\x64
xcopy /E /H /K /I /Y ..\..\..\c_c++\windows .\build\c_c++
xcopy /E /H /K /I /Y ..\..\..\java .\build\java
xcopy /E /H /K /I /Y ..\..\..\python .\build\python
xcopy /E /H /K /I /Y ..\..\..\matlab .\build\matlab
echo "%3"

if "%3" == "chk" (
	md .\build.\tmp_dbg
    "c:\program files (x86)\inno setup 5\iscc.exe" /dDebug="1" /o.\build\tmp_dbg .\build\win7.iss
	signtool sign /v /ac "..\GlobalSign Root CA.crt" /s my /n "University of California, San Diego" /t http://timestamp.verisign.com/scripts/timestamp.dll .\build\tmp_dbg\setup.exe
	move .\build\tmp_dbg\setup.exe .\setup_dbg.exe 	
	rmdir /s /q .\build\tmp_dbg\
) else (
	md .\build\tmp
    "c:\program files (x86)\inno setup 5\iscc.exe" /o.\build\tmp\ .\build\win7.iss 
	signtool sign /v /ac "..\GlobalSign Root CA.crt" /s my /n "University of California, San Diego" /t http://timestamp.verisign.com/scripts/timestamp.dll .\build\tmp\setup.exe
	move .\build\tmp\setup.exe .\setup.exe
	rmdir /s /q .\build\tmp\
)



