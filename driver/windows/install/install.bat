@echo off

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

if "%3" == "chk" (
    "c:\program files\inno setup 5\iscc.exe" /dDebug="1" /o.\build .\build\win7.iss 
) else (
    "c:\program files\inno setup 5\iscc.exe" /o.\build .\build\win7.iss 
)
signtool sign /v /s my /n "University of California, San Diego" /t http://timestamp.verisign.com/scripts/timestamp.dll .\build\setup.exe


