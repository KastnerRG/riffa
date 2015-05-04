@echo off

set OLDDIR=%CD%
set BDIR1=%CD%\sys\obj%_BUILDTYPE%_%DDK_TARGET_OS%_x86\i386
set BDIR2=%CD%\sys\obj%_BUILDTYPE%_%DDK_TARGET_OS%_amd64\amd64
chdir /d %CD%\install
call install.bat %BDIR1% %BDIR2% %_BUILDTYPE%
chdir /d %OLDDIR%