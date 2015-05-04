To build the C/C++ library:

windres.exe riffa.rc resource.o
gcc.exe -m64 -c -o riffa.o riffa.c -D RIFFA_EXPORTS
gcc.exe -m64 -o riffa.dll riffa.o resource.o -s -shared -Wl,--subsystem,windows,--out-implib,riffa.lib -luser32 -lsetupapi
del riffa.lib
gendef riffa.dll
lib /def:riffa.def /machine:X64 /out:riffa.lib

This will compile the resource file for the dll (metadata properties for the 
.dll). This will also produce a .dll and .lib file. The .dll is the dynamic
library. The .lib file is the import library. You will want to copy the .dll
into say Windows\System32. You will want to use the .h file and the .lib files
when compiling and linking your applications.
