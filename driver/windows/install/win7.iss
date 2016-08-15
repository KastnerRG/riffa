; -- 64Bit.iss --
; Demonstrates installation of a program built for the x64 (a.k.a. AMD64)
; architecture.
; To successfully run this installation and the program it installs,
; you must have a "x64" edition of Windows.

; SEE THE DOCUMENTATION FOR DETAILS ON CREATING .ISS SCRIPT FILES!
#ifdef Debug
#define DebugMsg1 "%n%nNOTE: This version has been compiled to output additional debug messages (with some performance overhead)."
#else
#define DebugMsg1 ""
#endif

[Setup]
AppName=RIFFA
AppVersion=2.2.2
AppPublisher=University of California, San Diego
AppPublisherURL=https://sites.google.com/a/eng.ucsd.edu/matt-jacobsen/riffa
AppCopyright=Copyright (C) 2016 The Regents of the University of California. All Rights Reserved.
LicenseFile=license.txt
PrivilegesRequired=admin
MinVersion=6.1
OnlyBelowVersion=6.2
DisableProgramGroupPage=yes
Compression=lzma2
SolidCompression=yes
UsePreviousAppDir=no
DefaultDirName={pf}\Riffa
OutputDir=outputdir
; "ArchitecturesInstallIn64BitMode=x64" requests that the install be
; done in "64-bit mode" on x64, meaning it should use the native
; 64-bit Program Files directory and the 64-bit view of the registry.
ArchitecturesInstallIn64BitMode=x64

[Messages]
WelcomeLabel1=Welcome to the [name] Setup Wizard
WelcomeLabel2=This will install the [name/ver] FPGA drivers and C/C++ bindings on your computer.{#DebugMsg1}%n%nSee the install program directory for details on installing other language bindings.%n%nIt is recommended that you close all other applications and disable any anti virus before continuing.
FinishedLabelNoIcons=Setup has finished installing [name/ver] on your computer.%n%nAny [name] compatible FPGA devices should be detected upon reboot.

[Dirs]
Name: "{app}\c_c++"; Permissions: users-modify
Name: "{app}\java"; Permissions: users-modify
Name: "{app}\python"; Permissions: users-modify
Name: "{app}\matlab"; Permissions: users-modify

[Files]
Source: "c_c++\x86\riffa.lib"; DestDir: "{app}\c_c++"; DestName: "riffa32.lib"
Source: "c_c++\x86\riffa.h"; DestDir: "{app}\c_c++"; Check: "not IsWin64"
Source: "c_c++\x86\sample_app\README.txt"; DestDir: "{app}\c_c++"; Check: "not IsWin64"
Source: "c_c++\x86\sample_app\timer.h"; DestDir: "{app}\c_c++"; Check: "not IsWin64"
Source: "c_c++\x86\sample_app\testutil.c"; DestDir: "{app}\c_c++"; Check: "not IsWin64"
Source: "c_c++\x86\sample_app\testutil.exe"; DestDir: "{app}\c_c++"; Check: "not IsWin64"
Source: "c_c++\x64\riffa.lib"; DestDir: "{app}\c_c++"; DestName: "riffa64.lib"; Check: IsWin64
Source: "c_c++\x64\riffa.h"; DestDir: "{app}\c_c++"; Check: IsWin64
Source: "c_c++\x64\sample_app\README.txt"; DestDir: "{app}\c_c++"; Check: IsWin64
Source: "c_c++\x64\sample_app\timer.h"; DestDir: "{app}\c_c++"; Check: IsWin64
Source: "c_c++\x64\sample_app\testutil.c"; DestDir: "{app}\c_c++"; Check: IsWin64
Source: "c_c++\x64\sample_app\testutil.exe"; DestDir: "{app}\c_c++"; Check: IsWin64

Source: "java\README.txt"; DestDir: "{app}\java"
Source: "java\riffa.jar"; DestDir: "{app}\java"
Source: "java\SampleApp.java"; DestDir: "{app}\java"

Source: "matlab\README.txt"; DestDir: "{app}\matlab"
Source: "matlab\Riffa.m"; DestDir: "{app}\matlab"

Source: "python\dist\README.txt"; DestDir: "{app}\python"
Source: "python\dist\riffa-2.0.zip"; DestDir: "{app}\python"
Source: "python\sample_app\sampleapp.py"; DestDir: "{app}\python"

Source: "x86\riffa.sys"; DestDir: "{tmp}"; Check: "not IsWin64"
Source: "x86\riffa.inf"; DestDir: "{tmp}"; Check: "not IsWin64"
Source: "x86\riffa.cat"; DestDir: "{tmp}"; Check: "not IsWin64"
Source: "x86\WdfCoInstaller01009.dll"; DestDir: "{tmp}"; Check: "not IsWin64"
Source: "x86\WdfCoInstaller01009_chk.dll"; DestDir: "{tmp}"; Check: "not IsWin64"
Source: "x64\riffa.sys"; DestDir: "{tmp}"; Check: IsWin64
Source: "x64\riffa.inf"; DestDir: "{tmp}"; Check: IsWin64
Source: "x64\riffa.cat"; DestDir: "{tmp}"; Check: IsWin64
Source: "x64\WdfCoInstaller01009.dll"; DestDir: "{tmp}"; Check: IsWin64
Source: "x64\WdfCoInstaller01009_chk.dll"; DestDir: "{tmp}"; Check: IsWin64

Source: "c_c++\x86\riffa.dll"; DestDir: {sys}; Flags: 32bit
Source: "c_c++\x64\riffa.dll"; DestDir: {sys}; Flags: 64bit; Check: IsWin64

[Run]
Filename: "{sys}\pnputil.exe"; Parameters: " -i -a {tmp}\riffa.inf"; WorkingDir: "{tmp}"; Description: "Install driver"; StatusMsg: "Installing drivers..."; Flags: runascurrentuser runhidden;

