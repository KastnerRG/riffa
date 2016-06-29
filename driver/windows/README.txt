To build the Windows driver:

1) Install Windows Driver Development Kit supporting Windows 7 (tested on 
   version 7600.16385.1).
2) Open a DDK command window environment for Windows 7 (which ever version 
   you're targeting).
3) Move to the directory containing this README.txt and run build -ceZ
4) The driver should be built and ready in the output directory along with a
   Windows 7 catalog file and the coinstaller DLLs.
5) To build the installer you will need to build the driver using the DDK for
   each architecture (x86/x64). If you want both setup.exe and setup_dbg.exe
   executables, you will run the build command FOUR TIMES before step 6.
6) To build the setup.exe file, run the win7install.bat script from the DDK
   unchecked/free command window. To build the setup_dbg.exe file, run the 
   script from the checked command window.
  
A few notes:

- You will need to sign the driver (riffa.sys) and catalog file (riffa.cat) 
  before you can install it on a x64 Windows 7 or Vista computer. The build 
  process will attempt to sign the catalog file with the UCSD certificate. You
  don't have that, so you won't get a signed driver simply by building. You'll
  need to get a certificate from a certificate authority that is capable of 
  cross-certificate kernel driver signing to authenticate yourself (.pfx), 
  and the cross-signing certificate from that authority (.crt file available 
  from link). These should both be added to the windows certificate list and
  and copied into the root folder for the windows driver (same location as this
  README.txt file). See this page for more details:
  http://msdn.microsoft.com/en-us/windows/hardware/gg487315.aspx
    
- Debugging on Windows is difficult because there exists no kernel log file.
  Drivers are supposed to log messages using a trace events framework which is
  overly complex and requires developer tools to first collect the output and
  then more (different) tools to make the output human readable. Instead, this
  driver writes normal log messages via a kernel debugger facility. To see the
  messages you'll need the Windows Development Kit debugger (WinDbg) or a small
  utility called DbgView. DbgView is a standalone kernel debug viewer that can
  be downloaded from Microsoft here:
  http://technet.microsoft.com/en-us/sysinternals/bb896647.aspx
  Just start with administrator privileges and be sure to enable Capture Kernel, 
  Capture Events, and Capture Verbose Kernel Output.

- Building with the checked environment will produce a version of the driver
  with verbose debugging output. Building with the free environment will 
  produce a version of the driver with minimal messaging output. The debug 
  version will have a "(Debug)" label in the Windows device manager, so you 
  can tell which version is installed.

- Inno Setup scripts produce a Windows Installer. You may use our script if you
  like.
