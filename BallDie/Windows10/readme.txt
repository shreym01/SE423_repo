To install in Windows 10 follow these steps

copy msstdfmt.dll to c:\windows\system32  and to c:\windows\syswow64

then make sure to open a CMD  "Run as Administrator"

Then run four commands   paste into command prompt

%windir%\System32\regsvr32.exe /u Msstdfmt.dll

%windir%\SysWoW64\regsvr32.exe /u msstdfmt.dll

%windir%\System32\regsvr32.exe /i msstdfmt.dll  ignore the error message

%windir%\SysWoW64\regsvr32.exe /i msstdfmt.dll   ignore the error message


Found this at web site  www.dlldownloader.com/msstdfmt-dll/