@echo off
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
cd /d "C:\Users\jay40\source\repos\Physics2D\smoketest"
cl /nologo /std:c++20 /EHsc /MDd /Zi /I "C:\libs\Physics2D\include" /I "C:\libs\Threads\include" main.cpp /link /LIBPATH:"C:\libs\Physics2D\lib\debug" /LIBPATH:"C:\libs\Threads\lib\debug" Physics2D.lib Threads.lib /OUT:smoketest.exe
