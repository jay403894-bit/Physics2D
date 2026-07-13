@echo off
REM Builds AND deploys BOTH Debug (/MDd) and Release (/MD) configs, every time --
REM mirrors T_Threads' own deploy_lib.bat (see C:\Users\jay40\source\repos\T_Threads\T_Threads\deploy_lib.bat).
REM Physics2D links Threads.lib directly (UpdateWorldParallel calls JLib::TaskScheduler::ParallelFor),
REM so its own compile line needs C:\libs\Threads\include too.
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
cd /d "C:\Users\jay40\source\repos\Physics2D\Physics2D"

echo Compiling sources (/MDd debug)...
cl /nologo /c /std:c++20 /EHsc /MDd /Zi /I include /I C:\libs\Threads\include src\SpatialGrid.cpp src\PhysicsWorld.cpp src\PhysicsSystem.cpp
if errorlevel 1 ( echo CL_FAILED & exit /b 1 )
echo Archiving debug lib...
lib /nologo /OUT:Physics2D.lib SpatialGrid.obj PhysicsWorld.obj PhysicsSystem.obj
if errorlevel 1 ( echo LIB_FAILED & exit /b 1 )
echo Deploying debug lib to C:\libs ...
if not exist "C:\libs\Physics2D\lib\debug" mkdir "C:\libs\Physics2D\lib\debug"
copy /Y Physics2D.lib "C:\libs\Physics2D\lib\debug\Physics2D.lib" >nul

echo Compiling sources (/MD /O2 release)...
cl /nologo /c /std:c++20 /EHsc /MD /O2 /DNDEBUG /Zi /I include /I C:\libs\Threads\include src\SpatialGrid.cpp src\PhysicsWorld.cpp src\PhysicsSystem.cpp
if errorlevel 1 ( echo CL_FAILED & exit /b 1 )
echo Archiving release lib...
lib /nologo /OUT:Physics2D_rel.lib SpatialGrid.obj PhysicsWorld.obj PhysicsSystem.obj
if errorlevel 1 ( echo LIB_FAILED & exit /b 1 )
echo Deploying release lib to C:\libs ...
if not exist "C:\libs\Physics2D\lib\release" mkdir "C:\libs\Physics2D\lib\release"
copy /Y Physics2D_rel.lib "C:\libs\Physics2D\lib\release\Physics2D.lib" >nul

xcopy /Y /Q include\*.h "C:\libs\Physics2D\include\" >nul
echo DEPLOY_OK (debug + release)
