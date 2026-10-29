@echo off
call "C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul
cd /d "%~dp0"
if not exist build mkdir build
cl /nologo /std:c++17 /EHsc /W4 /Fo:build\ /Fe:build\test_display_model.exe test_display_model.cpp || exit /b 1
build\test_display_model.exe
