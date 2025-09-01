@echo off
echo Cleaning build files...

cd /d "%~dp0"

REM 删除MDK-ARM目录下的编译文件
cd MDK-ARM
if exist can (
    echo Deleting can folder...
    rmdir /s /q can
)

REM 删除其他编译文件
del /q *.bak 2>nul
del /q *.tmp 2>nul
del /q *.log 2>nul

echo Build files cleaned successfully!
echo Please rebuild the project now.
pause
