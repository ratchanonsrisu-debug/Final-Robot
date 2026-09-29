@echo off
chcp 65001 >nul
cd /d "%~dp0"
set "VENV=D:\Cha sahdu\github\RoboMaster-241-251\.venv\Scripts\python.exe"
if exist "%VENV%" ( "%VENV%" run.py %* & goto end )
py -3.8 run.py %*
:end
pause
