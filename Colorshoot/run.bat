@echo off
chcp 65001 >nul
rem Open the dashboard. robomaster SDK needs Python 3.7-3.8.
rem Python search order: RoboMaster project venv (has SDK) -> py -3.8 -> python
cd /d "%~dp0"
set "VENV=D:\Cha sahdu\github\RoboMaster-241-251\.venv\Scripts\python.exe"
if exist "%VENV%" ( "%VENV%" run.py %* & goto end )
py -3.8 --version >nul 2>&1 && ( py -3.8 run.py %* & goto end )
python run.py %*
:end
pause
