@echo off
setlocal

set PYTHON_INSTALLER=python-3.7.9-amd64.exe

pushd "%~dp0"
if exist %PYTHON_INSTALLER% (
  start /wait .\%PYTHON_INSTALLER% PrependPath=1 InstallAllUsers=1
  @echo Installing additional python packages for PASTA-CARLA...
  runas /user:%USERNAME% "cmd /k python -m pip install -r %~dp0/requirements.txt & timeout /T 10 & exit"
  @echo Done.
) else (
  @echo No such a file: %PYTHON_INSTALLER%
  @echo Please download %PYTHON_INSTALLER% from https://www.python.org/ and put it in this folder.
)
popd

pause
