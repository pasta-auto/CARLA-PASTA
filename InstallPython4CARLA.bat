@echo off
setlocal

set PYTHON_INSTALLER=python-3.7.9-amd64.exe

pushd "%~dp0"
if exist %PYTHON_INSTALLER% (
  start /wait .\%PYTHON_INSTALLER% PrependPath=1
  @echo Installing additional python packages for PASTA-CARLA...
  python -m pip install -r .\requirements.txt
  @echo Done.
) else (
  @echo No such a file: %PYTHON_INSTALLER%
  @echo Please download %PYTHON_INSTALLER% from https://www.python.org/ and put it in this folder.
)
popd

pause
