@echo off
call scripts\windows-bat-runs\common-vars.bat
SET launch_name=ReSTIR PT
echo Current run is %launch_name%

%bin_path%\Debug\Mogwai.exe --script=%script_path%\ReSTIRPTDemo.py
