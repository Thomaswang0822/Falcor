@echo off
call scripts\windows-bat-runs\common-vars.bat
SET launch_name=SSReSTIR
echo Current run is %launch_name%

%bin_path%\Debug\Mogwai.exe --script=%script_path%\SSReSTIRDemo.py  --scene=%scene_path%\VeachAjar\VeachAjar.pyscene
