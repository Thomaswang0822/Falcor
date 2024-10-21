@echo off
call scripts\windows-bat-runs\common-vars.bat
SET launch_name=PathTracer
echo Current run is %launch_name%

%bin_path%\Debug\Mogwai.exe --script=%script_path%\PathTracer.py  --scene=%scene_path%\VeachAjar\VeachAjarAnimated.pyscene
