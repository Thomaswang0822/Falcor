@echo off
call scripts\windows-bat-runs\common-vars.bat
SET launch_name=RTXDI
echo Current run is %launch_name%

%bin_path%\Debug\Mogwai.exe --script=%script_path%\RTXDI.py --scene=%Falcor_root%\media\Arcade\Arcade.pyscene
