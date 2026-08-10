@echo off
"E:\Matlab R2024b\bin\matlab.exe" -batch "try; addpath('c:/Users/Joyce_SUN/Desktop/LLM-MoH-DNOP/Multi-UAV-Path-Planning'); addpath('c:/Users/Joyce_SUN/Desktop/LLM-MoH-DNOP'); diary('view_scenes.log'); view_scenes; diary off; catch e; disp(getReport(e,'extended')); end; quit"
