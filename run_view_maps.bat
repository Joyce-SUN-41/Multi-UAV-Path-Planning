@echo off
cd /d "c:\Users\Joyce_SUN\Desktop\LLM-MoH-DNOP\Multi-UAV-Path-Planning"
"E:\Matlab R2024b\bin\matlab.exe" -batch "addpath('c:\Users\Joyce_SUN\Desktop\LLM-MoH-DNOP'); addpath('c:\Users\Joyce_SUN\Desktop\LLM-MoH-DNOP\Multi-UAV-Path-Planning'); view_maps('png')" > c:\Users\Joyce_SUN\Desktop\LLM-MoH-DNOP\Multi-UAV-Path-Planning\view_maps_out.log 2>&1
echo EXIT_CODE=%ERRORLEVEL% >> c:\Users\Joyce_SUN\Desktop\LLM-MoH-DNOP\Multi-UAV-Path-Planning\view_maps_out.log
