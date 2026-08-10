@echo off
cd /d "c:\Users\Joyce_SUN\Desktop\LLM-MoH-DNOP\Multi-UAV-Path-Planning"
"E:\Matlab R2024b\bin\matlab.exe" -nodisplay -nosplash -batch "addpath('c:\Users\Joyce_SUN\Desktop\LLM-MoH-DNOP'); addpath('c:\Users\Joyce_SUN\Desktop\LLM-MoH-DNOP\Multi-UAV-Path-Planning'); verify_roads" > c:\Users\Joyce_SUN\Desktop\LLM-MoH-DNOP\Multi-UAV-Path-Planning\verify_roads_out.log 2>&1
echo EXIT_CODE=%ERRORLEVEL% >> c:\Users\Joyce_SUN\Desktop\LLM-MoH-DNOP\Multi-UAV-Path-Planning\verify_roads_out.log
