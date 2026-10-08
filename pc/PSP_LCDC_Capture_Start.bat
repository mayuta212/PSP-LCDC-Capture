@echo off
cd /d "%~dp0"
py -3 bridge.py
if errorlevel 1 (
  echo.
  echo PSP LCDC Capture bridge exited with an error.
  pause
)
