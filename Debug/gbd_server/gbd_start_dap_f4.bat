@echo off
chcp 65001 >nul 2>&1
title OpenOCD - STM32F4 (Auto Reconnect)

:: ====== ���� ======
set INTERFACE=interface\cmsis-dap.cfg
set TARGET=target\stm32f4x.cfg
set LOGFILE=%TEMP%\openocd_f4.tmp
set CHECK_INTERVAL=2
:: ==================

taskkill /f /im openocd.exe >nul 2>&1
timeout /t 1 /nobreak >nul

:CONNECT
type nul > "%LOGFILE%" 2>nul

echo.
echo [%time%] ============================================
echo [%time%]  Starting OpenOCD - STM32F4
echo [%time%]  GDB: target remote :3333
echo [%time%]  Ctrl+C to quit
echo [%time%] ============================================
echo.

start "" /b cmd /c openocd.exe -f %INTERFACE% -f %TARGET% ^>"%LOGFILE%" 2^>^&1

echo [%time%] Waiting for OpenOCD to start...
set WAIT_COUNT=0

:WAIT_START
timeout /t 1 /nobreak >nul
set /a WAIT_COUNT+=1

tasklist /fi "imagename eq openocd.exe" 2>nul | find /i "openocd.exe" >nul
if errorlevel 1 (
    echo.
    echo [%time%] OpenOCD failed to start!
    echo [%time%] ---------- Log ----------
    type "%LOGFILE%" 2>nul
    echo [%time%] -------------------------
    echo [%time%] Retry in 5s...
    timeout /t 5 /nobreak >nul
    goto CONNECT
)

find /i "Listening on port" "%LOGFILE%" >nul 2>&1
if not errorlevel 1 (
    echo [%time%] *** Connected successfully! ***
    echo.
    echo [%time%] --- OpenOCD Log ---
    type "%LOGFILE%" 2>nul
    echo.
    echo [%time%] --- Monitoring ---
    goto MONITOR
)

if %WAIT_COUNT% lss 15 goto WAIT_START
echo [%time%] Startup timeout. Monitoring...

:MONITOR
timeout /t %CHECK_INTERVAL% /nobreak >nul

tasklist /fi "imagename eq openocd.exe" 2>nul | find /i "openocd.exe" >nul
if errorlevel 1 (
    echo.
    echo [%time%] OpenOCD process exited!
    goto RECONNECT
)

find /i "error writing data" "%LOGFILE%" >nul 2>&1
if not errorlevel 1 (
    echo.
    echo [%time%] !! DAP-Link disconnected !!
    echo [%time%] Killing OpenOCD...
    taskkill /f /im openocd.exe >nul 2>&1
    timeout /t 1 /nobreak >nul
    goto RECONNECT
)

goto MONITOR

:RECONNECT
echo.
echo [%time%] ========================================
echo [%time%]  Connection lost. Retry in 5s...
echo [%time%]  (Plug DAP-Link back in)
echo [%time%]  Ctrl+C to quit.
echo [%time%] ========================================
echo.
timeout /t 5 /nobreak >nul
goto CONNECT