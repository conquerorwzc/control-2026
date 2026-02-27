@echo off
chcp 65001 >nul 2>&1
title OpenOCD - STM32H7 (Auto Reconnect)

:: ====== 配置 ======
set INTERFACE=interface\cmsis-dap.cfg
set TARGET=target\stm32h7x.cfg
set LOGFILE=%~dp0openocd_h7.log
set CHECK_INTERVAL=2
:: ==================

:: 清理残留
taskkill /f /im openocd.exe >nul 2>&1
timeout /t 1 /nobreak >nul

:CONNECT
:: 清空日志
type nul > "%LOGFILE%" 2>nul

echo.
echo [%time%] ============================================
echo [%time%]  Starting OpenOCD - STM32H7
echo [%time%]  GDB: target remote :3333
echo [%time%]  Ctrl+C to quit
echo [%time%] ============================================
echo.

:: 后台启动 OpenOCD，所有输出写入日志
start "" /b cmd /c openocd.exe -f %INTERFACE% -f %TARGET% ^>"%LOGFILE%" 2^>^&1

:: 等待启动
echo [%time%] Waiting for OpenOCD to start...
set WAIT_COUNT=0

:WAIT_START
timeout /t 1 /nobreak >nul
set /a WAIT_COUNT+=1

:: 进程还在吗？
tasklist /fi "imagename eq openocd.exe" 2>nul | find /i "openocd.exe" >nul
if errorlevel 1 (
    echo.
    echo [%time%] OpenOCD failed to start!
    echo [%time%] ---------- Log ----------
    type "%LOGFILE%" 2>nul
    echo [%time%] -------------------------
    echo [%time%] DAP-Link not found? Retry in 5s...
    timeout /t 5 /nobreak >nul
    goto CONNECT
)

:: 检查是否已经监听端口（说明连接成功）
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

:: 最多等 15 秒
if %WAIT_COUNT% lss 15 goto WAIT_START

echo [%time%] Startup timeout, but process alive. Monitoring...

:: ====== 主监控循环 ======
:MONITOR
timeout /t %CHECK_INTERVAL% /nobreak >nul

:: 检查1: OpenOCD 进程还活着吗？
tasklist /fi "imagename eq openocd.exe" 2>nul | find /i "openocd.exe" >nul
if errorlevel 1 (
    echo.
    echo [%time%] OpenOCD process exited!
    goto RECONNECT
)

:: 检查2: 日志中是否出现断开错误？
find /i "error writing data" "%LOGFILE%" >nul 2>&1
if not errorlevel 1 (
    echo.
    echo [%time%] !! DAP-Link disconnected !!
    echo [%time%] Killing OpenOCD...
    taskkill /f /im openocd.exe >nul 2>&1
    timeout /t 1 /nobreak >nul
    goto RECONNECT
)

:: 一切正常，继续监控
goto MONITOR

:RECONNECT
echo.
echo [%time%] ========================================
echo [%time%]  Connection lost.
echo [%time%]  Retrying in 5s...
echo [%time%]  (Plug DAP-Link back in)
echo [%time%]  Ctrl+C to quit.
echo [%time%] ========================================
echo.
timeout /t 5 /nobreak >nul
goto CONNECT