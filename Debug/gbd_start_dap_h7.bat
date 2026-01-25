%关闭命令回显%
@ echo off
%打印提示信息%
echo Openocd Runing.........
%执行OpenOCD服务%
openocd.exe -f interface\cmsis-dap.cfg -f target\stm32h7x.cfg
%防止控制台窗口关闭%
pause