:: 使用UTF-8编码
:: chcp 65001 >nul

@echo off
setlocal enabledelayedexpansion

:: 验证工具存在性
where CoolFormat.exe >nul 2>nul
if errorlevel 1 (
    echo 错误：CoolFormat.exe未安装或未添加到PATH环境变量
    pause
    exit /b 1
)

:: 递归处理文件
for /r %%i in (*.c *.cpp *.h *.hpp *.cc *.cxx) do (
    echo 格式化: %%~nxi
    :: echo CoolFormat.exe -f "%%i"
    CoolFormat.exe -f "%%i"
)

echo 所有C/C++文件已格式化完成
:: pause

