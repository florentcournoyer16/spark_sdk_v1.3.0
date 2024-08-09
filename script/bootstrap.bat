@echo off
setlocal ENABLEDELAYEDEXPANSION
@REM Enable UTF-8 characters.
call chcp 65001 >nul 2>nul
@REM BANNER
:::
:::     ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣤⠂⠀
:::     ⠀⠀⠀⣠⣴⠾⠟⠛⠛⢉⣠⣾⣿⠃⠀⠀
:::     ⠀⣠⡾⠋⠀⠀⢀⣠⡶⠟⢉⣾⠛⢷⣄⠀     ███████╗██████╗  █████╗ ██████╗ ██╗  ██╗
:::     ⣰⡟⠀⢀⣠⣶⠟⠉⠀⢀⣾⠇⠀⠀⢻⣆     ██╔════╝██╔══██╗██╔══██╗██╔══██╗██║ ██╔╝
:::     ⣿⠁⠀⠉⠛⠿⢶⣤⣀⠈⠋⠀⠀⠀⠈⣿     ███████╗██████╔╝███████║██████╔╝█████╔╝
:::     ⣿⡀⠀⠀⠀⣠⡄⠈⠙⠻⢶⣤⣄⠀⢀⣿     ╚════██║██╔═══╝ ██╔══██║██╔══██╗██╔═██╗
:::     ⠸⣧⡀⠀⣰⡿⠀⠀⣠⣴⠿⠋⠀⠀⣼⠏     ███████║██║     ██║  ██║██║  ██║██║  ██╗
:::     ⠀⠙⢷⣤⡿⣡⣴⠿⠋⠀⠀⢀⣠⡾⠋⠀     ╚══════╝╚═╝     ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝
:::     ⠀⠀⢠⣿⠿⠋⣁⣤⣤⣶⠶⠟⠋⠀⠀⠀     MICROSYSTEMS
:::     ⠀⠠⠋⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
:::

set POWERSHELL=%SYSTEMROOT%\System32\WindowsPowerShell\v1.0\powershell.exe
for %%A in ("%~dp0.") do set ORIGDIR=%%~dpA
set MAMBA_ROOT_PREFIX=%ORIGDIR%\.environment
set MICROMAMBAEXE=%MAMBA_ROOT_PREFIX%\Library\bin\micromamba.exe
set _7ZREXE=%MAMBA_ROOT_PREFIX%\Library\bin\7zr.exe
set _7ZA7Z=%MAMBA_ROOT_PREFIX%\Library\bin\7z2201-extra.7z
set _7ZAEXE=%MAMBA_ROOT_PREFIX%\Library\bin\7z-extra\7za.exe
for %%I in (.) do set PROJECT_NAME=sdk

@REM Create virtual environement folder
if not exist "%MAMBA_ROOT_PREFIX%\Library\bin" mkdir %MAMBA_ROOT_PREFIX%\Library\bin

if not exist %_7ZREXE% (
    echo Downloading 7zip executable ...
    call %POWERSHELL% -Command Set-Variable ProgressPreference SilentlyContinue; Invoke-Webrequest -URI https://www.7-zip.org/a/7zr.exe -OutFile %_7ZREXE%
)

if not exist %_7ZAEXE% (
    echo Downloading 7zip extra ...
    call %POWERSHELL% -Command Set-Variable ProgressPreference SilentlyContinue; Invoke-Webrequest -URI https://www.7-zip.org/a/7z2201-extra.7z -OutFile %_7ZA7Z%

    if not exist %_7ZA7Z% (
        echo Error : 7zip extra download failed
        exit /B 1
    )

    echo Extracting 7zip extra ...
    %_7ZREXE% x %_7ZA7Z% -o%MAMBA_ROOT_PREFIX%\Library\bin\7z-extra  >nul 2>&1
    del %_7ZA7Z%
)

if not exist %MICROMAMBAEXE% (
    echo Downloading Micromamba ...
    pushd .
    cd %MAMBA_ROOT_PREFIX%
    call %POWERSHELL% -Command Set-Variable ProgressPreference SilentlyContinue; Invoke-Webrequest -URI https://micro.mamba.pm/api/micromamba/win-64/1.0.0 -OutFile %ORIGDIR%\micromamba.tar.bz2

    echo Extracting micromamba ...
    %_7ZAEXE% x %ORIGDIR%\micromamba.tar.bz2 -aoa  >nul 2>&1
    del %ORIGDIR%\micromamba.tar.bz2
    %_7ZAEXE% x micromamba.tar -ttar -aoa -r Library\bin\micromamba.exe  >nul 2>&1
    del %MAMBA_ROOT_PREFIX%\micromamba.tar

    popd

    @REM Initialize shell activation script
    if not exist %MICROMAMBAEXE% (
        echo Error : micromamba install failed!
        exit /B 1
    )

    call %MICROMAMBAEXE% -q shell hook --shell=cmd.exe

    if not exist %MAMBA_ROOT_PREFIX%/.condarc (
        @REM Disable micromamba banner and set terminal prompt modifier
        echo env_prompt: ^"^(%PROJECT_NAME%^)^"  > %MAMBA_ROOT_PREFIX%/.condarc
        echo show_banner: false >> %MAMBA_ROOT_PREFIX%/.condarc
    )
)

echo Initialize virtual environment ...

if not defined NO_BANNER (
    @REM Print banner
    for /f "delims=: tokens=*" %%A in ('findstr /b ::: "%~f0"') do @echo(%%A
)
call %MAMBA_ROOT_PREFIX%\condabin\mamba_hook.bat
if not exist %MICROMAMBAEXE% (
    echo Error : micromamba install failed!
    exit /B 1
)

call micromamba create --name %PROJECT_NAME% -y --file %ORIGDIR%\script\environment.yml
call micromamba clean --all --yes
call micromamba activate %PROJECT_NAME%

if not exist "%MAMBA_ROOT_PREFIX%\packs" mkdir %MAMBA_ROOT_PREFIX%\packs

if not exist "%MAMBA_ROOT_PREFIX%\packs\Keil.STM32G4xx_DFP.1.5.0.pack" (
    echo "Download STM32G4 pack"
    call %POWERSHELL% -Command Invoke-Webrequest -URI https://keilpack.azureedge.net/pack/Keil.STM32G4xx_DFP.1.5.0.pack -OutFile %MAMBA_ROOT_PREFIX%\packs\Keil.STM32G4xx_DFP.1.5.0.pack
)
if not exist "%MAMBA_ROOT_PREFIX%\packs\Keil.STM32U5xx_DFP.2.0.0.pack" (
    echo "Download STM32U5 pack"
    call %POWERSHELL% -Command Invoke-Webrequest -URI https://keilpack.azureedge.net/pack/Keil.STM32U5xx_DFP.2.0.0.pack -OutFile %MAMBA_ROOT_PREFIX%\packs\Keil.STM32U5xx_DFP.2.0.0.pack
)

@REM Check for AutoRun registry issues
set "KEY=HKEY_CURRENT_USER\SOFTWARE\Microsoft\Command Processor"
set "VALUE=AutoRun"

for /F "tokens=2*" %%A in ('reg query "%KEY%" /v "%VALUE%" 2^>nul') do set "AutoRun=%%B"

set "Failed=0"

if defined AutoRun (
    for %%Q in (!AutoRun!) do (
        cmd /C "%%~Q" && (
            set "NewAutoRun=!NewAutoRun! "%%~Q""
        ) || (
            set "Failed=1"
        )
    )
    if "!Failed!"=="1" (
        if not "!NewAutoRun!" == "" (
            set "NewAutoRun=!NewAutoRun:~1!"
        )
        echo Your AutoRun registry entry appears to contain invalid commands, which may lead to compilation errors.
        echo.
        echo Original AutoRun: !AutoRun!
        echo Proposed New AutoRun: !NewAutoRun!
        echo.
        echo Please backup the "HKEY_CURRENT_USER\SOFTWARE\Microsoft\Command Processor\AutoRun" registry before proceeding
        set /P Confirmation="Would you like to proceed with the automatic correction process? (Y/N): "
        if /I "!Confirmation!"=="Y" (
            echo Replacing AutoRun commands...
            reg add "%KEY%" /v "%VALUE%" /d "!NewAutoRun!" /f
        ) else (
            echo WARNING : You chose not to proceed with the correction process. This may lead to compilation errors. Run this script again to correct this issue later.
        )
    )
)

exit /B 0
