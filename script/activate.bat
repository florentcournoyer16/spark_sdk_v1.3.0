@echo off
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

for %%A in ("%~dp0.") do set ACTIVATEPATH=%%~dpA
for %%A in ("%~dp0\..") do set PROJECT_NAME=sdk

set MAMBA_ROOT_PREFIX=%ACTIVATEPATH%.environment

echo Initialize virtual environment ...

if not defined NO_BANNER (
    @REM Print banner
    for /f "delims=: tokens=*" %%A in ('findstr /b ::: "%~f0"') do @echo(%%A
)

@REM Check if using the offline environment (package created with conda-pack). If so, use the standard activation script and return.
if exist %ACTIVATEPATH%.environment\envs\sdk\offline.txt (
    call %ACTIVATEPATH%.environment\envs\sdk\Scripts\activate.bat
    call %ACTIVATEPATH%.environment\envs\sdk\Scripts\conda-unpack.exe
) else (
    call %MAMBA_ROOT_PREFIX%\condabin\mamba_hook.bat
    call micromamba activate %PROJECT_NAME%
)

if not %errorlevel% == 0 (
    echo "Virtual environment not found. Please Run bootstrap.bat."
)

exit /b 0
