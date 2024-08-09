BANNER=\
'

     \u001b[31m⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣤⠂⠀\u001b[0m
     \u001b[31m⠀⠀⠀⣠⣴⠾⠟⠛⠛⢉⣠⣾⣿⠃⠀⠀\u001b[0m
     \u001b[31m⠀⣠⡾⠋⠀⠀⢀⣠⡶⠟⢉⣾⠛⢷⣄⠀\u001b[0m     ███████╗██████╗  █████╗ ██████╗ ██╗  ██╗
     \u001b[31m⣰⡟⠀⢀⣠⣶⠟⠉⠀⢀⣾⠇⠀⠀⢻⣆\u001b[0m     ██╔════╝██╔══██╗██╔══██╗██╔══██╗██║ ██╔╝
     \u001b[31m⣿⠁⠀⠉⠛⠿⢶⣤⣀⠈⠋⠀⠀⠀⠈⣿\u001b[0m     ███████╗██████╔╝███████║██████╔╝█████╔╝
     \u001b[31m⣿⡀⠀⠀⠀⣠⡄⠈⠙⠻⢶⣤⣄⠀⢀⣿\u001b[0m     ╚════██║██╔═══╝ ██╔══██║██╔══██╗██╔═██╗
     \u001b[31m⠸⣧⡀⠀⣰⡿⠀⠀⣠⣴⠿⠋⠀⠀⣼⠏\u001b[0m     ███████║██║     ██║  ██║██║  ██║██║  ██╗
     \u001b[31m⠀⠙⢷⣤⡿⣡⣴⠿⠋⠀⠀⢀⣠⡾⠋⠀\u001b[0m     ╚══════╝╚═╝     ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝
     \u001b[31m⠀⠀⢠⣿⠿⠋⣁⣤⣤⣶⠶⠟⠋⠀⠀⠀\u001b[0m     MICROSYSTEMS
     \u001b[31m⠀⠠⠋⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀\u001b[0m

'
PATH="$HOME/.local/bin:$PATH"

if test -n "$BASH"; then
  SOURCE=$( cd -- "$( dirname -- "$( dirname -- "${BASH_SOURCE[0]}" )" )" &> /dev/null && pwd )
elif test -n "$ZSH_NAME"; then
  SOURCE="$( dirname -- "$( dirname -- "$( readlink -f -- "$0"; )"; )"; )"
else
  echo 'Error : Unable to detect shell. Only bash and zsh are supported'
  return 1
fi

PROJECT_ROOT="$SOURCE"
export PROJECT_ROOT

set_default=false
RELATIVE_ENV_FILE_PATH=""

# Parse command-line arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help)
         echo "Usage: source bootstrap.sh [filename] [-d | --default] [-h | --help]"
         echo "Optional arguments:"
         echo "  filename: YML file to use for environment creation"
         echo "  -d | --default: Set the new environment as the default one"
         echo "  -h | --help: Show this help message"
         return 0
         ;;
        -d|--default)
            set_default=true
            shift
            ;;
        *)
            RELATIVE_ENV_FILE_PATH="$1"
            shift
            ;;
    esac
done

DEFAULT_ENV_FILE_PATH=$SOURCE/.environment/etc/profile.d/default_env

# check if default environment file exists.
if [ -f $DEFAULT_ENV_FILE_PATH ]; then
   # if yes, read it.
   DEFAULT_ENV_PATH=$(cat $DEFAULT_ENV_FILE_PATH)
else
   # if not, create it.
   DEFAULT_ENV_PATH=$SOURCE/script/environment.yml
   mkdir -p $SOURCE/.environment/etc/profile.d
   echo $DEFAULT_ENV_PATH > $DEFAULT_ENV_FILE_PATH
fi

if [[ -n "$RELATIVE_ENV_FILE_PATH" ]]; then
   # if environment file is specified, use it.
   ENV_FILE_PATH="$(realpath "$RELATIVE_ENV_FILE_PATH")"
else
   # if not, use the default one.
   ENV_FILE_PATH=$DEFAULT_ENV_PATH
fi

# Read project name from environment file.
PROJECT_NAME="$( grep '^name:' $ENV_FILE_PATH | sed 's/^name: //' )"

if [ -z "$PROJECT_NAME" ]; then
   echo 'Error : Unable to detect project name. Please check the environment file.'
   return 1
fi

if $set_default; then
   # if default environment is set, update the default environment file.
  echo $ENV_FILE_PATH > $DEFAULT_ENV_FILE_PATH
fi

OS=$(uname -s | tr '[:upper:]' '[:lower:]')

export MAMBA_ROOT_PREFIX="$SOURCE/.environment"
export MAMBA_EXE="$MAMBA_ROOT_PREFIX/bin/micromamba"

if [ -f $MAMBA_ROOT_PREFIX/etc/profile.d/micromamba.sh ]; then
   source $MAMBA_ROOT_PREFIX/etc/profile.d/micromamba.sh
fi

if ! [ -f $MAMBA_ROOT_PREFIX/bin/micromamba ]; then
   mkdir -p $MAMBA_ROOT_PREFIX/etc/profile.d

   echo 'Downloading Micromamba ...'
   cd $MAMBA_ROOT_PREFIX

   if [ $OS = "darwin" ]; then
      if [ "$(uname -m)" = "x86_64" ]; then
         curl -Ls https://micro.mamba.pm/api/micromamba/osx-64/1.0.0 | tar -xvj bin/micromamba
      else
         curl -Ls https://micro.mamba.pm/api/micromamba/osx-arm64/1.0.0 | tar -xvj bin/micromamba
      fi
   elif [ $OS = "linux" ]; then
      if [ "$(uname -m)" = "x86_64" ]; then
         curl -Ls https://micro.mamba.pm/api/micromamba/linux-64/1.0.0 | tar -xvj bin/micromamba
      else
         echo 'Error : Unsupported system'
         return 1
      fi
   else
      echo 'Error : Unsupported system'
      return 1
   fi
   cd -

   eval "$(.environment/bin/micromamba shell hook --shell=posix)"
   printf '%s\n' "$(.environment/bin/micromamba shell hook --shell=posix)" > $MAMBA_ROOT_PREFIX/etc/profile.d/micromamba.sh

   #Disable micromamba banner.
   echo "show_banner: false" >> $MAMBA_ROOT_PREFIX/.condarc
fi

if command -v micromamba &> /dev/null; then
   echo "Initialize virtual environment : $PROJECT_NAME"
   printf '%b\n' "$BANNER"

   #init micromamba environment.
   micromamba create -y --file $ENV_FILE_PATH
   micromamba clean --all --yes
   micromamba activate $PROJECT_NAME

   if [ $OS = "linux" ]; then
      cd $CONDA_PREFIX/lib/
      ln -sf libtinfo.so.6 libtinfo.so.5
      ln -sf libncurses.so.6 libncurses.so.5
      cd -
   fi

   if [ -v ${BOOTSTRAP_SKIP_PYOCD} ]; then
      #Check if pyOCD packages are already installed.
      if [ ! -d "$MAMBA_ROOT_PREFIX/packs" ]; then
         mkdir $MAMBA_ROOT_PREFIX/packs
      fi

      if [ ! -f "$MAMBA_ROOT_PREFIX/packs/Keil.STM32G4xx_DFP.1.5.0.pack" ]; then
         echo "Download STM32G4 pack"
         curl -Ls https://keilpack.azureedge.net/pack/Keil.STM32G4xx_DFP.1.5.0.pack --output $MAMBA_ROOT_PREFIX/packs/Keil.STM32G4xx_DFP.1.5.0.pack
      fi
      if [ ! -f "$MAMBA_ROOT_PREFIX/packs/Keil.STM32U5xx_DFP.2.1.0.pack" ]; then
         echo "Download STM32U5 pack"
         curl -Ls https://keilpack.azureedge.net/pack/Keil.STM32U5xx_DFP.2.1.0.pack --output $MAMBA_ROOT_PREFIX/packs/Keil.STM32U5xx_DFP.2.1.0.pack
      fi

   fi

else
   echo 'Error : micromamba not installed properly'
   return 1
fi

return 0
