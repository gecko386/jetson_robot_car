#!/bin/bash
 
# The following function prints a text using custom color
# -c or --color define the color for the print. See the array colors for the available options.
# -n or --noline directs the system not to print a new line after the content.
# Last argument is the message to be printed.

SCRIPT_PATH=$(dirname $(realpath $0))

PATH=$SCRIPT_PATH:$PATH

cecho () 
{
 
    declare -A colors;
    colors=(\
        ['black']='\E[0;47m'\
        ['red']='\E[0;31m'\
        ['green']='\E[0;32m'\
        ['yellow']='\E[0;33m'\
        ['blue']='\E[0;34m'\
        ['magenta']='\E[0;35m'\
        ['cyan']='\E[0;36m'\
        ['white']='\E[0;37m'\
    );
 
    local defaultMSG="No message passed.";
    local defaultColor="black";
    local defaultNewLine=true;
 
    while [[ $# -gt 1 ]];
    do
    key="$1";
 
    case $key in
        -c|--color)
            color="$2";
            shift;
        ;;
        -n|--noline)
            newLine=false;
        ;;
        *)
            # unknown option
        ;;
    esac
    shift;
    done
 
    message=${1:-$defaultMSG};   # Defaults to default message.
    color=${color:-$defaultColor};   # Defaults to default color, if not specified.
    newLine=${newLine:-$defaultNewLine};
 
    echo -en "${colors[$color]}";
    echo -en "$message";
    if [ "$newLine" = true ] ; then
        echo;
    fi
    tput sgr0; #  Reset text attributes to normal without clearing screen.
 
    return;
}
 
warning () 
{
 
    cecho -c 'yellow' "$@";
}
 
error () 
{
 
    cecho -c 'red' "$@";
}
 
information () 
{
 
    cecho -c 'green' "$@";
}

### number of cores available in system
NUM_CORES=$(grep -c ^processor /proc/cpuinfo) 

### check if current system is a NVIDIA Tegra system
is_tegra () 
{
    query=$(uname -a | cut -f3 -d' ' | cut -f2 -d'-')
    if [ $query == "tegra" ]; then
        return 0
    else
        return 1
    fi
}

get_arch ()
{
    if is_tegra;
    then 
        echo "aarch64"
    else
        echo "x86_64"
    fi
}

exit_if_last_command_failed ()
{
    success=$?
    if [[ $success -ne 0 ]];
    then
        error "$1"
        exit -1
    fi
}

go_to_folder_or_die ()
{
    cd $1
    exit_if_last_command_failed "Cannot access to $1. Please, check logs"
}

get_system_user ()
{
    if is_tegra;
    then
        echo "nvidia"
    else
        _user=$(printf '%s\n' "${SUDO_USER:-$USER}")
        if [[ -z "$_user" ]];
        then
            echo $_user
        else
            whoami
        fi
    fi
}

get_system_user_path ()
{
    _user=$(get_system_user)
    
    if [[ $_user = "root" ]];
    then
        echo "/root"
    else
        echo "/home/$_user"
    fi
}

### git functions

clone_repo () 
{
    SYS_USER=$(get_system_user)
    sudo -u $SYS_USER git clone $1
    success=$?
    if [[ $success -ne 0 ]];
    then
        error "Problems cloning repo. Please, check logs"
        return 1
    fi
    return 0
}

clone_repo_recursive () 
{
    SYS_USER=$(get_system_user)
    sudo -u $SYS_USER git clone --recursive $1
    success=$?
    if [[ $success -ne 0 ]];
    then
        error "Problems cloning repo. Please, check logs"
        return 1
    fi
    return 0
}
