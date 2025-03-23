#!/bin/bash

# Set Gazebo version for future compatibility
export GZ_VERSION=garden

# Set Gazebo IP address, defaults to localhost if not required
export GZ_IP=127.0.0.1

# Append additional paths to GZ_SIM_RESOURCE_PATH for models and world files
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:\
$HOME/gz_ws/src/asv_wave_sim/gz-waves-models/models:\
$HOME/gz_ws/src/asv_wave_sim/gz-waves-models/world_models:\
$HOME/gz_ws/src/asv_wave_sim/gz-waves-models/worlds

# Append additional paths to GZ_SIM_SYSTEM_PLUGIN_PATH for system plugins
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:\
$HOME/gz_ws/install/lib

# Append additional paths to GZ_GUI_PLUGIN_PATH for GUI plugins
export GZ_GUI_PLUGIN_PATH=$GZ_GUI_PLUGIN_PATH:\
$HOME/gz_ws/src/asv_wave_sim/gz-waves/src/gui/plugins/waves_control/build

# Append additional paths for BlueBoat models and worlds
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:\
$HOME/SITL_Models/Gazebo/models:\
$HOME/SITL_Models/Gazebo/worlds

# Check to make sure the script is being sourced and not directly executed
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "This script needs to be sourced to set environment variables correctly."
    echo "Please run 'source $0' instead of executing it directly."
    exit 1
fi

export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH

