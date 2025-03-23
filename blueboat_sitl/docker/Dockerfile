FROM osrf/ros:humble-desktop AS base

ARG USERNAME=blueboat_sitl
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get upgrade -y

# Install a few handy tools
RUN apt-get update \
  && apt-get -y --quiet --no-install-recommends install \
    bash-completion \
    build-essential \
    git \
    glmark2 \
    gnupg \
    iputils-ping \
    lsb-release \
    mlocate \
    software-properties-common \
    sudo \
    wget \
    vim \
  && rm -rf /var/lib/apt/lists/*

# Install NVIDIA software
RUN apt-get update \
 && apt-get -y --quiet --no-install-recommends install \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
  && rm -rf /var/lib/apt/lists/*
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV QT_X11_NO_MITSHM=1

# Install some ardupilot and ardupilot_gazebo prereqs
RUN apt-get update \
 && apt-get -y --quiet --no-install-recommends install \
    python3-wxgtk4.0 \
    rapidjson-dev \
    xterm \
  && rm -rf /var/lib/apt/lists/*

# Install Gazebo Garden and other required packages
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
  && apt-get update \
  && apt-get -y --quiet --no-install-recommends install \
    libgz-sim7-dev \
    libopencv-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    gz-garden \
  && rm -rf /var/lib/apt/lists/*

# Set up rosdep for Gazebo Garden
RUN sudo bash -c 'wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list' \
  && rosdep update

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && echo "\n# Added by blueboat_sitl Dockerfile:" >> /home/$USERNAME/.bashrc \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc

# Switch to our new user
USER $USERNAME
ENV USER=$USERNAME
ENV GZ_VERSION=garden

# Clone ArduSub code
WORKDIR /home/$USERNAME
RUN git clone https://github.com/ArduPilot/ardupilot.git --recurse-submodules

# Checkout the commit which works fdor BlueROV2
WORKDIR /home/$USERNAME/ardupilot
RUN git checkout aca9940534


# Install ArduSub prereqs (this also appends to .bashrc)
WORKDIR /home/$USERNAME/ardupilot
ENV SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1
RUN Tools/environment_install/install-prereqs-ubuntu.sh -y

# Build ArduSub
WORKDIR /home/$USERNAME/ardupilot
RUN modules/waf/waf-light configure --board sitl \
  && modules/waf/waf-light build --target bin/ardusub

# Clone ardupilot_gazebo code
WORKDIR /home/$USERNAME
RUN git clone https://github.com/ArduPilot/ardupilot_gazebo.git

# Build ardupilot_gazebo
RUN [ "/bin/bash" , "-c" , " \
  cd ardupilot_gazebo \
  && mkdir build \
  && cd build \
  && cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  && make -j4" ]

# Create colcon workspace and copy workspace.repos file
WORKDIR /home/$USERNAME
RUN mkdir -p colcon_ws/src/blueboat_sitl
COPY --chown=$USER_UID:$USER_GID blueboat_sitl.repos colcon_ws/src/blueboat_sitl

# Get workspace repos
WORKDIR /home/$USERNAME/colcon_ws/src
RUN vcs import < blueboat_sitl/blueboat_sitl.repos

# Run rosdep update before installing dependencies
RUN rosdep update

# Downgrade setuptools to a compatible version
RUN python3 -m pip install --upgrade pip setuptools==59.6.0

# Run rosdep over workspace repos
RUN rosdep install -y --from-paths . --ignore-src --skip-keys="gz-transport12 gz-sim7 gz-math7 gz-msgs9"

# Build everything so far
WORKDIR /home/$USERNAME/colcon_ws
RUN [ "/bin/bash" , "-c" , "\
  source /opt/ros/humble/setup.bash \
  && colcon build" ]

# Run rosdep over blueboat package.xml files
RUN rosdep update \
  && rosdep install -y --from-paths . --ignore-src --skip-keys="gz-transport12 gz-sim7 gz-math7 gz-msgs9"

# MAVROS depends on GeographicLib, and GeographicLib needs some datasets
RUN [ "/bin/bash" , "-c" , "\
  wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
  && chmod +x install_geographiclib_datasets.sh \
  && sudo ./install_geographiclib_datasets.sh" ]

# Copy everything
COPY --chown=$USER_UID:$USER_GID . src/blueboat_sitl

# Build everything
RUN [ "/bin/bash" , "-c" , "\
  source /opt/ros/humble/setup.bash \
  && colcon build" ]

# Set up the environment
WORKDIR /home/$USERNAME/colcon_ws
RUN echo "export PATH=/home/$USERNAME/.local/bin:\$PATH" >> /home/$USERNAME/.bashrc \
  && echo "export PATH=/home/$USERNAME/.local/lib/python3.10/site-packages:\$PATH" >> /home/$USERNAME/.bashrc \
  && echo "source /home/$USERNAME/colcon_ws/src/blueboat_sitl/setup.bash" >> /home/$USERNAME/.bashrc

USER root
RUN apt-get update && apt-get install -y nano
RUN apt-get -y --quiet --no-install-recommends install \
 libcgal-dev \
 libfftw3-dev

RUN python3 -m pip install opencv-python

RUN apt-get update && apt-get install -y \
    ros-humble-joy \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-humble-joy-linux \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install transforms3d pymavlink transformations geopy

ENV DIST=humble
RUN apt update \
 && apt install -y --no-install-recommends \
    ros-${DIST}-plotjuggler-ros \
 && apt clean

RUN apt-get update && apt-get install -y \
    qml-module-qtpositioning \
    qml-module-qtlocation

RUN apt-get install -y \
    python3-tk    

RUN python3 -m pip install gekko  

USER $USERNAME

USER root

RUN apt-get update && apt-get install -y ros-humble-joint-state-publisher ros-humble-ros2-control

# Install MAVROS, MAVROS Extras, and geographic_msgs
RUN apt-get update && apt-get install -y \
  ros-humble-mavros \
  ros-humble-mavros-extras \
  ros-humble-geographic-msgs \
  ros-humble-tf-transformations \
  && /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# Create or modify the apm_pluginlists.yaml
RUN mkdir -p /opt/ros/humble/share/mavros/launch && \
    echo "/mavros/**:\n  ros__parameters:\n    plugin_denylist:\n      # common\n      - actuator_control\n      - ftp\n      - hil\n      # extras\n      - altitude\n      - debug_value\n      - image_pub\n      - px4flow\n      - vibration\n      - vision_speed_estimate\n      - wheel_odometry\n    plugin_allowlist:\n      - 'fake_gps'\n" > /opt/ros/humble/share/mavros/launch/apm_pluginlists.yaml

# Create or modify the apm_config.yaml
RUN echo "# fake_gps\n/mavros/**/fake_gps:\n  ros__parameters:\n    # select data source\n    use_mocap: true         # ~mocap/pose\n    mocap_transform: false  # ~mocap/tf instead of pose\n    mocap_withcovariance: false  # ~mocap/pose uses PoseWithCovarianceStamped Message\n    use_vision: false       # ~vision (pose)\n    use_hil_gps: true\n    gps_id: 4\n    # origin (default: Zürich)\n    geo_origin.lat: 47.3667          # latitude [degrees]\n    geo_origin.lon: 8.5500           # longitude [degrees]\n    geo_origin.alt: 408.0            # altitude (height over the WGS-84 ellipsoid) [meters]\n    eph: 2.0\n    epv: 2.0\n    horiz_accuracy: 0.5\n    vert_accuracy: 0.5\n    speed_accuracy: 0.0\n    satellites_visible: 6   # virtual number of visible satellites\n    fix_type: 3             # type of GPS fix (default: 3D)\n    tf.listen: false\n    tf.send: false           # send TF?\n    tf.frame_id: \"map\"       # TF frame_id\n    tf.child_frame_id: \"fix\" # TF child_frame_id\n    tf.rate_limit: 10.0      # TF rate\n    gps_rate: 5.0           # GPS data publishing rate\n" > /opt/ros/humble/share/mavros/launch/apm_config.yaml


RUN apt-get update && apt-get install -y wmctrl


USER $USERNAME

