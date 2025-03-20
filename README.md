# Marine Robotics Simulation Framework

This repository provides a simulation framework for evaluating motion control in tethered multi-robot systems within dynamic marine environments.<br>
Specifically, it focuses on the coordinated operation of an Autonomous Underwater Vehicle (AUV) and an Autonomous Surface Vehicle (ASV).<br>
The framework leverages GazeboSim, enhanced with realistic marine environment plugins and ArduPilot's Software-In-The-Loop (SITL) mode.<br>

SITL allows you to simulate the vehicle hardware and firmware ([ArduSub](https://www.ardusub.com/) ) on your host directly.


![image](https://github.com/user-attachments/assets/64d262c0-5a9e-4808-a12e-3bfa3e6461a4)



## Prerequisites

- Download and Install [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html) (optional).
- Install [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) to support Docker to access GPU (required).
- Repository has been tested on: Ubuntu 22.04, Ubuntu 24.04, ArchLinux (Kernel 6.8).


## Build

```bash
git clone https://github.com/markusbuchholz/marine-robotics-sim-framework/git

cd marine-robotics-sim-framework//bluerov2_ardupilot_SITL/docker

sudo ./build.sh

```

## Build in Docker

Adjust in ```run.sh```.

```bash
local_gz_ws="/home/markus/bluerov2_ardupilot_SITL/gz_ws"
local_SITL_Models="/home/markus/bluerov2_ardupilot_SITL/SITL_Models"
```

```bash
sudo ./run.sh

colcon build

source install/setup.bash

cd ../gz_ws

colcon build --symlink-install --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON -DCMAKE_CXX_STANDARD=17

source install/setup.bash

source gazebo_exports.sh
 
```
---

## Run Gazebo BlueRov2 simulator

Note:
- IMPORTANT: Run Gazebo first as it contains the ArduPilot plugin. Later, start the ArduPilot SITL (Rover) simulator. Lastly, you can run QGroundControl.

```bash
ros2 launch move_blueboat launch_robot_simulation.launch.py
```

You can alway connect to running Docker container from other terminal,

```bash
sudo docker exec -it bluerov2_sitl /bin/bash

```
---
## Run SITL

Notes:

- The flag ```-l``` is the localization (lat,lon,alt,heading). Check your favorite location with Google Maps.
- ```sim_vehicle.py --help ``` -prints all available commands and flags.
- in ```run.sh``` adjust these two lines for your host specific:


```bash

sudo docker exec -it simulation_sitl /bin/bash

cd ../ardupilot

sim_vehicle.py -v ArduSub -f vectored_6dof --model JSON --map --console -l 55.99541530863445,-3.3010225004910683,0,0

# if you need to recompile 
Tools/environment_install/install-prereqs-ubuntu.sh -y

# after recompiling 
. ~/.profile

```
---
## BlueBoat SITL


```bash

ros2 run joy joy_node

colcon build --symlink-install --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON -DCMAKE_CXX_STANDARD=17

ros2 launch move_blueboat launch_robot_simulation.launch.py

sim_vehicle.py -v Rover -f gazebo-rover --model JSON --map --console -l 55.99541530863445,-3.3010225004910683,0,0 -I1

cd /home/blueboat_sitl/gz_ws/extras_interface
python3 ros2_blueboat_interface.py

```

---
## Tether modeling

We consider a tether which is composed of ```N``` spheres, each of mass m and radius ```r```. 
The spheres are connected sequentially, with each pair of adjacent spheres connected by a ball-type joint.
The joint allows the spheres to rotate relative to each other in two perpendicular directions, simulating the flexibility of an actual underwater tether.
We consider the tether to hold the ```catenary``` shape. 


```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="tether_part">        
    <plugin
        filename="gz-waves1-hydrodynamics-system"
        name="gz::sim::systems::Hydrodynamics">
        <!-- Adjust buoyancy settings for near-neutral buoyancy -->
        <buoyancy>
          <volume>0.000025</volume>
          <center_of_buoyancy>0 0 0</center_of_buoyancy>
          <mass_density>1000</mass_density> 
        </buoyancy>
    </plugin>
    <link name="base_link">
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>0.025</radius> <!-- Size of the buoy -->
          </sphere>
        </geometry>
        <material>
          <ambient>1.0 0.0 0.0 1.0</ambient>  <!-- Bright red -->
          <diffuse>1.0 0.0 0.0 1.0</diffuse>  <!-- Bright red -->
          <specular>0.8 0.8 0.8 1.0</specular> <!-- Slightly reflective -->
        </material>
      </visual>
      
      <collision name="collision">
        <geometry>
          <sphere>
            <radius>0.01</radius> <!-- Smaller collision radius -->
          </sphere>
        </geometry>
      </collision>
      
      <inertial>
        <mass>0.00429</mass> <!-- Mass adjusted for neutral buoyancy -->
        <inertia>
          <!-- Recalculated based on the solid sphere inertia formula -->
          <ixx>0.00000006</ixx> <!-- 2/5 * mass * radius^2 -->
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.00000006</iyy>
          <iyz>0.0</iyz>
          <izz>0.00000006</izz>
        </inertia>
      </inertial>
    </link>
  </model>
</sdf>

```


## Simulate the position of the tether

Run,
```bash
cd extras_tether

python3 dynamic_update_cabel_pos.py
```



![image](https://github.com/user-attachments/assets/885abed6-2334-42dc-bf4f-20a5430df920)



---
## Start QGC (outside Docker)

```bash
./QGroundControl.AppImage
```

## Move BlueRov2

Move the vehicle using Ardusub to navigate the designated waypoints, utilizing either ROS 2 or ArduPilot.

```bash
# It is recommended to Arm the vehicle first

arm throttle
```

```bash
ros2 launch move_blueboat wp_rov
```

```bash
cd  /gz_ws/src/extras_rov

python3 wp_pos_req.py
```

## Change the Motion Velocity

The following program, before motion, changes the ArduSub parameters influencing the global velocity motion for the vehicle.
- ```WPNAV_SPEED``` - for horizontal speed
- ```WPNAV_SPEED_DN``` - for descent speed
- ```WPNAV_SPEED_UP``` - for climb speed

```bash
ros2 launch move_blueboat wp_velo_rov
```

## PlotJuggler

```bash
ros2 run plotjuggler plotjuggler
```

## Acknowledgement

- [blue](https://github.com/Robotic-Decision-Making-Lab/blue)
- [orca4](https://github.com/clydemcqueen/orca4)

## References

- [ROS-Gazebo Bridge](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge)
- [Ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo)
- [Gazebo demos](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_sim_demos)
- [BlueBoat operational manual](https://bluerobotics.com/learn/blueboat-operators-guide/)
- [QGroundControl User Guide](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/)
- [BlueBoat SITL](https://github.com/ArduPilot/SITL_Models/blob/master/Gazebo/docs/BlueBoat.md)
- [ArduPilot and Gazebo](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)
- [SITL](https://www.ardusub.com/developers/sitl.html)
- [Wave Sim](https://github.com/srmainwaring/asv_wave_sim)
- [icra2023_ros2_gz_tutorial](https://github.com/osrf/icra2023_ros2_gz_tutorial?tab=readme-ov-file#overview)
- [Gazebo Ocean Simulation](https://docs.google.com/presentation/d/1JXwWMPPVT7y03Vr6cWtrwAwyvdysmg3NW9ZmI276dMY/edit#slide=id.p)
- [Multi-LRAUV Simulation](https://docs.google.com/presentation/d/1RIuvOOTdQvoAKKRzGnNZW8Ikp_VGVaUOyAZc-BANXdo/edit#slide=id.g71c89e7412_2_52)
- [rover-sitl](https://ardupilot.org/dev/docs/rover-sitlmavproxy-tutorial.html)
- [cruise-throttle-and-cruise-speed](https://ardupilot.org/rover/docs/rover-tuning-throttle-and-speed.html#cruise-throttle-and-cruise-speed) - refer to the doc. for instructions on how to set global velocity and acc.
- [Rover: L1 navigation overview](https://ardupilot.org/dev/docs/rover-L1.html)
- [Extended Kalman Filter Navigation Overview and Tuning](https://ardupilot.org/dev/docs/extended-kalman-filter.html)
- [Rover Control Modes](https://ardupilot.org/rover/docs/rover-control-modes.html)
- [Dynamic position mode](https://ardupilot.org/rover/docs/loiter-mode.html)
- [flight-modes](https://ardupilot.org/copter/docs/flight-modes.html)
- [common-non-gps-navigation](https://ardupilot.org/copter/docs/common-non-gps-navigation-landing-page.html)
- [Guided Mode](https://ardupilot.org/copter/docs/ac2_guidedmode.html)
- [ArduPilot-ROVER](https://ardupilot.org/rover/index.html)
- [GPS / Non-GPS Transitions](https://ardupilot.org/copter/docs/common-non-gps-to-gps.html)
- [EKF Failsafe](https://ardupilot.org/copter/docs/ekf-inav-failsafe.html)
- [tuning-navigation](https://ardupilot.org/rover/docs/rover-tuning-navigation.html)



