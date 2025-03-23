# Notes for integrating Gazebo boat simulator with SITL

## Overview

The objective is to integrate the ArduPilot SITL simulation for sailboats with
Gazebo. The projects involved are:

- [srmainwaring/asv_wave_sim](https://github.com/srmainwaring/asv_wave_sim.git)
- [srmainwaring/asv_sim](https://github.com/srmainwaring/asv_sim.git)
- [srmainwaring/rs750](https://github.com/srmainwaring/rs750)
- [khancyr/ardupilot_gazebo](https://github.com/khancyr/ardupilot_gazebo)
- [ArduPilot/ardupilot](https://github.com/ArduPilot/ardupilot)


For an initial proof of concept version we'd like:

- A Gazebo only version (no ROS dependencies)
- Minimal changes to ArduPilot
- Minimal changes to ardupilot_gazebo
- Launch SITL for a sailboat and connect to a Gazebo session
- Control the rudder and sails from SITL
- SITL to read the Gazebo IMU sensor
- SITL to run a simple waypoint mission and have the boat follow it in Gazebo 
- Provide a docker image

## Approach

### ArduPilot SITL code

We'd like to run a gazebo version of the sailboat simulation, which by analogy
with the version for the `gazebo-rover` frame will look like:

```bash
sim_vehicle.py -v Rover -f gazebo-sailboat -L Mumbles --map --console
```

This requires the following changes:

- Add a new vehicle type to `ardupilot/Tools/autotest/pysim/vehicleinfo.py` that loads the sailboat parameters but is prefixed with `gazebo` so SITL sets up the i/o ports for Gazebo.
- Comparing the update functions in`libraries/SITL/SIM_Gazebo.cpp` and `libraries/SITL/SIM_Sailboat.cpp` the sailboat simulation updates the wind and some associated sensor variables. A proper solution will require Gazebo to read a wind sensor, for the POC
we'll temporarily add the `SIM_Sailboat.cpp` sim code to `SIM_Gazebo.cpp`.
- The physics parameters in the Gazebo sim need calibration / tuning. At low windspeed
the boat will not tack, so make a temporary adjustment to the default params an increase
the windspeed.

### ArduPilot Gazebo code

- The sails need a new controller as SITL provides a sheeting angle which needs to be converted into positive / negative joint angles for the sails. The controller only applies a force when the desired angle is closer to the centre-line than the current position.

### rs750 / asv_wave_sim code

- Switch the waves off for the POC as the gyros etc will not calibrate while the boat is in motion.
- Create a pure SDF model for the rs750 and add the plugin elements for the SITL integration.


## Appendix - code details

***Data updated in SIM_Sailboat:***

```c++
Sailboat::update(...)

  update_wind()

  rpm[0] = wind_apparent_speed
  airspeed_pitot = wind_apparent_speed

  gyro
  dcm
  accel_body
  velocity_ef
  position

  update_position()
  time_advance()
  update_mag_field_bf()
```

***Data updated in SIM_Gazebo:***

```c++
Gazebo::recv_fdm(...)

accel_body
gyro
dcm
velocity_ef
position
time_now_us

Gazebo::update(...)

  send_servos();
  recv_fdm()

  update_position()
  time_advance()
  update_mag_field_bf()

  drain_sockets()
```

***SITL logs for `--frame gazebo-rover`***

```console
$ sim_vehicle.py -v Rover -f gazebo-rover --console --map
SIM_VEHICLE: Start
SIM_VEHICLE: Killing tasks
SIM_VEHICLE: Starting up at SITL location
SIM_VEHICLE: WAF build
SIM_VEHICLE: Configure waf
SIM_VEHICLE: "/home/rhys/Code/ardupilot/ardupilot/modules/waf/waf-light" "configure" "--board" "sitl"
Setting top to                           : /home/rhys/Code/ardupilot/ardupilot 
Setting out to                           : /home/rhys/Code/ardupilot/ardupilot/build 
Autoconfiguration                        : enabled 
Setting board to                         : sitl 
Using toolchain                          : native 
Checking for 'g++' (C++ compiler)        : /usr/bin/g++ 
Checking for 'gcc' (C compiler)          : /usr/bin/gcc 
Checking for c flags '-MMD'              : yes 
Checking for cxx flags '-MMD'            : yes 
Checking for need to link with librt     : not necessary 
Checking for feenableexcept              : yes 
Checking for HAVE_CMATH_ISFINITE         : yes 
Checking for HAVE_CMATH_ISINF            : yes 
Checking for HAVE_CMATH_ISNAN            : yes 
Checking for NEED_CMATH_ISFINITE_STD_NAMESPACE : yes 
Checking for NEED_CMATH_ISINF_STD_NAMESPACE    : yes 
Checking for NEED_CMATH_ISNAN_STD_NAMESPACE    : yes 
Checking for header endian.h                   : yes 
Checking for header byteswap.h                 : yes 
Checking for HAVE_MEMRCHR                      : yes 
Checking for program 'python'                  : /usr/bin/python 
Checking for python version >= 2.7.0           : 2.7.17 
Checking for program 'python'                  : /usr/bin/python 
Checking for python version >= 2.7.0           : 2.7.17 
Source is git repository                       : yes 
Update submodules                              : yes 
Checking for program 'git'                     : /usr/bin/git 
Checking for program 'size'                    : /usr/bin/size 
Benchmarks                                     : disabled 
Unit tests                                     : enabled 
Scripting                                      : enabled 
Scripting runtime checks                       : enabled 
Checking for program 'rsync'                   : /usr/bin/rsync 
'configure' finished successfully (1.214s)
SIM_VEHICLE: Building
SIM_VEHICLE: "/home/rhys/Code/ardupilot/ardupilot/modules/waf/waf-light" "build" "--target" "bin/ardurover"
Waf: Entering directory `/home/rhys/Code/ardupilot/ardupilot/build/sitl'
Waf: Leaving directory `/home/rhys/Code/ardupilot/ardupilot/build/sitl'

BUILD SUMMARY
Build directory: /home/rhys/Code/ardupilot/ardupilot/build/sitl
Target         Text     Data    BSS    Total  
----------------------------------------------
bin/ardurover  2471404  104075  83712  2659191

Build commands will be stored in build/sitl/compile_commands.json
'build' finished successfully (1.122s)
SIM_VEHICLE: Using defaults from (/home/rhys/Code/ardupilot/ardupilot/Tools/autotest/default_params/rover.parm,/home/rhys/Code/ardupilot/ardupilot/Tools/autotest/default_params/rover-skid.parm)
SIM_VEHICLE: Run Rover
SIM_VEHICLE: "/home/rhys/Code/ardupilot/ardupilot/Tools/autotest/run_in_terminal_window.sh" "Rover" "/home/rhys/Code/ardupilot/ardupilot/build/sitl/bin/ardurover" "-S" "--model" "gazebo-rover" "--speedup" "1" "--defaults" "/home/rhys/Code/ardupilot/ardupilot/Tools/autotest/default_params/rover.parm,/home/rhys/Code/ardupilot/ardupilot/Tools/autotest/default_params/rover-skid.parm" "-I0"
SIM_VEHICLE: Run MavProxy
SIM_VEHICLE: "mavproxy.py" "--map" "--console" "--out" "127.0.0.1:14550" "--out" "127.0.0.1:14551" "--master" "tcp:127.0.0.1:5760" "--sitl" "127.0.0.1:5501"
RiTW: Starting Rover : /home/rhys/Code/ardupilot/ardupilot/build/sitl/bin/ardurover -S --model gazebo-rover --speedup 1 --defaults /home/rhys/Code/ardupilot/ardupilot/Tools/autotest/default_params/rover.parm,/home/rhys/Code/ardupilot/ardupilot/Tools/autotest/default_params/rover-skid.parm -I0
# Option “-e” is deprecated and might be removed in a later version of gnome-terminal.
# Use “-- ” to terminate the options and put the command line to execute after it.
WARNING: You should uninstall ModemManager as it conflicts with APM and Pixhawk
Connect tcp:127.0.0.1:5760 source_system=255
[Errno 111] Connection refused sleeping
Loaded module console
Loaded module map
Log Directory: 
Telemetry log: mav.tlog
MAV> Waiting for heartbeat from tcp:127.0.0.1:5760
INITIALISING> Received 1033 parameters (ftp)
```

```console
Set parameter SIM_SPEEDUP to 1.000000
Starting SITL Gazebo
Bind 127.0.0.1:9003 for SITL in
Setting Gazebo interface to 127.0.0.1:9002 
Starting sketch 'Rover'
Starting SITL input
Using Irlock at port : 9005
bind port 5760 for 0
Serial port 0 on TCP port 5760
Waiting for connection ....
Connection on serial port 5760
Loaded defaults from /home/rhys/Code/ardupilot/ardupilot/Tools/autotest/default_params/rover.parm,/home/rhys/Code/ardupilot/ardupilot/Tools/autotest/default_params/rover-skid.parm
Home: -35.363262 149.165237 alt=584.000000m hdg=353.000000
Rate: target:1200.0 achieved:0.0 speedup 0.0/1.0
bind port 5762 for 2
Serial port 2 on TCP port 5762
bind port 5763 for 3
Serial port 3 on TCP port 5763
validate_structures:474: Validating structures
Rate: target:1000.0 achieved:814.8 speedup 0.8/1.0
Rate: target:1000.0 achieved:929.5 speedup 0.9/1.0
```

***SITL logs for `--frame sailboat`***

```console
$ sim_vehicle.py -v Rover -f sailboat --console --map
SIM_VEHICLE: Start
SIM_VEHICLE: Killing tasks
SIM_VEHICLE: Starting up at SITL location
SIM_VEHICLE: WAF build
SIM_VEHICLE: Configure waf
SIM_VEHICLE: "/home/rhys/Code/ardupilot/ardupilot/modules/waf/waf-light" "configure" "--board" "sitl"
Setting top to                           : /home/rhys/Code/ardupilot/ardupilot 
Setting out to                           : /home/rhys/Code/ardupilot/ardupilot/build 
Autoconfiguration                        : enabled 
Setting board to                         : sitl 
Using toolchain                          : native 
Checking for 'g++' (C++ compiler)        : /usr/bin/g++ 
Checking for 'gcc' (C compiler)          : /usr/bin/gcc 
Checking for c flags '-MMD'              : yes 
Checking for cxx flags '-MMD'            : yes 
Checking for need to link with librt     : not necessary 
Checking for feenableexcept              : yes 
Checking for HAVE_CMATH_ISFINITE         : yes 
Checking for HAVE_CMATH_ISINF            : yes 
Checking for HAVE_CMATH_ISNAN            : yes 
Checking for NEED_CMATH_ISFINITE_STD_NAMESPACE : yes 
Checking for NEED_CMATH_ISINF_STD_NAMESPACE    : yes 
Checking for NEED_CMATH_ISNAN_STD_NAMESPACE    : yes 
Checking for header endian.h                   : yes 
Checking for header byteswap.h                 : yes 
Checking for HAVE_MEMRCHR                      : yes 
Checking for program 'python'                  : /usr/bin/python 
Checking for python version >= 2.7.0           : 2.7.17 
Checking for program 'python'                  : /usr/bin/python 
Checking for python version >= 2.7.0           : 2.7.17 
Source is git repository                       : yes 
Update submodules                              : yes 
Checking for program 'git'                     : /usr/bin/git 
Checking for program 'size'                    : /usr/bin/size 
Benchmarks                                     : disabled 
Unit tests                                     : enabled 
Scripting                                      : enabled 
Scripting runtime checks                       : enabled 
Checking for program 'rsync'                   : /usr/bin/rsync 
'configure' finished successfully (1.221s)
SIM_VEHICLE: Building
SIM_VEHICLE: "/home/rhys/Code/ardupilot/ardupilot/modules/waf/waf-light" "build" "--target" "bin/ardurover"
Waf: Entering directory `/home/rhys/Code/ardupilot/ardupilot/build/sitl'
Waf: Leaving directory `/home/rhys/Code/ardupilot/ardupilot/build/sitl'

BUILD SUMMARY
Build directory: /home/rhys/Code/ardupilot/ardupilot/build/sitl
Target         Text     Data    BSS    Total  
----------------------------------------------
bin/ardurover  2471404  104075  83712  2659191

Build commands will be stored in build/sitl/compile_commands.json
'build' finished successfully (1.140s)
SIM_VEHICLE: Using defaults from (/home/rhys/Code/ardupilot/ardupilot/Tools/autotest/default_params/rover.parm,/home/rhys/Code/ardupilot/ardupilot/Tools/autotest/default_params/sailboat.parm)
SIM_VEHICLE: Run Rover
SIM_VEHICLE: "/home/rhys/Code/ardupilot/ardupilot/Tools/autotest/run_in_terminal_window.sh" "Rover" "/home/rhys/Code/ardupilot/ardupilot/build/sitl/bin/ardurover" "-S" "--model" "sailboat" "--speedup" "1" "--defaults" "/home/rhys/Code/ardupilot/ardupilot/Tools/autotest/default_params/rover.parm,/home/rhys/Code/ardupilot/ardupilot/Tools/autotest/default_params/sailboat.parm" "-I0"
SIM_VEHICLE: Run MavProxy
SIM_VEHICLE: "mavproxy.py" "--map" "--console" "--out" "127.0.0.1:14550" "--out" "127.0.0.1:14551" "--master" "tcp:127.0.0.1:5760" "--sitl" "127.0.0.1:5501"
RiTW: Starting Rover : /home/rhys/Code/ardupilot/ardupilot/build/sitl/bin/ardurover -S --model sailboat --speedup 1 --defaults /home/rhys/Code/ardupilot/ardupilot/Tools/autotest/default_params/rover.parm,/home/rhys/Code/ardupilot/ardupilot/Tools/autotest/default_params/sailboat.parm -I0
# Option “-e” is deprecated and might be removed in a later version of gnome-terminal.
# Use “-- ” to terminate the options and put the command line to execute after it.
WARNING: You should uninstall ModemManager as it conflicts with APM and Pixhawk
Connect tcp:127.0.0.1:5760 source_system=255
Loaded module console
Loaded module map
Log Directory: 
Telemetry log: mav.tlog
MAV> Waiting for heartbeat from tcp:127.0.0.1:5760
INITIALISING> Received 1055 parameters (ftp)
Received 1055 parameters (ftp)
MANUAL> 
```

```console
Set parameter SIM_SPEEDUP to 1.000000
Starting sketch 'Rover'
Starting SITL input
Using Irlock at port : 9005
bind port 5760 for 0
Serial port 0 on TCP port 5760
Waiting for connection ....
Connection on serial port 5760
Loaded defaults from /home/rhys/Code/ardupilot/ardupilot/Tools/autotest/default_params/rover.parm,/home/rhys/Code/ardupilot/ardupilot/Tools/autotest/default_params/sailboat.parm
Home: -35.363262 149.165237 alt=584.000000m hdg=353.000000
Rate: target:1200.0 achieved:0.0 speedup 0.0/1.0
bind port 5762 for 2
Serial port 2 on TCP port 5762
bind port 5763 for 3
Serial port 3 on TCP port 5763
validate_structures:474: Validating structures
Rate: target:1200.0 achieved:1208.4 speedup 1.0/1.0
```
