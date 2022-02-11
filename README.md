# (Hybrid-) AUG Simulator

## Contents
<!-- TOC generated with https://github.com/ekalinin/github-markdown-toc -->
<!--
 cat fls_model_standalone.md | ./gh-md-toc -
-->
* [Summary Poster](#summary-poster)
* [Utility Guide](#utility-guide)
* [Installation](#installation)
* [Quickstart](#Quickstart)
  * [Bundled files](#Bundled-files)
  * [Local custom files](#Local-custom-files)
* [Interface with slocum glider driver](#Interface-with-slocum-glider-driver-simulator)
* [Features](#Features)
  * [Bathymetry included](#Bathymetry-included)
  * [Surface detection](#Surface-detection)
  * [Multiple gliders support](#Multiple-gliderssupport)
  * [Live Feed to Fledermaus](#Live-Feed-to-Fledermaus)
  * [Glider control parameters](#Glider-control-parameters)
  * [Hybrid Glider 6DOF dynamics](#Hybrid-Glider-6DOF-dynamics)

## Summary Poster (2022-02-10)

![Poster](https://user-images.githubusercontent.com/7955120/153356178-ad6c20a5-9418-48f7-b287-292c8a000572.png)

- Parent repo : [Project DAVE](https://github.com/Field-Robotics-Lab/dave)

## Utility guide

* [Utility guide (google doc, live document)](https://docs.google.com/document/d/1Rlh-2ZkqkKEEsECacgi9XIiPgPHdoRVjJmTLnLg1Bu4/edit?usp=sharing)

## Installation

* First check to make sure you meet the [System Requirements](https://github.com/Field-Robotics-Lab/dave/wiki/System-Requirements).
* Then choose from one of the following two installation options:
    1. **Directly on Host**
        1. [Install environment and dependent repositories](https://github.com/Field-Robotics-Lab/dave/wiki/Install-Directly-on-Host) : Instructions to install ROS Noetic, Gazebo 11, UUV Simulator and DAVE directly on your host machine.

            ```diff
            - When cloning the dave repo, bathymetry_plugin_whoi branch from the fork of the dave repo
            git clone https://github.com/woensug-choi/dave.git
            git checkout bathymetry_plugin_whoi
            - The IMU/GPS sensor included in this repo requires hector libraries. You may install with following command
            sudo apt-get install ros-noetic-hector-gazebo-plugins
            - The initial position setter requires python version of gdal
            sudo apt install python3-gdal=3.0.4+dfsg-1build3
            - GPS Viewer requires pyQt and folium modules
            pip3 install folium PyQtWebEngine pyqt5-tools
            - The kinematics/dynamics plugin uses UwGliderStatus/UwGliderCommand msg to interact with the vehicle
            git clone https://github.com/Field-Robotics-Lab/frl_msgs
            - nps_uw_sensors_gazebo repository is required
            git clone git@github.com:Field-Robotics-Lab/nps_uw_sensors_gazebo.git
            ```

        2. Clone this repository in `~/uuv/src` folder and compile with `catkin_make` at `~/uuv_ws` directory.

    2. **Usin`g Docker**
        1. Make sure you have Docker v19.03 or higher ([installation instructions](https://docs.docker.com/engine/install/ubuntu/)) and nvidia-container-toolkit ([installation instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#setting-up-nvidia-container-toolkit))
        2. Run the `build.bash` script located in the `docker` folder of this repository

            ```
            ./build.bash
            ```
        3. Run the container with `run.bash` script

            ```
            ./run.bash
            source ~/glider_hybrid_whoi/install/setup.bash
            ```
        * Opening additional terminals

            ```
            ./join.bash
            source /opt/ros/noetic/setup.bash
            source ~/glider_hybrid_whoi/install/setup.bash
            ```
    Fore more including docker-compose: [Docker environment description](https://github.com/Field`-Robotics-Lab/glider_hybrid_whoi/blob/master/docker/README.MD)

## Quickstart

- This is for running the simulator only

![image](https://user-images.githubusercontent.com/7955120/153543724-512330bb-83af-4c3c-9a3f-3f66f1672fed.png)

### Bundled files
* Running the simulator (Run each commands in separate terminal window)
  1. Spawn underwater world with gazebo

      ```bash
      roslaunch glider_hybrid_whoi_gazebo start_demo_kinematics_stratified_current.launch
      # or
      roslaunch glider_hybrid_whoi_gazebo BuzzBay_stratified_current.launch
      # Bathymetry for Buzzbay is included in this repo.
      # The glider may look as if they are teard apart and shiver. It's rendering error.
      # All sensors should work as if thay are in place just fine.
      # If it hits the surface, it will reset its orietation
      # To dive back down from the sruface, you may need to pich down and push hard.
      # The range is (lat,lon) = (41.50, -70.70) to (41.56, -70.65)
      ```
  2. Control glider with ROS

      ```
      rosrun kinematics_ros_plugins test_directKinematics.py
      ```
  4. Glider status

      ```
      rostopic echo /glider_hybrid_whoi/kinematics/UwGliderStatus
      ```
  5. Access log files

    - Location of the log CSV file : `/tmp/KinematicsLog.csv`
    - Location of the standalone GPS log HTML file : `/tmp/GPSViewer_log.html` (Open with Chrome)

### Local custom files
1. **Directly on Host**
  - There's a template at `local_glider_files_example/bear/simulation`
  - You may launch the launch/world/vehicle description inside it with

    ```bash
    roslaunch glider_hybrid_whoi/local_glider_files_example/bear/simulation/PuertoRico.launch
    # The glider may look as if they are teard apart and shiver. It's rendering error.
    # All sensors should work as if thay are in place just fine.
    ```

2. **Using Docker**
  - If using docker, you need to mount the local files into the docker container
  - instead of using `./run.bash`, launch docker image with `./run-local.bash`
  - The host machine's parent directory of the `docker`, which is `glider_hybrid_whoi`, will be mounted at `/home/ros/local_glider_files`.
  - There's a template at `local_glider_files_example/bear/simulation`
  - You may launch the launch/world/vehicle description inside it with

    ```bash
    ./run-local.bash
    source ~/glider_hybrid_whoi/install/setup.bash
    roslaunch /home/ros/local_glider_files/local_glider_files_example/bear/simulation/PuertoRico.launch
    # The glider may look as if they are teard apart and shiver. It's rendering error.
    # All sensors should work as if thay are in place just fine.
    ```

## Interface with slocum glider driver simulator

For view/edit : [Google Drawing Raw file link](https://docs.google.com/drawings/d/1pj5O0DZ_2o480-9z-qvqAat_yXFckzXrrgTLxbgDyxo/edit?usp=sharing)
![resources](https://docs.google.com/drawings/d/e/2PACX-1vTMQxfsQmqEMxr5fZ30UBqPzI6ULpPdf2XNiF2ak633ty7KP532fMXOgHIJEqI7Z-o-Ok6vdjtgwYdC/pub?w=960&h=720)

The `slocum_glider_sim_driver` and `slocum_glider_sim_console` nodes are
provided by [https://gitlab.com/sentinel-aug/ros/slocum_glider](https://gitlab.com/sentinel-aug/ros/slocum_glider)

## Features

### Bathymetry included
- Buzzbay bathymetry is included
  - Roughly 1500x1500 m tiles with 50 m overlap regions are included (almost 780 MB)
  - Click the `play` button on the Gazebo window and wait for the first bathymetry to be spawned. Next bathymetry tile will be spawned and the previous tile will be removed automatically according to the glider position.

    ```bash
    roslaunch glider_hybrid_whoi_gazebo BuzzBay_stratified_current.launch
    ```

### Surface detection
- If the glider reach the surface, the pitch value is set to zero and the position is kept on surface unless it's heading back down.

### Multiple gliders support
- https://github.com/Field-Robotics-Lab/glider_hybrid_whoi/pull/31
- Demo case : `roslaunch glider_hybrid_whoi_gazebo start_demo_kinematics_stratified_current_two_gliders.launch`

### GPS Viewer
```xml
<!-- GPS Viewer -->
<include file="$(find gps_map_viewer)/launch/gps_map_viewer.launch">
    <arg name="namespace" value="$(arg robot_name)"/>
    <arg name="refresh_rate" value="1.0"/>
    <arg name="save_html" value="True"/>
</include>
```
- `namespace` : name of the vehicle model
- `refresh_rate` : refresh rate for GPS position marker update in the map
- `save_html` : Save GPS log as seen on the viewer as HTML which will be saved at `/tmp/GPSViewer_log.html` which you can open with a browser
- `default_zoom` : set initial zoom (range from 0 to 15, larger mean more zoom)

### Live Feed to Fledermaus
- Live feeding to Fledermaus's Vessel Manager to visualize its location is now available using UDP connection with NMEA strings.
  - Assumes you are running on WSL2 at Windows machine
  - Uses custom NMEA msg format that can include not only lat/lon/depth but also roll/pitch/heading.
  - Networking with WSL is tricky since the Windows and the Ubuntu in WSL recognize `localhost` differently. This is hacked by using the host window machine's public ip.
    Maybe you have to add this to C:\Users/User/.wslconfig

    ```
    [wsl2]
    localhostForwarding=true
    ```
    and restart the WSL by `wsl --shutdown` at cmd.

### Glider control parameters
- Parametes for pitch control, buoyancy induced velocity with the flight model, and thruster power is defined at [glider_hybrid_whoi_base_kinematics.xacro](https://github.com/Field-Robotics-Lab/glider_hybrid_whoi/blob/10524388cce32865ae051e285dbe631ea89159e4/glider_hybrid_whoi_description/urdf/glider_hybrid_whoi_base_kinematics.xacro#L139)

#### Pitch control
| Battery position | Target once | Target servo |
| ------------- | ------------- | ------------- |
| :heavy_check_mark:  | :heavy_check_mark:  | :heavy_multiplication_x: |
- Change of pitch is defined with first-order function with two coefficients (mx + b) for battery position command values
- coefficient name : `f_pitch_battpos_cal_m`, `f_pitch_battpos_cal_b`

#### Thruster control
| Voltage | Power |
| ------------- | ------------- |
| :heavy_check_mark:  | :heavy_check_mark: |
- Thruster power is defined with 2nd-order fucntion with three coefficients (ax^2 + bx + c) for both voltage/power command values
- coefficient name : `f_thruster_voltage_v1`, `f_thruster_voltage_v2`, `f_thruster_voltage_v3`, `f_thruster_power_w1`, `f_thruster_power_w2`, `f_thruster_power_w3`
- Final thrust to the glider is calculated with `sqrt(motorpower/(fluidDensity*Area*C_D)`
- Propeller rotation is visualized using the command value as rotation speed.

#### Rudder control
| Heading | Angle (Center, Port, Stbd) | Angle (Direct) |
| ------------- | ------------- | ------------- |
| :heavy_check_mark:  | :heavy_check_mark: | :heavy_check_mark: |
- The size of the port and starboard angle is set to PI/6.

#### Buoyancy engine control
| Flight model |
| ------------- |
| :heavy_check_mark:  |
- Simple flight model is adopted to change the angle of attach with pumped volume to produce buoyancy induced velocity
- +/- 3 degrees depending on the sign of the pitch
- Calculated with Equations 24 and 28 from Eichhorn et al. paper.

#### Glider status
| Lat/Lon | Roll-Pitch-Yaw | Heading | Depth | Altitude | Power | Rudder angle | Battery position | Pumped Volume | nav_sat_fix |
| ------------- | ------------- | ------------- | ------------- | ------------- | ------------- | ------------- | ------------- | ------------- | ------------- |
| :heavy_check_mark:*  | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark:* | :heavy_multiplication_x: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |
- Lat/Lon is acquired from the GPS sensor and Altitude from the DVL sensor. Which is also sent through nav_sat_fix

### Hybrid Glider 6DOF dynamics
```diff
- It's yet a draft PR
```
- [EPIC-DAUG Glider dynamics Overleaf](https://www.overleaf.com/read/pqstymzccttj)

#### Quickstart
```diff
- Change to 'dynamics_working' branch. `<use_dynamics>` handle is on by default.
```
````bash
roslaunch glider_hybrid_whoi_gazebo start_demo_kinematics_stratified_current.launch
# At another terminal window
rostopic pub /glider_hybrid_whoi/kinematics/UwGliderCommand frl_vehicle_msgs/UwGliderCommand "pitch_cmd_type: 1
target_pitch_value: -0.0001
motor_cmd_type: 2
target_motor_cmd: 0.1
rudder_control_mode: 2
rudder_angle: 4
target_rudder_angle: 0.5
target_pumped_volume: -220.0"
````
![image](https://user-images.githubusercontent.com/7955120/135945104-810f4e2d-aa2e-4ade-aed8-0d7cb216085c.png)
Here, you see the glider ascending and yawing at a constant angle. As it rotates, the roll is also affected as seen in the bottom left camera feed. You can also see that the rotation of the propeller graphically according to the thruster power command input.

- For dynamics calculation,
   - Pitch command type of battery position `frl_vehicle_msgs::UwGliderCommand::PITCH_CMD_BATT_POS` should be used
   - Dynamics flag should be on https://github.com/Field-Robotics-Lab/glider_hybrid_whoi/blob/6d3a6cf6998e5cad4a4460e5b58c564262e5fbd2/glider_hybrid_whoi_description/urdf/glider_hybrid_whoi_base_kinematics.xacro#L162
   - Dynamics properties should be defined at `.xacro` file  https://github.com/Field-Robotics-Lab/glider_hybrid_whoi/blob/6d3a6cf6998e5cad4a4460e5b58c564262e5fbd2/glider_hybrid_whoi_description/urdf/glider_hybrid_whoi_base_kinematics.xacro#L164-L186

#### Limitations and To-dos
- Rudder hydrodynamics coefficients
   - Currently, Rudder hydrodynamics force is modeled with a simple copy&paste of the Hull hydrodynamics (Graver's theory). Area 1/10, Lift and Drag 1/10, Moment *10. Just to check the system stability. There is much to improve how to model this including how to include the effect of ocean current.
- Heading Control
   - Currently, controlling with target heading is not included since it requires a control algorithm for target yaw in the vehicle frame.
- Elevation wing : Not included yet. It could be modeled similar to that of the rudder.

