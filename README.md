# Kinematics plugin for WHOI hybrid gliders
Kinematics control plugin for WHOI hybrid gliders

## Requirements
```diff
- If not using docker environment, use bathymetry_plugin_whoi branch of the dave repo
# at dave repo
git cehckout bathymetry_plugin_whoi
- The IMU/GPS sensor included in this repo requires hector libraries. You may install with following command
sudo apt-get install ros-noetic-hector-gazebo-plugins
- The initial position setter requires python version of gdal
sudo apt-get install python3-gdal
- The kinematics/dynamics plugin uses UwGliderStatus/UwGliderCommand msg to interact with the vehicle
git clone https://github.com/Field-Robotics-Lab/frl_msgs
- nps_uw_sensors_gazebo repository is required
git clone git@github.com:Field-Robotics-Lab/nps_uw_sensors_gazebo.git
```

## Utility guide (live document)
* https://docs.google.com/document/d/1Rlh-2ZkqkKEEsECacgi9XIiPgPHdoRVjJmTLnLg1Bu4/edit?usp=sharing

## How-to
### Installation
* First check to make sure you meet the [System Requirements](https://field-robotics-lab.github.io/dave.doc/contents/installation/System-Requirements/).
* Then choose from one of the following two installation options:
    1. **Directly on Host**
         1. [Install environment and dependent repositories](https://field-robotics-lab.github.io/dave.doc/contents/installation/Install-Directly-on-Host/t) : Instructions to install ROS Noetic, Gazebo 11, UUV Simulator and DAVE directly on your host machine.
         2. Clone this repository in `~/uuv/src` folder and compile with `catkin_make` at `~/uuv_ws` directory.
    2. **Using Docker**
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
        Fore more including docker-compose: [Docker environment description](https://github.com/Field-Robotics-Lab/glider_hybrid_whoi/blob/master/docker/README.MD)
    3. **Using docker-compose**
         1. Run the `build.bash` script located in the `docker` folder of this repository
             ```
             ./build.bash
             ```
         2. Run the container
             ```
             docker-compose up
             ```
         2. Send mission commands from another terminal
             ```
             docker-compose exec glider_extctl_sim /ros_entrypoint.sh rosrun slocum_glider_extctl_sim slocum_glider_sim_console
             # Ctrl+C to land on mission command terminal
             run initbuzz.mi
             run backse01.mi
             ```
         3. Access log CSV file
             ```
             ./join.bash
             tail -f /tmp/Kinematics KinematicsLog.csv
             ```

### Quickstart

* Running the simulator (Run each commands in separate terminal window)
    1. Spawn underwater world with gazebo
        ```bash
        roslaunch glider_hybrid_whoi_gazebo start_demo_kinematics_stratified_current.launch
        # or
        roslaunch glider_hybrid_whoi_gazebo BuzzBay_stratified_current.launch
        # Bathymetry for Buzzbay is included in this repo
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
## Interface
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
  # if in docker environment
  roslaunch glider_hybrid_whoi_gazebo BuzzBay_stratified_current_docker.launch
  ```

### Surface detection
- If the glider reach the surface, the pitch value is set to zero and the position is kept on surface unless it's heading back down.

### Multiple gliders support
- https://github.com/Field-Robotics-Lab/glider_hybrid_whoi/pull/31
- Demo case : `roslaunch glider_hybrid_whoi_gazebo start_demo_kinematics_stratified_current_two_gliders.launch`

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

### Glider dynamic parameters
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

#### Code Structure Diagram
![image](https://user-images.githubusercontent.com/7955120/101485884-2ba8d400-399f-11eb-90ab-6f1be48d3f18.png)
