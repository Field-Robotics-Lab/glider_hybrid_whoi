# Kinematics plugin for WHOI hybrid gliders
Kinematics control plugin for WHOI hybrid gliders

## How-to
### Installation
* First check to make sure you meet the [System Requirements](https://github.com/Field-Robotics-Lab/dave/wiki/System-Requirements).
* Then choose from one of the following two installation options:
    1. **Directly on Host**
         1. [Install environment and dependent repositories](https://github.com/Field-Robotics-Lab/dave/wiki/Install-Directly-on-Host) : Instructions to install ROS Melodic, Gazebo 9, UUV Simulator and DAVE directly on your host machine.
         2. Clone this repository in `~/uuv/src` folder and compile with `catkin_make` at `~/uuv` directory.
    2. **Using Docker**
         1. Make sure you have Docker v19.03 or higher ([installation instructions](https://docs.docker.com/engine/install/ubuntu/)) and nvidia-container-toolkit ([installation instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#setting-up-nvidia-container-toolkit))
         2. Run the `build.bash` script located in the `docker` folder of this repository
             ```
             ./build.bash .
             ```
         3. Run the container with `run.bash` script
             ```
             ./run.bash glider_kinematics
             ```
         * Opening additional terminals
             ```
             ./join.bash glider_kinematics
             ```
### Quickstart
* Running the simulator (Run each commands in separate terminal window)
    1. Spawn underwater world with gazebo
        ```
        roslaunch glider_hybrid_whoi_gazebo seafloor_underwater_world.launch
        ```
    2. Spawn glider into the gazebo world
        ```
        roslaunch glider_hybrid_whoi_gazebo start_demo_teleop_kinematics.launch joy_id:=0
        ``` 
    3. Control glider with ROS
        ```
        # Using test python script
        rosrun direct_kinematics_ros_plugins test_directKinematics.py
        # Directly to the ros node
        # Toward surface
        rostopic pub /glider_hybrid_whoi/direct_kinematics/UwGlidCommand frl_vehicle_msgs/UwGliderCommand '{motor_cmd_type: 1, target_motor_cmd: 2, pitch_cmd_type: 2, target_pitch_value: -0.4, rudder_control_mode: 1, target_heading: 0.8}'
        # Back to underwater
        rostopic pub /glider_hybrid_whoi/direct_kinematics/UwGlidCommand frl_vehicle_msgs/UwGliderCommand '{motor_cmd_type: 1, target_motor_cmd: 2, pitch_cmd_type: 2, target_pitch_value: 0.4, rudder_control_mode: 1, target_heading: -0.8}'
        ```
    4. Glider status
        ```
        rostopic echo /glider_hybrid_whoi/direct_kinematics/UwGliderStatus
        ```

## Features
* Organized based on support for [frl_vehicle_msgs](https://github.com/Field-Robotics-Lab/frl_msgs/tree/master/frl_vehicle_msgs/msg)
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
