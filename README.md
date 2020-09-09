# glider_hybrid_whoi
Modifying UUV Simulator's eca_a9 and uuv_gazebo_plugin for Hybrid Gliders with Buyoncy ballast, sliding mass, and Propellers

## Status (using yEd diagram)
#### Blue : Newely added,  Red lines : With problems,  Yellow : To be added later
![alt text](https://github.com/woensug-choi/glider_hybrid_whoi/blob/master/DevelopmentDiagram.png?raw=true)

## GAZEBO vs MATLAB verification (matched.)
![alt text](https://github.com/Field-Robotics-Lab/glider_hybrid_whoi/blob/master/CaseAll.PNG?raw=true)
- to run verification with hard-coded inputs
```bash
roslaunch glider_hybrid_whoi_gazebo empty_underwater_world.launch paused:=true
roslaunch glider_hybrid_whoi_gazebo start_demo_teleop_test.launch joy_id:=0
```
- inputs(pump and battery positions) are defined at [HydrodynamicModel.cc](https://github.com/Field-Robotics-Lab/glider_hybrid_whoi/blob/1ec945d94cb247c326ee1a9fbc3f55c1383ca161/hybrid_glider_gazebo_plugins/hybrid_glider_gazebo_plugins/src/HydrodynamicModel.cc#L436)

## Interface
![alt text](https://github.com/Field-Robotics-Lab/glider_hybrid_whoi/blob/master/uw_glider_interface.png?raw=true)

## Current Problems
https://github.com/Field-Robotics-Lab/glider_hybrid_whoi/issues/7

## Development to-dos
https://github.com/Field-Robotics-Lab/glider_hybrid_whoi/issues/6

## How to run
```bash
roslaunch glider_hybrid_whoi_gazebo empty_underwater_world.launch
roslaunch glider_hybrid_whoi_gazebo start_demo_teleop.launch joy_id:=0
```
