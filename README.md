# glider_hybrid_whoi
Modifying UUV Simulator's eca_a9 and uuv_gazebo_plugin for Hybrid Gliders with Buyoncy ballast, sliding mass, and Propellers

## Status (using yEd diagram)
#### Blue : Newely added,  Red lines : With problems,  Yellow : To be added later
![alt text](https://github.com/woensug-choi/glider_hybrid_whoi/blob/master/DevelopmentDiagram.png?raw=true)

## Current Problems
https://github.com/Field-Robotics-Lab/glider_hybrid_whoi/issues/7

## Development to-dos
https://github.com/Field-Robotics-Lab/glider_hybrid_whoi/issues/6

## How to run
```bash
roslaunch uuv_gazebo_worlds ocean_waves.launch
roslaunch glider_hybrid_whoi_gazebo start_demo_teleop.launch joy_id:=0
```
