# Insane movements found on UUV simulator's original plugin !!
```bash
roslaunch uuv_gazebo_worlds ocean_waves.launch
roslaunch eca_a9_description upload_eca_a9.launch
```
which uploads eca_a9 model of UUV simulator with its plugins. The AUV moves upward gradually when uploaded with neutural buoyancy option turned off. When neuturally_buoyancy option is turned on, it drawns slowly (same problem with modified hybrid glider plugin). HOWEVER, if waited long enough, the model suddenly make very insane movements without any external forces applied. Don't know whats causing it. Seems building the kinematics from the scrtach would be easier than debugging uuv simulator.

# glider_hybrid_whoi
Modifying UUV Simulator's eca_a9 and uuv_gazebo_plugin for Hybrid Gliders with Buyoncy ballast, sliding mass, and Propellers


## Status (using yEd diagram)
#### Blue : Newely added,  Red lines : With problems,  Yellow : To be added later
![alt text](https://github.com/woensug-choi/glider_hybrid_whoi/blob/master/DevelopmentDiagram.png?raw=true)

## Current Problems
https://github.com/woensug-choi/glider_hybrid_whoi/issues/2

## Development to-dos
https://github.com/woensug-choi/glider_hybrid_whoi/issues/1

## How to run
```bash
roslaunch uuv_gazebo_worlds ocean_waves.launch
roslaunch glider_hybrid_whoi_gazebo start_demo_teleop.launch joy_id:=0
```
