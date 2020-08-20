# glider_hybrid_whoi

Modifying UUV Simulator's eca_a9 and uuv_gazebo_plugin for Hybrid Gliders with Buyoncy ballast, sliding mass, and Propellers

## Development to-dos
https://github.com/woensug-choi/glider_hybrid_whoi/issues/1

## Current Problems
https://github.com/woensug-choi/glider_hybrid_whoi/issues/2


## How to run
```bash
roslaunch uuv_gazebo_worlds ocean_waves.launch
roslaunch glider_hybrid_whoi_gazebo start_demo_teleop.launch joy_id:=0
```
