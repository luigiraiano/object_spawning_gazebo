# Obejct Spawing in Gazebo
This package allows spawning objects within Gazebo Environment

## Spawn an scenario in Gazebo
```
roslaunch object_spawning_gazebo spawn_scenario.launch scenario:=my_scenario
```

- Depending on the information contained within `my_scenario.xml`, a URDF or SDF will be launched using either `spawn_urdf.launch` or `spawn_sdf.launch`

## Ball Trajectile Gazebo World Plugin
To run a demo with a simulated ball trajectile within Gazebo launch the following command:
```
roslaunch object_spawning_gazebo ball_shooter.launch
```
A demo of the simulation can be found at the followinf [link](https://youtu.be/T3CwwptJAtQ).


## Ball Trajectile Gazebo Model Plugin

1. Launch a Gazebo World. For example to launch an empty world:
```
roslaunch gazebo_ros empty_world.launch
```

2. Spawn the ball shooter within gazebo
```
roslaunch object_spawning_gazebo ball_trajectile_shooter.launch launch_rviz:=true
```
- Note: if an RVIZ gui is launched from another node, set the argument `launch_rviz:=false`.

## Gazebo Plugin Turorials
- [Introduction](https://classic.gazebosim.org/tutorials?tut=plugins_hello_world)
- [Model Plugin](https://classic.gazebosim.org/tutorials?tut=plugins_model&cat=write_plugin)


## TODO
- [x] The BallShooter must be a Model Plugin defined within the ball sdf
