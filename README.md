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

## Gazebo Plugin Turorials
- [Introduction](https://classic.gazebosim.org/tutorials?tut=plugins_hello_world)
- [Model Plugin](https://classic.gazebosim.org/tutorials?tut=plugins_model&cat=write_plugin)


## TODO
- [ ] The BallShooter must be a Model Plugin defined within the ball sdf
