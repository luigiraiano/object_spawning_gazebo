# Obejct Spawing in Gazebo
This package allows spawning objects within Gazebo Environment

## Spawn an scenario in Gazebo
```
roslaunch object_spawning_gazebo spawn_scenario.launch scenario:=my_scenario
```

- Depending on the information contained within `my_scenario.xml`, a URDF or SDF will be launched using either `spawn_urdf.launch` or `spawn_sdf.launch`
