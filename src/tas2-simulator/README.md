## TAS2 simulator

Contributor: Kaustabh Paul

#### Contents

- Changed sdf file of TAScar to enable 360° Lidar measurements with a new maximum range to 16m

- Combined SLAM toolbox online_async_launch.py and GazeboBringupTAScar in Bringup.py

- navloc is used for launching Nav2 without collision monitor

- navloc2 is used for launching Nav2 with collision Monitor

- navloc3 used for launching collision Monitor externaly by running

```
    ros2 launch tas2-simulator collisionM.py
```

##### Nav2 related

- Configured parameters for goal point following, costmaps etc. while running SLAM

(Tested on given maps and maps from https://drive.google.com/drive/folders/1JP12kp4JZ6SM0E8zhxI99ERBMf3qL6QW)

- added collision monitor


### Issues
- configured navigation parameter's functionality depended on which gazebo world was used (basement.world or basement_gr6.world)

- problems with collision monitor and behavior trees

### Source
https://github.com/ros-planning/navigation2


