# Sound source localization and SLAM

Welcome to our project repository for TAS 2022WS.    
Member: **Dian Yuan**, Yan Wang, Kaustabh Paul

## Project Description
In the project, we simulated a rescue and search mission. Firstly, we create a set of signal source points in the environment that represent the rescue targets. After that, we let TAS car detect and try to reach some signal source points in the background and simultaneously use SLAM to create the map. Finally, the map should be completed and thus making it the TAS easier to reach the new signal source point.

## Structure
Basically our project has 3 main parts.  
1. **signal_generator package**   Contributor: Yan Wang  
This package create signal source and publishes the **/signal_info** that allows the planner to estimate the signal source position. More detailed information of this package can be found in the [README file](/src/signal_generator/README.md).
2. **planner package**   Contributor: **Dian Yuan**  
This package implentent the algrithm to locate the sound source and build a state machine to drive the car to sound source. It also contains a visualization node for a more intuitive view of the sound source in the rviz interface. More info [here](/src/planner/README.md) 
3. **SLAM and NAV2**   Contributor: Kaustabh Paul  
For details see package [`mapping`](/src/mapping/README.md) and [`tas2-simulator`](/src/tas2-simulator/README.md).
## Getting started
### Environment Configuration
To make sure `tas2-simulator` can successfully run on your computer, you need to have ROS2 Humble installed on Ubuntu22.04.

### Installation Guide
The TAS2 simulator is designed using ROS2 and tested for Ubuntu 22.04.

Following are the steps to install the TAS2 simulator.

- Install ROS2. Check the guidelines [here](https://docs.ros.org/en/humble/index.html) for installation. 
- Install the colcon
  ```
  sudo apt install python3-colcon-common-extensions 
  ```
- create the workspace and src directories. [Further Information Here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
  ```
  mkdir -p ~/ros2_ws/src
  ```
- Clone this repository to the 'src' directory
  ```
  cd ros2_ws/src
  git clone ""link to the repository here""

  ```
- The workspace should have the source code of the TAS2-simulator. Your workspace directory should look like this;
  ```
  ros2_ws
  │   build
  │   install    
  │   log
  └───src
      └───TAS2-simulator
          │   config
          │   launch
          |   maps
          |   models
          |   rviz
          |   CMakeLists.txt
          │   package.xml
  ```
-  Now build the workspace.
    ```
    cd ros2_ws
    colcon build --symlink-install
    ```

- Remember to source your workspace
  ```
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
    echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
  ```
- Before trying the demo, you may need to install extra packages
  ```
    sudo apt install ros-humble-joint-state-publisher
    sudo apt install ros-humble-joint-state-publisher-gui
    sudo apt install ros-humble-xacro
    sudo apt install ros-humble-gazebo-ros-pkgs
    sudo apt install ros-humble-nav2-bringup
    sudo apt install ros-humble-navigation2
    sudo apt install ros-humble-robot-localization
    sudo apt install ros-humble-ackermann-msgs
    sudo apt install ros-humble-ros2-control
    sudo apt install ros-humble-ros2-controllers
    sudo apt install ros-humble-slam-toolbox
    sudo apt-get install ros-humble-tf2-tools 
    sudo apt-get install ros-humble-tf-transformations
    sudo apt-get install ros-humble-nav2-smac-planner
    sudo apt install ros-humble-navigation2
    sudo apt install ros-humble-rviz2  
   ```
### How to run it

Clone the repository and cd to the `ws-g6` folder, run 
> colcon build  


You need to launch 3 terminals to test our project. Remember to source `setup.bash` in each terminal.

- In the first termial, excute
    ```
    ros2 launch tas2-simulator Bringup.py
    ```

- Wait until Gazebo and Rviz are successful loaded(**very important!**) and then excute the following command in second terminal.
    ```
    ros2 launch tas2-simulator navloc.py 
    ```



- Wait until no more info is published in the second terminal.  
    If you'd like to test it with predefined sound source demo(recommended), run 
    ```
    ros2 launch planner predefined_source_demo.py
    ```
    You are expected to see the car start locating and moving to the signal source, at the same time SLAM is being done in parallel, so you will see the map being improved.  
    
    Or you can test with the random generate sound source. Currently there can be some issue with this demo. Refer to the [Issues](https://gitlab.lrz.de/tas_2223/tas-project/group-6/ws-g6/-/blob/main/README.md#issues) part.
    ```
    ros2 launch signal_generator signal_launch.py
    ```
##### Map exploration mode
We also develop a very simple demo for exploring the room without sound source locating and finding tasks.
- In the map exploration mode the robot explores given map autonomously while building the map with SLAM

- In the first termial, excute
    ```
    ros2 launch tas2-simulator Bringup.py
    ```
- In the second termial, excute
    ```
    ros2 run Lscan wanderer
    ```
## Issues
- Although we have tried a lot to tune the parameters to tune the car, sometimes the car does not move as well as we would like. When going through narrow doors, the car sometimes hit the corners. Although we added the backward motion mode to keep the car from getting stuck there, these collisions may affect the SLAM results.
- We only assume two microphones on the cars and only implemented very basic TDoA algorithms, we need the robot current position from the SLAM node. Some time the source position estimation would have relative large error. In this case the robot would try to relocate the signal source. But if localization part (in SLAM) still publish pose with large error(especially orientation), the sound source can never been located and this mission would fail.
- With random source signal generator, we only make sure the sound signal source are generated reachable base on the provided [LSR_N5_basement.pgm](src/tas2-simulator/maps/LSR_N5_basement.pgm), some time the source point can actually never reached, e.g. too close to wall or in a block room. In this situation the task would definitely fail.
- When the sound source is very far, the algorithm usually cannot locate it with only one try (and it is also possible to locate some quite wrong position because of the limitation of the algorithm). It may take mutiple try when finally correct locate it.
- Some time when launch the `Gazebo` you'll find out the car has some offset (as first picture below) compared with the normal position(the second picture).  The reason is unknown yet, but if this happen, we suggest you to clean up your build file: `rm -r build install log` and try to build the project again.

<img src = '/img/issue1.png' width = "40%">  
<img src = '/img/normal.png' width = "40%">  
 
- In our tests, this item requires high CPU performance, and if there is a bottleneck in computer performance it may have an impact on performance.

### Thanks
- Thanks to Chair of Automatic Control Engineering at Technical University of Munich for providing the `tas2-simulator`.  
- Special thanks to Christian Ott for providing TAS car mesh files and Martin Schuck for designing Gazebo world for N5 basement and related URDF/SDF files.
