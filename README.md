## Short Introduction

There are **three workspaces**:

* `bfmc_workspace`, which contains the necessary files for running the simulator
* `startup_workspace`, where is provided some useful libraries for running our Python code in the Gazebo simulator
* `vroom_workspace`, which containes all the necessary files for running our implemented algorithms

## How to Start

- Install [Ubuntu 18.04.5 LTS (Bionic Beaver)](https://releases.ubuntu.com/18.04/). For best performances, avoid using a virtual machine.
- Install [ROS Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu). We suggest using the Desktop Full Install. Make sure you follow all the instructions and pay attention to the ***Initialize rosdep*** subchapter.
- Install [Gazebo9](https://zoomadmin.com/HowToInstall/UbuntuPackage/gazebo9).
- Install [Redis Server](https://redis.io/topics/quickstart).

### Workspace Setup 

Clone this repository inside the `Documents` folder


### SETUP Guide

`cd Documents/VROOM-BFMC-Simulator/bfmc_workspace`<br />
`catkin_make`<br />
`source devel/setup.bash`<br />


- Gazebo needs to know where the workspace's packages and models are:
    - export `models_pkg` to the `GAZEBO_MODEL_PATH` variable
    - export the `src` folders to `ROS_PACKAGE_PATH` variable

```sh
# You can either run these commands each time you want to use the workspace, or add them to the end of the `~/.bashrc` file and source it.
# Don't forget to replace {YOUR_USER} with your actually user name
export GAZEBO_MODEL_PATH="/home/{YOUR_USER}/Documents/VROOM-BFMC-Simulator/bfmc_workspace/src/models_pkg:$GAZEBO_MODEL_PATH"
export ROS_PACKAGE_PATH="/home/{YOUR_USER}/Documents/VROOM-BFMC-Simulator/bfmc_workspace/src:$ROS_PACKAGE_PATH"
```


`cd ..`<br />
`cd startup_workspace`<br />
`catkin_make`<br />
`source devel/setup.bash`<br /><br />
`cd ..` <br />
`cd vroom_workspace`<br />
`pip install -r requirements.txt`<br />
`catkin_make`<br />
`source devel/setup.bash`<br />


### RUN Guide

Before running the files below, check that they are executables (*allow executing file as program*).

`cd Documents/VROOM-BFMC-Simulator/bfmc_workspace`<br />
`source devel/setup.bash`<br />
`roslaunch sim_pkg map_with_all_objects_REC.launch`<br /><br />
`cd ..`<br />
`cd startup_workspace`<br />
`source devel/setup.bash`<br />
`rosrun startup_package run_view.py`<br /><br />
`cd ..`<br />
`cd vroom_workspace`<br />
`source devel/setup.bash`<br />
`rosrun startup_package main_FSM.py`<br /><br />
`cd src/startup_package/src/bfmclib`<br />
`python3 sign_detection_sim.py`
