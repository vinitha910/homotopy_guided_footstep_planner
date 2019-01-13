# Homotopy Guided Footstep Planner

This repository contains a humanoid footstep planning framework that uses the Homotopy-Shortest Path Algorithm ([HBSP](https://aaai.org/ocs/index.php/ICAPS/ICAPS18/paper/view/17721/16943)) for automatically generating heuristic functions.

## Table of Contents
1. [Installation](#installation)
   - [Install Protobuf](#protobuf)
   - [Install ROS](#ros)
   - [Footstep Planner Setup](#footstep)
2. [Planner Usage](#usage)
3. [Defining World Environments](src/footstep_planner/worlds/README.md)
4. [Defining Robot Parameters](src/footstep_planner/proto/README.md)
5. [Test Scenario Generation](src/footstep_planner/python/README.md)
6. [License](#license)
7. [Authors/Acknowledgements](#authors)

## Installation <a name="installation"></a>

### Install Protobuf <a name="protobuf"></a>

To install protobuf 3, run the following commands:
```bash
$ sudo add-apt-repository ppa:maarten-fonville/protobuf
$ sudo apt-get update
$ sudo apt-get install libprotobuf15 libprotobuf-dev protobuf-compiler python-protobuf
```

### Install ROS <a name="ros"></a>
This code base works on ROS Indigo and Kinetic. Install the appropiate version of ROS by following the documentation at http://wiki.ros.org/kinetic/Installation or http://wiki.ros.org/indigo/Installation

### Setting up the Footstep Planner and its Dependencies <a name="footstep"></a>
Clone the repository into a directory called `humanoid_ws`.
```bash
$ git clone https://github.com/vinitha910/homotopy_guided_footstep_planner humanoid_ws
```
The root directory will be the root of your catkin workspace when developing on this project. The `src` directory contains a `.rosinstall` file with references to all necessary sub-projects.

If you do not have the `wstool` utility, install it by following the instructions at http://wiki.ros.org/wstool.

#### Initialize the Catkin Workspace
```bash
$ source /opt/ros/<ROS_VERSION>/setup.bash
$ cd humanoid_ws/src
$ catkin_init_workspace
```

#### Clone all Sub-projects via `wstool`
```bash
$ cd humanoid_ws
$ wstool update -t src
```

#### Build the Workspace
You should expect the build to fail because of missing dependencies
```bash
$ cd humanoid_ws
$ catkin build
$ source devel/setup.bash
```

#### Install System Dependencies
```bash
$ cd humanoid_ws/src
$ for d in $(find . -name package.xml); do PKG=$(basename $(dirname $d)); rosdep install -i $PKG; done
```

#### Install and Build SBPL
```bash
$ cd humanoid_ws/src
$ git clone https://github.com/sbpl/sbpl.git
$ cd sbpl
$ mkdir build && cd build && cmake .. && make
$ sudo make install
```

#### Rebuild and Source the Workspace
```bash
$ cd humanoid_ws
$ catkin build
$ source devel/setup.bash
```

## Usage <a name="usage"></a>

Running the footstep planner is done by entering 
```bash
$ roslaunch footstep_planner footstep_planner.launch debug:=<debug_boolean> scenario:=<scenario_file>
```
into the terminal. Set `<debug_boolean>` to `true` to launch the gdb debugger, otherwise set to false. `<scenario_file>` is the name of a scenario `.yaml` file in footstep_planner/scenarios/. Make sure to have sourced `devel/setup.bash` before running the launch file. 
 
## License <a name="license"></a>

Homotopy Guided Footstep Planner is licensed under a BSD license. See [LICENSE](LICENSE) for more information.

## Authors/Acknowledgements <a name="authors"></a>
This code base was developed by Vinitha Ranganeni ([**@vinitha910**](https://github.com/vinitha910)) and Sahit Chintalapudi([**@chsahit**] (https://github.com/chsahit)) primarily at the [Search-Based Planning Lab](http://sbpl.net) in the [Robotics Institute](http://ri.cmu.edu/) at [Carnegie Mellon University](http://www.cmu.edu/). The developement of this code base was continued at [University of Washington](https://www.washington.edu/). We'd like to thank the National Science Foundation for funding our work through grants [IIS-1659774](https://www.nsf.gov/awardsearch/showAward?AWD_ID=1659774), [IIS-1409549](https://www.nsf.gov/awardsearch/showAward?AWD_ID=1409549) and [DGE-1762114](https://www.nsf.gov/awardsearch/showAward?AWD_ID=1762114).

