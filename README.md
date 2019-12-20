

* [Depth Based Object and Robot Tracking](#depth-based-object-and-robot-tracking)
  * [Requirements](#requirements)
  * [Dependencies](#dependencies)
  * [Object Tracking](#object-tracking)
    * [Workspace setup and compilation](#workspace-setup-and-compilation)
    * [Getting Started Example Object Tracking Project](#example-object-tracking-project)
    * [Addition documentation](#additional-documentation)
    * [How to cite](#how-to-cite)
  * [Robot Tracking](#robot-tracking)
    * [Getting Started Example Robot Tracking Project](#example-robot-tracking-project-using-mpi-apollo-robot)
    * [How to cite](#how-to-cite)
    



# Bayesian Object Tracking

The Bayesian Object Tracking organization on github is a collection of packages for
3D tracking of rigid objects (using depth images), as well as robot arm tracking (using depth images and joint encoders).
For more details about the research which underlies these packages, please have a look at https://am.is.tuebingen.mpg.de/research_projects/probabilistic-object-tracking-using-a-depth-camera.

The core library for object tracking (dbot) is ROS independent. However, 
the integration with sensors (dbot_ros, dbrt) is based on the ROS eco-system.

Here, we give instructions on how to install the code and a getting started 
repository. This repository contains a complete example, including the 
necessary models and data. We recommend that you follow the instructions below
to install and run the example. Once the example is working, you can adapt it 
to your needs.


## Requirements
 * MS Kinect 1/2 or Asus XTION depth sensor (might work with other sensors, but has not been tested)
 * c++11 Compiler (gcc-4.7 or later)
 * (optional) [CUDA](https://developer.nvidia.com/cuda-downloads) 6.5 or later with CUDA enabled
   graphic card 

## Dependencies
 * ROS
 * [Eigen](http://eigen.tuxfamily.org/) 3.2.1 or later
 
## Object Tracking
The object tracking (dbot, dbot_ros) can be used without the robot tracking package (dbrt). 

### Workspace setup and compilation

```bash
cd $HOME
mkdir -p projects/tracking/src  
cd projects/tracking/src
git clone git@github.com:filtering-library/fl.git
git clone git@github.com:bayesian-object-tracking/dbot.git
git clone git@github.com:bayesian-object-tracking/dbot_ros_msgs.git
git clone git@github.com:bayesian-object-tracking/dbot_ros.git
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release -DDBOT_BUILD_GPU=On
source devel/setup.bash
```

### Install and run the example
The getting started repository contains a ROS bagfile (a depth image sequence of an object being moved)
and mesh models of some objects. Additionally it contains launch files, which allow you
to run the code easily.

To install, follow these steps (note that cloning may take a while because the bagfile is large):
```bash
cd $HOME/projects/tracking/src
git clone https://git-amd.tuebingen.mpg.de/open-source/dbot_getting_started.git
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release -DDBOT_BUILD_GPU=On
source devel/setup.bash
```

Now you can run the example:
```bash
roslaunch dbot_example launch_example_gpu.launch
```

If you did not install CUDA, you can run instead (note that the tracking performance is significantly better with the GPU version):
```bash
roslaunch dbot_example launch_example_cpu.launch
```



As soon as you launch the example, rviz should start, and an interactive marker should show up (in the form of an impact wrench). This marker is for initialization of the tracker, you can move it to align it 
with the point cloud. In this example, it should already be approximately aligned. Once you 
are done moving the marker, you can click on it and the tracker should start (note that in the recorded sequence the object starts moving at some point, make sure you initialize before that). You should see a green object 
model following the actual object visible in the white point cloud.

### Additional documentation

For additional details about the object tracking, please checkout the 
[dbot_ros](https://github.com/bayesian-object-tracking/dbot_ros/blob/master/README.md) package.

### How to cite
```
inproceedings{wuthrich-iros-2013,
 title = {Probabilistic Object Tracking Using a Range Camera},
 author = {W{\"u}thrich, M. and Pastor, P. and Kalakrishnan, M. and Bohg, J. and Schaal, S.},
 booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems},
 pages = {3195-3202},
 publisher = {IEEE},
 month = nov,
 year = {2013},
 month_numeric = {11}
}
```

## Robot Tracking

### Workspace setup and compilation
The robot tracking setup builds on top of the object tracking, i.e. follow 
first the workspace setup of the object tracking above. Then continue
with the instructions below according to your ROS and Ubuntu version:


#### With ROS kinetic and Ubuntu 16.04 or older
```bash
cd $HOME/projects/tracking/src
git clone git@github.com:bayesian-object-tracking/dbrt.git
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release -DDBOT_BUILD_GPU=On
```

#### With ROS melodic and Ubuntu 18.04 or newer
```bash
cd $HOME/projects/tracking/src
git clone git@github.com:bayesian-object-tracking/dbrt.git
cd dbrt
git checkout bionic-beaver-melodic-dev
cd ../..
catkin_make -DCMAKE_BUILD_TYPE=Release -DDBOT_BUILD_GPU=On
```



### Install and run the example

Add the following example project to the workspace (note that cloning may take a while due to the size of the data)

#### With ROS indigo and Ubuntu 14.04

```bash
cd $HOME/projects/tracking/src
git clone https://git-amd.tuebingen.mpg.de/open-source/dbrt_getting_started.git
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release -DDBOT_BUILD_GPU=On
source devel/setup.bash
```

#### With ROS kinetic and Ubuntu 16.04 or newer
```bash
cd $HOME/projects/tracking/src
git clone https://git-amd.tuebingen.mpg.de/open-source/dbrt_getting_started.git
cd dbrt_getting_started
git checkout xenial-xerus-kinetic-dev
cd ../../
catkin_make -DCMAKE_BUILD_TYPE=Release -DDBOT_BUILD_GPU=On
source devel/setup.bash
```

Now you can run the robot tracker along with the 
recorded sensory data:

```bash
roslaunch dbrt_example launch_example_gpu.launch
```

If CUDA is not being used, you can start the CPU based setup instead (note that the tracking performance is significantly better with the GPU version):
```bash
roslaunch dbrt_example launch_example_cpu.launch
```

This will start the data playback, the visualization and the robot tracker.
You should see a point cloud in white, the robot model using only joint
encoders in red, and the corrected robot model (fusing joint encoders and depth images) in blue. It should be visible
that the blue robot model is significantly better aligned with the point cloud than 
the red one.



### Additional documentation

For additional details about the robot tracking, please checkout the 
[dbrt](https://github.com/bayesian-object-tracking/dbrt/blob/master/README.md) package.

### How to cite
```
@article{GarciaCifuentes.RAL,
 title = {Probabilistic Articulated Real-Time Tracking for Robot Manipulation},
 author = {Garcia Cifuentes, Cristina and Issac, Jan and W{\"u}thrich, Manuel and Schaal, Stefan and Bohg, Jeannette},
 journal = {IEEE Robotics and Automation Letters (RA-L)},
 volume = {2},
 number = {2},
 pages = {577-584},
 month = apr,
 year = {2017},
 month_numeric = {4}
}
```
