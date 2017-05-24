

* [Depth Based Object and Robot Tracking](#depth-based-object-and-robot-tracking)
  * [Requirements](#requirements)
  * [Dependencies](#dependencies)
  * [Object Tracking](#object-tracking)
    * [Workspace setup and compilation](#workspace-setup-and-compilation)
    * [Getting Started Example Object Tracking Project](#example-object-tracking-project)
    * [Addition documentation](#additional-documentation)
  * [Robot Tracking](#robot-tracking)
    * [Getting Started Example Robot Tracking Project](#example-robot-tracking-project-using-mpi-apollo-robot)
    



# Depth Based Object and Robot Tracking 

This is a collection of packages for tracking known objects and articulated
robots based on depth images and additionally joint angle sensors for the robot 
tracking. The core libraries for object tracking are ROS independent. However, 
the tracker integration with sensors is based on the ROS eco-system. In this 
document we will show an example on how to use the trackers and provided 
additional information on customizing the setup for different robots.

## Requirements
 * MS Kinect or Asus XTION depth sensor
 * Ubuntu 14.04
 * c++11 Compiler (gcc-4.7 or later)
 * (optional) [CUDA](https://developer.nvidia.com/cuda-downloads) 6.5 or later with CUDA enabled
   graphic card 

## Dependencies
 * [ROS Indigo](http://wiki.ros.org/indigo)
 * [Filtering library](https://github.com/filtering-library/fl) (fl)
 * [Eigen](http://eigen.tuxfamily.org/) 3.2.1 or later
 
## Object Tracking
The object tracking can be used without the robot tracking package (dbrt). 

### Workspace setup and compilation
```bash
$ cd $HOME
$ mkdir -p projects/tracking/src  
$ cd projects/tracking/src
$ git clone git@github.com:filtering-library/fl.git
$ git clone git@github.com:bayesian-object-tracking/dbot.git
$ git clone git@github.com:bayesian-object-tracking/dbot_ros_msgs.git
$ git clone git@github.com:bayesian-object-tracking/dbot_ros.git
$ cd ..
$ catkin_make -DCMAKE_BUILD_TYPE=Release -DDBOT_BUILD_GPU=On
$ source devel/setup.bash
```
If no CUDA enabled device is available, you can deactivate the GPU implementation via 
```bash
$ catkin_make -DCMAKE_BUILD_TYPE=Release -DDBOT_BUILD_GPU=Off
```

### Example Object Tracking Project 

Checkout the following package in your tracking workspace and compile again. 
This package contains a bag file including record depth images and an object 
model. To launch the example, do the following:

```bash
cd src
git clone https://git-amd.tuebingen.mpg.de/open-source/dbot_getting_started.git
```

### Addition documentation

For additionl details about the object tracking, please checkout the [dbot_ros](https://github.com/bayesian-object-tracking/dbot_ros/blob/master/README.md) package.

## Robot Tracking

The robot tracking setup builds on top of the object tracking, i.e. follow 
first the workspace setup and of the object tracking above. Then checkout 
the following package to the workspace

```bash
$ cd $HOME
$ cd projects/tracking/src
$ git clone git@github.com:bayesian-object-tracking/dbrt.git
$ cd ..
$ catkin_make -DCMAKE_BUILD_TYPE=Release -DDBOT_BUILD_GPU=On
```
Again, if no CUDA enabled device is available, you can deactivate the GPU implementation via 
```bash
$ catkin_make -DCMAKE_BUILD_TYPE=Release -DDBOT_BUILD_GPU=Off
```

### Example Robot Tracking Project using MPI Apollo Robot

Add the following example project to the workspace

```bash
git clone https://git-amd.tuebingen.mpg.de/open-source/dbrt_getting_started.git
```
and re-compile. Once compile you can run the robot tracker along with the 
recorded sensory data:

```bash
roslaunch dbrt_example launch_example_gpu.launch
```

If CUDA is not being used, you can start the CPU based setup by launching 
`lula_example_cpu.launch` instead. Note that the CPU version will run slower.


