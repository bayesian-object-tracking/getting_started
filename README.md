

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

The core libraries for object tracking are ROS independent. However, 
the tracker integration with sensors is based on the ROS eco-system.

Here, we give instructions on how to install the code and a getting started 
repository. This repository contains a complete example, including the 
necessary models and data. We recommend that you follow the instructions below
to install and run the example. Once the example is working, you can adapt it 
to your needs.


## Requirements
 * MS Kinect or Asus XTION depth sensor (unless you work with recorded data)
 * Ubuntu 14.04 (might work on other versions, but has not been tested)
 * c++11 Compiler (gcc-4.7 or later)
 * (optional) [CUDA](https://developer.nvidia.com/cuda-downloads) 6.5 or later with CUDA enabled
   graphic card 

## Dependencies
 * [ROS Indigo](http://wiki.ros.org/indigo)
 * [Eigen](http://eigen.tuxfamily.org/) 3.2.1 or later
 
## Object Tracking
The object tracking can be used without the robot tracking package (dbrt). 

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
If no CUDA enabled device is available, you can build without the GPU implementation via 
```bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DDBOT_BUILD_GPU=Off
```

### Install and run the example

The getting started repository contains a ROS bagfile (a depth image sequence of an object being moved),
and mesh models of some objects. Additionally it contains launch files, which allow
to run the code easily.

To install, follow these steps:
```bash
cd projects/tracking/src
git clone https://git-amd.tuebingen.mpg.de/open-source/dbot_getting_started.git
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release -DDBOT_BUILD_GPU=On
source devel/setup.bash
```
Again, if no CUDA enabled device is available, you can build without the GPU implementation via 
```bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DDBOT_BUILD_GPU=Off
```

Now you can run the example:
```bash
roslaunch dbot_example launch_example_gpu.launch
```
If you did not install CUDA, you can run instead:
```bash
roslaunch dbot_example launch_example_cpu.launch
```
Note that the tracking performance is significantly better with the GPU version.


As soon as you launch the example, an interactive marker should show up in 
rviz. This is for initialization of the tracker, you can move it to align it 
with the point cloud, but it should already be approximately aligned. Once you 
are done, you can click on the object marker and the tracker should start. You should 
do so before the object is being moved in the playback of the bagfile.

### Addition documentation

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
with the instructions below:

```bash
cd $HOME
cd projects/tracking/src
git clone git@github.com:bayesian-object-tracking/dbrt.git
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release -DDBOT_BUILD_GPU=On
```
Again, if no CUDA enabled device is available, you can deactivate the GPU implementation via 
```bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DDBOT_BUILD_GPU=Off
```

### Install and run the example

Add the following example project to the workspace

```bash
cd src
git clone https://git-amd.tuebingen.mpg.de/open-source/dbrt_getting_started.git
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release -DDBOT_BUILD_GPU=On
source devel/setup.bash
```
Now you can run the robot tracker along with the 
recorded sensory data:

```bash
roslaunch dbrt_example launch_example_gpu.launch
```

If CUDA is not being used, you can start the CPU based setup instead: 
```bash
roslaunch dbrt_example launch_example_cpu.launch
```
Note that the tracking performance is significantly better with the GPU version.

This will start the data playback, the visualization and the robot tracker.
You should see a point cloud in white, the robot model using only joint
encoders in red, and the corrected robot model in blue. It should be visible
that the blue robot model is significantly better aligned with the point cloud than 
the red one.



### Addition documentation

For additionl details about the object tracking, please checkout the 
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
