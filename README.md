===============================================
The Dynamic Whole-body Locomotion library (DWL)
===============================================

.. contents:: Table of Contents

Introduction
===============================================
The Dynamic Whole Body Locomotion library (DWL) implements a set of functionalities to develop, design, and deploy locomotion algorithms, i.e. planning, control, etc. DWL has different modules such as: kinematics, dynamics, solvers (tree-search, optimization, etc), and environment descriptions. All these tools are designed for many different locomotion problems such as planning, control and state estimation. DWL library is developed to be able to deploy in different robot framework systems such as ROS, and test it on real-time kernels. DWL has different library dependencies, which most of them are optional. DWL is develop by Carlos Mastalli (carlos.mastalli@iit.it) at Istituto Italiano di Tecnologia.

As well as DWL source code, we provide implementation of different dwl-based locomotion modules such: control, planning and perception. All these modules use ROS as framework for communication between them. As standard message interface between them, we use dwl_msgs packages and a set of commons methods for controllers and planners.

Visit the DWL installation page for detailed installation instructions, and also the Dynamic Legged Systems Lab of Istituto Italiano di Tecnologia (http://www.iit.it/en/advr-labs/dynamic-legged-systems.html) for more details about the project.

[![ScreenShot](https://j.gifs.com/zJEDWD.gif)](https://www.youtube.com/watch?v=ENHvCGrnr2g)


Software Overview
===============================================
The algorithms are built primarily in C/C++. The library uses a number of the local dependencies, which some of them are optionals.

DWL has the following required dependencies:
* [Boost](http://www.boost.org) (version 1.5.4 or higher)
* [CMake](http://www.cmake.org) (version 2.8.3 or higher)
* [Eigen](http://eigen.tuxfamily.org) (version 3.2.4 or higher)
* [Yaml-cpp](https://code.google.com/p/yaml-cpp/) (version 0.2.7 or higher)
* [RBDL](http://rbdl.bitbucket.org/) (version 2.4.0 or higher)
* [urdfdom_header](https://github.com/ros/urdfdom_headers) (version 0.2.3 or higher)
* [console_bridge](https://github.com/ros/console_bridge) (version 0.2.7 or higher)
* [urdfdom](https://github.com/ros/urdfdom) (version 0.2.10 or higher)

The following dependencies are optional:
* [Doxygen](http://www.doxygen.org)
* [qpOASES](https://projects.coin-or.org/qpOASES) (version 3.2.0 or higher)
* [Ipopt](https://projects.coin-or.org/Ipopt) (version 3.12.4 or higher)
* [libcmaes](https://github.com/beniz/libcmaes) (version 0.9.5 or higher)
* [Octomap](http://octomap.github.io) (version 1.6.8 or higher)


Building
===============================================
All these required dependencies must be installed using install_deps.sh script.

Once dependencies are installed, you can build DWL on Linux. Go to the top-level directory of DWL and type the
following commands:

    mkdir -p build/Release
    cd build/Release
    cmake ../..

Additionally you could installed as catkin project as follows:
	cd your_ros_ws/
	catkin_make


Installation
===============================================
- go to top folder (the folder this file is located in)
- execute cmake: "cmake ."
- either simply compile the library: "make", or call the installation: "sudo make install" (this will copy the library and header to your system folder and create a corresponding FindDWL.cmake file)



Publications
===============================================
* C. Mastalli, I. Havoutis, M. Focchi, D. G. Caldwell, C. Semini, Hierarchical Planning of Dynamic Movements without Scheduled Contact Sequences, IEEE International Conference on Robotics and Automation (ICRA), 2016
* C. Mastalli, A. Winkler, I. Havoutis, D. G. Caldwell, C. Semini, On-line and On-board Planning and Perception for Quadrupedal Locomotion, IEEE International Conference on Technologies for Practical Robot Applications (TEPRA), 2015
* A. Winkler, C. Mastalli, I. Havoutis, M. Focchi, D. G. Caldwell, C. Semini, Planning and Execution of Dynamic Whole-Body Locomotion for a Hydraulic Quadruped on Challenging Terrain, IEEE International Conference on Robotics and Automation (ICRA), 2015
