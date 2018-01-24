The Dynamic Whole-body Locomotion library (DWL)
===============================================

<img align="left" height="240" src="https://imgur.com/SkeMizm.png"/> 

## <img align="center" height="20" src="https://i.imgur.com/vAYeCzC.png"/> Introduction

The *Dynamic Whole-body Locomotion* library (DWL) implements a set of functionalities to develop, design, and deploy motion planning, control and perception algorithms for legged locomotion. DWL has different modules such as: kinematics, dynamics, solvers (tree-search, optimization, etc), and environment descriptions. All these tools are designed for both fast prototyping and deployment thanks to its c++ implementation and Python bindings. The DWL toolbox can be used in various software frameworks such as ROS and LCM, and for real-time control and planning.

DWL was developed by Carlos Mastalli at [Dynamic Legged Systems lab (DLS)](http://www.iit.it/en/advr-labs/dynamic-legged-systems.html), Istituto Italiano di Tecnologia, Italy. The DWL is core toolbox used along of various software of the DLS lab at IIT.

| [![](https://i.imgur.com/BT7fRCU.gif)](https://www.youtube.com/watch?v=ENHvCGrnr2g&t=2s) | [![](https://i.imgur.com/4kKhryj.gif)](https://www.youtube.com/watch?v=KI9x1GZWRwE)
|:-------------------------:|:-------------------------:|
| [![](https://i.imgur.com/yXTtxUK.gif)](https://www.youtube.com/watch?v=ArV2yh7KSfE) | [![](https://i.imgur.com/RKe3sNo.gif)](https://www.youtube.com/watch?v=KI9x1GZWRwE)
|||



The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Carlos Mastalli, carlos.mastalli@laas.fr<br />
With support from the Dynamic Legged Systems lab at Istituto Italiano di Tecnologia<br />**



[![License BSD-3-Clause](https://img.shields.io/badge/license-BSD--3--Clause-blue.svg?style=flat)](https://tldrlegal.com/license/bsd-3-clause-license-%28revised%29#fulltext)
[![Build Status](https://api.travis-ci.org/robot-locomotion/dwl.svg?branch=master)](https://api.travis-ci.org/repositories/robot-locomotion/dwl.svg)



## <img align="center" height="20" src="https://i.imgur.com/fjS3xIe.png"/> Dependencies

The algorithms are built primarily in C++. The library uses a number of the local dependencies, which some of them are optionals. The Python bindings are generated using SWIG, you can generate them by setting DWL_WITH_PYTHON=True.

DWL has the following required dependencies:
* [Boost](http://www.boost.org) (version 1.5.4 or higher)
* [CMake](http://www.cmake.org) (version 2.8.3 or higher)
* [Eigen](http://eigen.tuxfamily.org) (version 3.2.0 or higher)
* [Yaml-cpp](https://code.google.com/p/yaml-cpp/) (version 0.5.2 or higher)
* [RBDL](http://rbdl.bitbucket.org/) (version 2.4.0 or higher)
* [SWIG](http://www.swig.org/) (version 3.0.12 or higher)
* [urdfdom_header](https://github.com/ros/urdfdom_headers) (version 0.2.3 or higher)
* [console_bridge](https://github.com/ros/console_bridge) (version 0.2.7 or higher)
* [urdfdom](https://github.com/ros/urdfdom) (version 0.2.10 or higher)
* [Octomap](http://octomap.github.io) (version 1.6.8 or higher)

The following dependencies are optional:
* [Doxygen](http://www.doxygen.org)
* [qpOASES](https://projects.coin-or.org/qpOASES) (version 3.2.0 or higher)
* [Ipopt](https://projects.coin-or.org/Ipopt) (version 3.12.4 or higher)
* [libcmaes](https://github.com/beniz/libcmaes) (version 0.9.5 or higher)


## <img align="center" height="20" src="https://i.imgur.com/x1morBF.png"/> Building

To build DWL from source code, you first need to install its dependencies. We recommend you to install them using the 
install_deps.sh script. The INSTALL_DEPS_PREFIX defines the folder where is installed its dependencies (usually /usr/local for linux machines). However you can install them in your local folder.

Once dependencies are installed, you can build DWL on Linux. Go to the top-level directory of DWL and type the
following commands:

    mkdir -p build/Release
    cd build/Release
    cmake -DCMAKE_INSTALL_PREFIX=${/your/dependencies/path} -DINSTALL_DEPS_PREFIX=${/dwl/installation/path}  ../../

Note that /usr/local is the default path of ${/your/dependencies/path} and ${/dwl/installation/path}.

Additionally you could installed as catkin project as follows:

    cd your_ros_workspace/
    catkin_make -DCMAKE_INSTALL_PREFIX=${/dwl/installation/path} -DINSTALL_DEPS_PREFIX=${/your/dependencies/path}

Finally you can generate the Python bindings and Doxygen documentation by doing:

   cmake -DDWL_WITH_PYTHON=True -DDWL_WITH_DOC=True ../



## <img align="center" height="20" src="https://i.imgur.com/x1morBF.png"/> Installation

If you compilate DWL as CMake project, you can install it using "make install". Additionally DWL has a optional command for installing only the Python modules, i.e. "make install_python". Note that all the DWL header files, libraries, executatables and CMake module file will be installed in CMAKE_INSTALL_PREFIX



## <img align="center" height="20" src="https://cdn2.iconfinder.com/data/icons/freecns-cumulus/16/519660-164_QuestionMark-512.png"/> HowTo

Using DWL, and its dependencies, is simple for both c++ and Python code. For c++ code, you just have to add in your CMakeLists file the follows:

    // Find DWL, this operation will defined the following variables:
    //  - dwl_LIBRARIES: the list of libraries to link against
    //  - dwl_LIBRARY_DIRS: The directory where the lib files are
    //  - dwl_INCLUDE_DIRS: The list of include directories
    find(dwl REQUIRED) //for CMake project
    find_package(catkin REQUIRED dwl) //for Catkin project
    include_directories(${dwl_INCLUDE_DIRS}
    target_link_libraries(your_library  S{dwl_LIBRARIES})

If you want to use the dwl Python module, you have to install it (see installation instructions) and adding your installation path in PYTHONPATH (e.g. "export PYTHONPATH=${PYTHONPATH}:${/dwl/installation/path}:")

For more information you could check our sample code list (c++ and Python) inside "sample" folder.



## <img align="center" height="20" src="http://www.pvhc.net/img205/oohmbjfzlxapxqbpkawx.png"/> Publications


* C. Mastalli, I. Havoutis, M. Focchi, D. G. Caldwell, C. Semini, [Motion planning for challenging locomotion: a study of decoupled and coupled approaches](https://hal.archives-ouvertes.fr/hal-01649836v1), IEEE International Conference on Robotics and Automation (ICRA), 2017
* B. Aceituno-Cabezas, C. Mastalli, H. Dai, M. Focchi, A. Radulescu, D. G. Calwell, J. Cappelletto, J. C. Griego, G. Fernardez, C. Semini, [Simultaneous Contact, Gait and Motion Planning for Robust Multi-Legged Locomotion via MICP](http://ieeexplore.ieee.org/document/8141917/), IEEE Robotics and Automation Letters  (RAL), 2017
* C. Mastalli, M. Focchi, I. Havoutis, A. Radulescu, D. G. Caldwell, C. Semini, [Trajectory and Foothold Optimization using Low-Dimensional Models for Rough Terrain Locomotion](https://old.iit.it/images/stories/advanced-robotics/hyq_files/publications/mastalli17icra.pdf), IEEE International Conference on Robotics and Automation (ICRA), 2017
* C. Mastalli, I. Havoutis, M. Focchi, D. G. Caldwell, C. Semini, [Hierarchical Planning of Dynamic Movements without Scheduled Contact Sequences](http://iit.it/images/stories/advanced-robotics/hyq_files/publications/icra16mastalli.pdf), IEEE International Conference on Robotics and Automation (ICRA), 2016
* C. Mastalli, A. Winkler, I. Havoutis, D. G. Caldwell, C. Semini, [On-line and On-board Planning and Perception for Quadrupedal Locomotion](http://iit.it/images/stories/advanced-robotics/hyq_files/publications/mastalli15tepra.pdf), IEEE International Conference on Technologies for Practical Robot Applications (TEPRA), 2015
* A. Winkler, C. Mastalli, I. Havoutis, M. Focchi, D. G. Caldwell, C. Semini, [Planning and Execution of Dynamic Whole-Body Locomotion for a Hydraulic Quadruped on Challenging Terrain](http://iit.it/images/stories/advanced-robotics/hyq_files/publications/winkler15icra.pdf), IEEE International Conference on Robotics and Automation (ICRA), 2015
