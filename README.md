The Dynamic Whole-Body Locomotion library (DWL)
===============================================

This is the core of DWL

Visit the [DWL installation page](http://.html) for detailed installation instructions, and also the Dynamic Legged Systems Lab of Istituto Italiano di Tecnologia (http://www.iit.it/en/advr-labs/dynamic-legged-systems.html) for more details about the project.

DWL has the following required dependencies:

* [Boost](http://www.boost.org) (version x.x or higher)
* [CMake](http://www.cmake.org) (version 2.8.3 or higher)
* [Eigen](http://eigen.tuxfamily.org) (version 3.2.2 or higher)
* [Odeint](http://headmyshoulder.github.io/odeint-v2/index.html) (version 2 or higher)
* [Yaml-cpp](https://code.google.com/p/yaml-cpp/) (version 0.2.7 or higher)

The following dependece are optional:
* [Doxygen](http://www.doxygen.org)
* [Ipopt](https://projects.coin-or.org/Ipopt) (version 3.11.8 or higher)
* [Octomap](http://octomap.github.io) (version 1.6.6 or higher)

All these required dependencies can be installed using install_deps.sh script.

Once dependencies are installed, you can build DWL on Linux. Go to the top-level directory of DWL and type the
following commands:

    mkdir -p build/Release
    cd build/Release
    cmake ../..

INSTALLATION:
- go to top folder (the folder this file is located in)
- execute cmake: "cmake ."
- either simply compile the library: "make", or call the installation: "sudo make install" (this will copy the library and header to your system folder and create a corresponding FindDWL.cmake file)
