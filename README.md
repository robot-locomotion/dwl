The Dynamic Whole-Body Locomotion library (WDL)
===============================================

This is the core of WDL

Visit the [DWL installation page](http://.html) for
detailed installation instructions.

DWL has the following required dependencies:

* [Boost](http://www.boost.org) (version x.x or higher)
* [CMake](http://www.cmake.org) (version x.x.x or higher)

The following dependece are optional:
* [Doxygen](http://www.doxygen.org)

Once dependencies are installed, you can build DWL on Linux. Go to the top-level directory of DWL and type the
following commands:

    mkdir -p build/Release
    cd build/Release
    cmake ../..

INSTALLATION:
- go to top folder (the folder this file is located in)
- execute cmake: "cmake ."
- either simply compile the library: "make", or call the installation: "sudo make install" (this will copy the library and header to your system folder and create a corresponding FindLSE.cmake file)
