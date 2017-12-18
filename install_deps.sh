#!/bin/bash

# List of usefull colors
COLOR_RESET="\033[0m"
COLOR_INFO="\033[0;32m"
COLOR_ITEM="\033[1;34m"
COLOR_QUES="\033[1;32m"
COLOR_WARN="\033[0;33m"
COLOR_BOLD="\033[1m"
COLOR_UNDE="\033[4m"

INSTALL_DEPS_PREFIX=/usr/local
COMMON_INSTALL_PREFIX=/usr

# Getting the current directory of this script
CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"


## This function detects the current os and distro
CURRENT_OS="Unsupported" #CENTOS, UBUNTU are other valid options
function findCurrentOSType()
{
	echo
	osType=$(uname)
	case "$osType" in
	"Darwin")
	{
		CURRENT_OS="OSX"
	} ;;
	"Linux")
	{
		# If available, use LSB to identify distribution
		if [ -f /etc/lsb-release -o -d /etc/lsb-release.d ]; then
			DISTRO=$(gawk -F= '/^NAME/{print $2}' /etc/os-release) # try to remove the ""
		else
			DISTRO=$(ls -d /etc/[A-Za-z]*[_-][rv]e[lr]* | grep -v "lsb" | cut -d'/' -f3 | cut -d'-' -f1 | cut -d'_' -f1)
		fi
		CURRENT_OS=$(echo $DISTRO | tr 'a-z' 'A-Z')
		CURRENT_OS="UBUNTU"
	} ;;
	*)
	{
		echo "Unsupported OS, exiting"
		exit
	} ;;
	esac
	echo -e "${COLOR_BOLD}Running on ${CURRENT_OS}.${COLOR_RESET}"
}


function install_eigen
{
	# Remove old folder (sanity procedure)
	cd $CURRENT_DIR/thirdparty
	sudo rm -rf eigen

	if [ "$CURRENT_OS" == "OSX" ]; then
		# Getting Eigen 3.2.10
		curl -L "http://www.bitbucket.org/eigen/eigen/get/3.2.10.tar.bz2" | tar xj
		mv eigen-eigen-*/ eigen
		cd eigen
		mkdir -p build
		cd build
		cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$INSTALL_DEPS_PREFIX -DPKGCONFIG_INSTALL_DIR=$INSTALL_DEPS_PREFIX/lib/pkgconfig ..
		sudo make -j install
	elif [ "$CURRENT_OS" == "UBUNTU" ]; then
		# Getting Eigen 3.2.10
		wget http://www.bitbucket.org/eigen/eigen/get/3.2.10.tar.bz2
		mkdir eigen && tar jxf 3.2.10.tar.bz2 -C eigen --strip-components 1
		rm -rf 3.2.10.tar.bz2
		cd eigen
		mkdir -p build
		cd build
		cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$INSTALL_DEPS_PREFIX -DPKGCONFIG_INSTALL_DIR=$INSTALL_DEPS_PREFIX/lib/pkgconfig ..
		sudo make -j install
    fi
}


function install_rbdl
{
	# Remove old folder (sanity procedure)
	cd $CURRENT_DIR/thirdparty
	sudo rm -rf rbdl

	if [ "$CURRENT_OS" == "OSX" ]; then
		curl -L "https://bitbucket.org/rbdl/rbdl/get/v2.4.0.zip" > v2.4.0.zip
		unzip v2.4.0.zip
		rm v2.4.0.zip
		mv rbdl-*/ rbdl
		cd rbdl
		mkdir -p build
		cd build
		cmake -D RBDL_BUILD_ADDON_URDFREADER:bool=ON -D CMAKE_INSTALL_LIBDIR:string=lib -DCMAKE_INSTALL_PREFIX=$INSTALL_DEPS_PREFIX ../
		make -j
		sudo make -j install
	elif [ "$CURRENT_OS" == "UBUNTU" ]; then
		# Getting RBDL 2.4.0
		wget https://bitbucket.org/rbdl/rbdl/get/v2.4.0.zip
		unzip v2.4.0.zip
		rm v2.4.0.zip
		mv rbdl-rbdl-* rbdl
		cd rbdl
		mkdir -p build
		cd build
		cmake -D RBDL_BUILD_ADDON_URDFREADER:bool=ON -D CMAKE_INSTALL_LIBDIR:string=lib -DCMAKE_INSTALL_PREFIX=$INSTALL_DEPS_PREFIX ../
		make -j
		sudo make -j install
	fi
}


function install_urdfdom_headers
{
	# Remove old folder (sanity procedure)
	cd $CURRENT_DIR/thirdparty
	sudo rm -rf urdf_headers

	if [ "$CURRENT_OS" == "OSX" ]; then
		# Getting urdfdom_headers 0.2.3
		curl -L "https://github.com/ros/urdfdom_headers/archive/0.2.3.tar.gz" | tar xz
		mv urdfdom_headers-*/ urdfdom_headers
		cd urdfdom_headers
		mkdir -p build
		cd build
		cmake ../
		sudo make -j install
	elif [ "$CURRENT_OS" == "UBUNTU" ]; then
		# Getting urdfdom_headers 0.2.3
		wget https://github.com/ros/urdfdom_headers/archive/0.2.3.tar.gz
		mkdir urdfdom_headers && tar zxf 0.2.3.tar.gz -C urdfdom_headers --strip-components 1
		rm -rf 0.2.3.tar.gz
		cd urdfdom_headers
		mkdir -p build
		cd build
		cmake ../
		sudo make -j install
	fi
}


function install_console_bridge
{
	# Remove old folder (sanity procedure)
	cd $CURRENT_DIR/thirdparty
	sudo rm -rf console_bridge

	if [ "$CURRENT_OS" == "OSX" ]; then
		# Getting console_bridge 0.2.7
		curl -L "https://github.com/ros/console_bridge/archive/0.2.7.tar.gz" | tar xz
		mv console_bridge-*/ console_bridge
		cd console_bridge
		mkdir -p build
		cd build
		brew install boost
		cmake ../
		sudo make -j install
	elif [ "$CURRENT_OS" == "UBUNTU" ]; then
		# Getting console_bridge 0.2.7
		wget https://github.com/ros/console_bridge/archive/0.2.7.tar.gz
		mkdir console_bridge && tar zxf 0.2.7.tar.gz -C console_bridge --strip-components 1
		rm -rf 0.2.7.tar.gz
		cd console_bridge
		mkdir -p build
		cd build
		cmake ../
		sudo make -j install
	fi
}


function install_urdfdom
{
	# Remove old folder (sanity procedure)
	cd $CURRENT_DIR/thirdparty
	sudo rm -rf urdfdom

	if [ "$CURRENT_OS" == "OSX" ]; then
		# Getting console_bridge 0.2.10
		curl -L "https://github.com/ros/urdfdom/archive/0.2.10.tar.gz" | tar xz
		mv urdfdom-*/ urdfdom
		cd urdfdom
		mkdir -p build
		cd build
		brew install tinyxml
		cmake ../
	elif [ "$CURRENT_OS" == "UBUNTU" ]; then
		# Getting console_bridge 0.2.10
		wget https://github.com/ros/urdfdom/archive/0.2.10.tar.gz
		mkdir urdfdom && tar zxf 0.2.10.tar.gz -C urdfdom --strip-components 1
		rm -rf 0.2.10.tar.gz
		cd urdfdom
		mkdir -p build
		cd build
		cmake ../
		sudo make -j install
	fi
}


function install_yamlcpp
{
	# Remove old folder (sanity procedure)
	cd $CURRENT_DIR/thirdparty
	sudo rm -rf yaml-cpp

	if [ "$CURRENT_OS" == "OSX" ]; then
		echo -e "${COLOR_WARN}Mac OSX installation not support yet${COLOR_RESET}"
		# Getting the YAML-CPP 0.5.2
		curl -L "https://github.com/jbeder/yaml-cpp/archive/release-0.5.2.zip" > release-0.5.2.zip
		unzip release-0.5.2.zip && rm -rf release-0.5.2.zip
		mv yaml-cpp-*/ yaml-cpp
		cd yaml-cpp
		mkdir -p build
		cd build
		cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=$INSTALL_DEPS_PREFIX ../
		sudo make -j install
	elif [ "$CURRENT_OS" == "UBUNTU" ]; then
		# Getting the YAML-CPP 0.5.2
		wget https://github.com/jbeder/yaml-cpp/archive/release-0.5.2.zip
		unzip release-0.5.2.zip && rm -rf release-0.5.2.zip
		mv yaml-cpp-*/ yaml-cpp
		cd yaml-cpp
		mkdir -p build
		cd build
		cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=$INSTALL_DEPS_PREFIX ../
		sudo make -j install
	fi
}


function install_swig
{
	# Remove old folder (sanity procedure)
	cd $CURRENT_DIR/thirdparty
	sudo rm -rf swig

	# Getting the SWIG 3.0.12
	wget http://downloads.sourceforge.net/swig/swig-3.0.12.tar.gz
	mkdir swig && tar zxf swig-3.0.12.tar.gz -C swig --strip-components 1
	rm -rf swig-3.0.12.tar.gz
	cd swig
#	./configure --prefix=/usr --without-clisp --without-maximum-compile-warnings
	./configure --prefix=$INSTALL_DEPS_PREFIX --without-clisp --without-maximum-compile-warnings
	make -j
	sudo make install
}


function install_lapack
{
	# Remove old folder (sanity procedure)
	cd $CURRENT_DIR/thirdparty
	sudo rm -rf lapack

	if [ "$CURRENT_OS" == "OSX" ]; then
		# Getting the LAPACK 3.6.0
		curl -L "http://www.netlib.org/lapack/lapack-3.6.0.tgz" | tar xz
		mv lapack-* lapack
		cd lapack
		mkdir -p build
		cd build
		#TODO this part doesn't work yet
		cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$INSTALL_DEPS_PREFIX -DBUILD_SHARED_LIBS=ON ../
		sudo make -j install
	elif [ "$CURRENT_OS" == "UBUNTU" ]; then
		# Getting the LAPACK 3.6.0
		wget http://www.netlib.org/lapack/lapack-3.6.0.tgz
		mkdir lapack && tar xzvf lapack-3.6.0.tgz -C lapack --strip-components 1
		rm -rf lapack-3.6.0.tgz
		cd lapack
		mkdir -p build
		cd build
		cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$INSTALL_DEPS_PREFIX -DBUILD_SHARED_LIBS=ON ../
		sudo make -j install
	fi
}


function install_ipopt
{
	# Remove old folder (sanity procedure)
	cd $CURRENT_DIR/thirdparty
	sudo rm -rf ipopt

	if [ "$CURRENT_OS" == "OSX" ]; then
		echo -e "${COLOR_WARN}Mac OSX installation not support yet${COLOR_RESET}"
		# Getting Ipopt 3.12.4
		curl -L "http://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.4.tgz" | tar xz
		mv Ipopt-* ipopt
		# Documentation for Ipopt Third Party modules:
		# http://www.coin-or.org/Ipopt/documentation/node13.html
		
		# Installing LAPACK and BLAS
		if [ ! -f "$INSTALL_DEPS_PREFIX/lib/liblapack.so" ]; then
			install_lapack
		fi

		cd $CURRENT_DIR/thirdparty
		cd ipopt/ThirdParty
		# Getting Metis dependency
		cd Metis
#		sed -i 's/metis\/metis/metis\/OLD\/metis/g' get.Metis
#		sed -i 's/metis-4\.0/metis-4\.0\.3/g' get.Metis
#		sed -i 's/mv metis/#mv metis/g' get.Metis
		./get.Metis
		# Patching is necessary. See http://www.math-linux.com/mathematics/Linear-Systems/How-to-patch-metis-4-0-error
		wget http://www.math-linux.com/IMG/patch/metis-4.0.patch
		#curl -L "http://www.math-linux.com/IMG/patch/metis-4.0.patch" -O metis-4.0.patch
		patch -p0 < metis-4.0.patch
		cd ..
		# Getting Mumps dependency
		cd Mumps
		./get.Mumps
		cd ..
		# Getting ASL dependency
		cd ASL
		wget --recursive --include-directories=ampl/solvers http://www.netlib.org/ampl/solvers || true
		rm -rf solvers
		mv www.netlib.org/ampl/solvers .
		rm -rf www.netlib.org/
		sed -i '.original' 's/^rm/# rm/g' get.ASL
		sed -i '.original' 's/^tar /# tar/g' get.ASL
		sed -i '.original' 's/^$wgetcmd/# $wgetcmd/g' get.ASL
		cd ..
		# bugfix of http://bugs.debian.org/cgi-bin/bugreport.cgi?bug=625018#10
		cd ..
		sed -n 'H;${x;s/#include "IpReferenced.hpp"/#include <cstddef>\
		\
		&/;p;}' Ipopt/src/Common/IpSmartPtr.hpp > IpSmartPtr.hpp
		mv IpSmartPtr.hpp Ipopt/src/Common/IpSmartPtr.hpp
		sed -n 'H;${x;s/#include <list>/&\
		#include <cstddef>/;p;}' Ipopt/src/Algorithm/LinearSolvers/IpTripletToCSRConverter.cpp > IpTripletToCSRConverter.cpp
		mv IpTripletToCSRConverter.cpp Ipopt/src/Algorithm/LinearSolvers/IpTripletToCSRConverter.cpp
		# create build directory
		mkdir -p build
		../configure --enable-static --prefix $INSTALL_DEPS_PREFIX --with-lapack="-L$INSTALL_DEPS_PREFIX/lib -llapack" --with-blas="-L$INSTALL_DEPS_PREFIX/lib -lblas"
		sudo make -j install
	elif [ "$CURRENT_OS" == "UBUNTU" ]; then
		# Installing necessary packages
		sudo apt-get install f2c libf2c2-dev libf2c2 gfortran

		# Getting Ipopt 3.12.4
		wget http://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.4.tgz
		mkdir ipopt && tar xzvf Ipopt-3.12.4.tgz -C ipopt --strip-components 1
		rm -rf Ipopt-3.12.4.tgz
		# Documentation for Ipopt Third Party modules:
		# http://www.coin-or.org/Ipopt/documentation/node13.html

		# Installing LAPACK and BLAS
		if [ ! -f "$INSTALL_DEPS_PREFIX/lib/liblapack.so" ]; then
			install_lapack
		fi

		cd $CURRENT_DIR/thirdparty
		cd ipopt/ThirdParty
		# Getting Metis dependency
		cd Metis
#		sed -i 's/metis\/metis/metis\/OLD\/metis/g' get.Metis
#		sed -i 's/metis-4\.0/metis-4\.0\.3/g' get.Metis
#		sed -i 's/mv metis/#mv metis/g' get.Metis
		./get.Metis
		# Patching is necessary. See http://www.math-linux.com/mathematics/Linear-Systems/How-to-patch-metis-4-0-error
		wget http://www.math-linux.com/IMG/patch/metis-4.0.patch
		patch -p0 < metis-4.0.patch
		cd ..
		# Getting Mumps dependency
		cd Mumps
		./get.Mumps
		cd ..
		# Getting ASL dependency
		cd ASL
		wget --recursive --include-directories=ampl/solvers http://www.netlib.org/ampl/solvers || true
		rm -rf solvers
		mv www.netlib.org/ampl/solvers .
		rm -rf www.netlib.org/
		sed -i 's/^rm/# rm/g' get.ASL
		sed -i 's/^tar /# tar/g' get.ASL
		sed -i 's/^$wgetcmd/# $wgetcmd/g' get.ASL
		cd ..
		# bugfix of http://bugs.debian.org/cgi-bin/bugreport.cgi?bug=625018#10
		cd ..
		sed -n 'H;${x;s/#include "IpReferenced.hpp"/#include <cstddef>\
		\
		&/;p;}' Ipopt/src/Common/IpSmartPtr.hpp > IpSmartPtr.hpp
		mv IpSmartPtr.hpp Ipopt/src/Common/IpSmartPtr.hpp
		sed -n 'H;${x;s/#include <list>/&\
		#include <cstddef>/;p;}' Ipopt/src/Algorithm/LinearSolvers/IpTripletToCSRConverter.cpp > IpTripletToCSRConverter.cpp
		mv IpTripletToCSRConverter.cpp Ipopt/src/Algorithm/LinearSolvers/IpTripletToCSRConverter.cpp
		# create build directory
		mkdir -p build
		cd build
		# start building
		../configure --enable-static --prefix $INSTALL_DEPS_PREFIX --with-lapack="-L$INSTALL_DEPS_PREFIX/lib -llapack" --with-blas="-L$INSTALL_DEPS_PREFIX/lib -lblas"
		sudo make -j install
	fi
}


function install_qpoases
{
	# Remove old folder (sanity procedure)
	cd $CURRENT_DIR/thirdparty
	sudo rm -rf qpOASES

	if [ "$CURRENT_OS" == "OSX" ]; then
		echo -e "${COLOR_WARN}Mac OSX installation not support yet${COLOR_RESET}"
		# Getting the qpOASES 3.2.0
		wget http://www.coin-or.org/download/source/qpOASES/qpOASES-3.2.0.tgz
		tar xzfv qpOASES-3.2.0.tgz && rm qpOASES-3.2.0.tgz
		mv qpOASES-3.2.0 qpOASES
		
		# Installing LAPACK and BLAS
		if [ ! -f "$INSTALL_DEPS_PREFIX/lib/liblapack.so" ]; then
			install_lapack
		fi

		cd $CURRENT_DIR/thirdparty
		cd qpOASES
		sudo make -j BINDIR=$INSTALL_DEPS_PREFIX/lib REPLACE_LINALG=0 LIB_LAPACK=$INSTALL_DEPS_PREFIX/lib/liblapack.so LIB_BLAS=$INSTALL_DEPS_PREFIX/lib/libblas.so
		mkdir build
		cd build
		cmake ../
		make -j4
		sudo make -j4 install
	elif [ "$CURRENT_OS" == "UBUNTU" ]; then
		# Getting the qpOASES 3.2.0
		wget http://www.coin-or.org/download/source/qpOASES/qpOASES-3.2.0.tgz
		tar xzfv qpOASES-3.2.0.tgz && rm qpOASES-3.2.0.tgz
		mv qpOASES-3.2.0 qpOASES

		# Installing LAPACK and BLAS
		if [ ! -f "$INSTALL_DEPS_PREFIX/lib/liblapack.so" ]; then
			install_lapack
		fi

		cd $CURRENT_DIR/thirdparty
		cd qpOASES
		sudo make -j BINDIR=/usr/local/lib REPLACE_LINALG=0 LIB_LAPACK=$INSTALL_DEPS_PREFIX/lib/liblapack.so LIB_BLAS=$INSTALL_DEPS_PREFIX/lib/libblas.so
		mkdir build
		cd build
#		cmake ../
		cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_DEPS_PREFIX ../
		make -j4
		sudo make -j4 install
	fi
}


function install_libcmaes
{
	# Remove old folder (sanity procedure)
	cd $CURRENT_DIR/thirdparty
	sudo rm -rf libcmaes

	if [ "$CURRENT_OS" == "OSX" ]; then
		echo -e "${COLOR_WARN}Mac OSX installation not support yet${COLOR_RESET}"

		# Compiling and installing Google unit test framework
		curl -L https://github.com/google/googletest/archive/release-1.8.0.tar.gz | tar xz
		mv googletest-release-1.8.0/ gtest
		cd gtest
		mkdir -p build
		cd build
		cmake -D BUILD_SHARED_LIBS:bool=ON  CMAKE_BUILD_TYPE=Release ../
		sudo make -j install
		cd $CURRENT_DIR/thirdparty

		# Getting the libcmaes 0.9.5
		wget https://github.com/beniz/libcmaes/archive/0.9.5.tar.gz
		mkdir libcmaes && tar zxf 0.9.5.tar.gz -C libcmaes --strip-components 1
		rm -rf 0.9.5.tar.gz
		cd libcmaes
		./autogen.sh
		./configure --enable-gglog --prefix=$INSTALL_DEPS_PREFIX --with-eigen3-include=$INSTALL_DEPS_PREFIX/include/eigen3
		sudo make -j4 install
	elif [ "$CURRENT_OS" == "UBUNTU" ]; then
		# Installing libcmaes dependecies
		sudo apt-get install autoconf automake libtool libgoogle-glog-dev libgflags-dev

		# Compiling and installing Google unit test framework
		cd /usr/src/gtest
		sudo mkdir -p build
		cd build
		sudo cmake ../
		sudo make
		sudo cp *.a /usr/lib
		cd $CURRENT_DIR/thirdparty

		# Getting the libcmaes 0.9.5
		wget https://github.com/beniz/libcmaes/archive/0.9.5.tar.gz
		mkdir libcmaes && tar zxf 0.9.5.tar.gz -C libcmaes --strip-components 1
		rm -rf 0.9.5.tar.gz
		cd libcmaes
		./autogen.sh
		./configure --enable-gglog --prefix=$INSTALL_DEPS_PREFIX --with-eigen3-include=$INSTALL_DEPS_PREFIX/include/eigen3
		sudo make -j4 install
	fi
}


function install_pyadolc
{
	# Remove old folder (sanity procedure)
	cd $CURRENT_DIR/thirdparty
	sudo rm -rf colpack

	# Installing ColPAck
	# Download ColPack
	wget -O ColPack-1.0.10.tar.gz https://github.com/CSCsw/ColPack/archive/v1.0.10.tar.gz

	# build ColPack
	tar xfvz ColPack-1.0.10.tar.gz
	rm ColPack-1.0.10.tar.gz
	mv ColPack-1.0.10 colpack
	cd colpack
	autoreconf -vif
	./configure --prefix=$INSTALL_DEPS_PREFIX --libdir='${prefix}/lib'
	make
	sudo make install


	# Remove old folder (sanity procedure)
	cd $CURRENT_DIR/thirdparty
	sudo rm -rf adolc

	# Download ADOL-C
	wget http://www.coin-or.org/download/source/ADOL-C/ADOL-C-2.6.0.tgz
	tar xfvz ADOL-C-2.6.0.tgz
	rm ADOL-C-2.6.0.tgz
	mv ADOL-C-2.6.0 adolc
	cd adolc
	./update_versions.sh
	./configure --enable-sparse --with-colpack=$INSTALL_DEPS_PREFIX --prefix=$INSTALL_DEPS_PREFIX
	make
	sudo make install


	# Remove old folder (sanity procedure)
	cd $CURRENT_DIR/thirdparty
	sudo rm -rf pyadolc

	# Clone pyadolc
	git clone git@github.com:robot-locomotion/pyadolc.git
	cd pyadolc

	export BOOST_DIR=$INSTALL_DEPS_PREFIX
	export ADOLC_DIR=$INSTALL_DEPS_PREFIX
	export COLPACK_DIR=$INSTALL_DEPS_PREFIX
	sudo python setup.py install
}


function install_octomap
{
	# Remove old folder (sanity procedure)
	cd $CURRENT_DIR/thirdparty
	sudo rm -rf octomap

	if [ "$CURRENT_OS" == "OSX" ]; then
		# Getting Octomap 1.6.8
		curl -L "https://github.com/OctoMap/octomap/archive/v1.6.8.tar.gz" | tar xj
		mv octomap-*/ octomap
		cd octomap
		mkdir -p build
		cd build
		cmake ../
		sudo make -j install
	elif [ "$CURRENT_OS" == "UBUNTU" ]; then
		# Getting Octomap 1.6.8
		wget https://github.com/OctoMap/octomap/archive/v1.6.8.tar.gz
		mkdir octomap && tar zxf v1.6.8.tar.gz -C octomap --strip-components 1
		rm -rf v1.6.8.tar.gz
		cd octomap
		mkdir -p build
		cd build
#		cmake ../
		cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$INSTALL_DEPS_PREFIX ../
		sudo make -j install
	fi
}


function install_gnuplot
{
	# Remove old folder (sanity procedure)
	cd $CURRENT_DIR/thirdparty
	sudo rm -rf gnuplot

	if [ "$CURRENT_OS" == "OSX" ]; then
		echo -e "${COLOR_WARN}Mac OSX installation not support yet${COLOR_RESET}"
	elif [ "$CURRENT_OS" == "UBUNTU" ]; then
		# Installing qt5
		sudo apt-get install qt5-default

		# Getting the gnuplot 5.0.3
		wget https://sourceforge.net/projects/gnuplot/files/gnuplot/5.0.3/gnuplot-5.0.3.tar.gz
		mkdir gnuplot && tar zxf gnuplot-5.0.3.tar.gz -C gnuplot --strip-components 1
		rm -rf gnuplot-5.0.3.tar.gz
		cd gnuplot
		./configure
		sudo make install
		cd ../
	fi
}




##############################################  MAIN  ########################################################
# Printing the information of the shell script
echo -e "${COLOR_BOLD}install_deps.sh - DWL Installation Script for Ubuntu Trusty Tahr 14.04${COLOR_RESET}"
echo ""
echo "Copyright (C) 2015-2018 Carlos Mastalli"
echo "All rights reserved."
echo "Released under the BSD 3-Clause License."
echo ""
echo "This program comes with ABSOLUTELY NO WARRANTY."
echo "This is free software, and you are welcome to redistribute it"
echo "under certain conditions; see the LICENSE text file for more information."

# This file is part of DWL Installer.
#
# DWL Installer is free software: you can redistribute it and/or
# modify it under the terms of the BSD 3-Clause License



## Detecting the current OS and distro
findCurrentOSType

echo ""
read -s -p "Press enter to start the installation. " 
echo ""

echo -e -n "${COLOR_QUES}Do you want to install the thirdparties in $INSTALL_DEPS_PREFIX [Y/n]: ${COLOR_RESET}"
read ANSWER_PATH
if [ "$ANSWER_PATH" == "N" ] || [ "$ANSWER_PATH" == "n" ]; then
echo -e -n "${COLOR_QUES}Please write absolute path: ${COLOR_RESET}"
read ANSWER_PATH_STRING
INSTALL_DEPS_PREFIX=$ANSWER_PATH_STRING
fi


mkdir -p ${CURRENT_DIR}/thirdparty
cd ${CURRENT_DIR}/thirdparty

# Added doxygen install. TODO moved from here and tested for other OS (i.e. Mac OSX)
sudo apt-get install doxygen

##---------------------------------------------------------------##
##---------------------- Installing Eigen -----------------------##
##---------------------------------------------------------------##
echo ""
echo -e "${COLOR_BOLD}Installing Eigen ...${COLOR_RESET}"
if [ -d "$INSTALL_DEPS_PREFIX/include/eigen3" ] || [ -d "$COMMON_INSTALL_PREFIX/include/eigen3" ]; then
	echo -e -n "${COLOR_QUES}Do you want to re-install Eigen 3.2.10? [y/N]: ${COLOR_RESET}"
	read ANSWER_EIGEN
	if [ "$ANSWER_EIGEN" == "Y" ] || [ "$ANSWER_EIGEN" == "y" ]; then
		install_eigen
    fi
else
	install_eigen
fi


##---------------------------------------------------------------##
##----------------------- Installing URDF -----------------------##
##---------------------------------------------------------------##
#echo ""
#echo -e "${COLOR_BOLD}Installing URDF facilities ...${COLOR_RESET}"
## Installing urdfdom_headers
#if [ -d "/usr/include/urdf_model" ] || [ -d "/usr/local/include/urdf_model" ]; then
#	echo -e -n "${COLOR_QUES}Do you want to re-install URDFDOM Headers? [y/N]: ${COLOR_RESET}"
#	read ANSWER_URDF
#	if [ "${ANSWER_URDF}" == "Y" ] || [ "${ANSWER_URDF}" == "y" ]; then
#		install_urdfdom_headers
#	fi
#else
#	install_urdfdom_headers
#fi
## Installing console_bridge
#if [ -d "/usr/include/console_bridge" ] || [ -d "/usr/local/include/console_bridge" ]; then
#	echo -e -n "${COLOR_QUES}Do you want to re-install console bridge? [y/N]: ${COLOR_RESET}"
#	read ANSWER_CONSOLE
#	if [ "${ANSWER_CONSOLE}" == "Y" ] || [ "${ANSWER_CONSOLE}" == "y" ]; then
#		install_console_bridge
#	fi
#else
#	install_console_bridge
#fi
# Installing urdfdom
#if [ -d "/usr/include/urdf_parser" ] || [ -d "/usr/local/include/urdf_parser" ]; then
#	echo -e -n "${COLOR_QUES}Do you want to re-install urdf parser? [y/N]: ${COLOR_RESET}"
#	read ANSWER_URDFPARSER
#	if [ "${ANSWER_URDFPARSER}" == "Y" ] || [ "${ANSWER_URDFPARSER}" == "y" ]; then
#		install_urdfdom
#	fi
#else
#	install_urdfdom
#fi


##---------------------------------------------------------------##
##----------------------- Installing RBDL -----------------------##
##---------------------------------------------------------------##
echo ""
echo -e "${COLOR_BOLD}Installing RBDL ...${COLOR_RESET}"
if [ -d "$INSTALL_DEPS_PREFIX/include/rbdl" ] || [ -d "$COMMON_INSTALL_PREFIX/include/rbdl" ]; then
	echo -e -n "${COLOR_QUES}Do you want to re-install RBDL 2.4.0? [y/N]: ${COLOR_RESET}"
	read ANSWER_RBDL
	if [ "$ANSWER_RBDL" == "Y" ] || [ "$ANSWER_RBDL" == "y" ]; then
		install_rbdl
    fi
else
	install_rbdl
fi


##---------------------------------------------------------------##
##-------------------- Installing YAML-CPP ----------------------##
##---------------------------------------------------------------##
echo ""
echo -e "${COLOR_BOLD}Installing YAML-CPP ...${COLOR_RESET}"
if [ -d "$INSTALL_DEPS_PREFIX/include/yaml-cpp" ] || [ -d "$COMMON_INSTALL_PREFIX/include/yaml-cpp" ]; then
	echo -e -n "${COLOR_QUES}Do you want to re-install YAML-CPP 0.5.2? [y/N]: ${COLOR_RESET}"
	read ANSWER_YAMLCPP
	if [ "$ANSWER_YAMLCPP" == "Y" ] || [ "$ANSWER_YAMLCPP" == "y" ]; then
		install_yamlcpp
    fi
else
	install_yamlcpp
fi



##---------------------------------------------------------------##
##------------------------ Installing SWIG ----------------------##
##---------------------------------------------------------------##
echo ""
echo -e "${COLOR_BOLD}Installing SWIG ...${COLOR_RESET}"
if [ -d "$INSTALL_DEPS_PREFIX/share/swig" ] || [ -d "$COMMON_INSTALL_PREFIX/share/swig" ]; then
	echo -e -n "${COLOR_QUES}Do you want to re-install SWIG 3.0.12? [y/N]: ${COLOR_RESET}"
	read ANSWER_SWIG
	if [ "$ANSWER_SWIG" == "Y" ] || [ "$ANSWER_SWIG" == "y" ]; then
		install_swig
    fi
else
	install_swig
fi


##---------------------------------------------------------------##
##---------------------- Installing Ipopt -----------------------##
##---------------------------------------------------------------##
echo ""
echo -e "${COLOR_BOLD}Installing Ipopt ...${COLOR_RESET}"
if [ -d "$INSTALL_DEPS_PREFIX/include/coin" ] || [ -d "$COMMON_INSTALL_PREFIX/include/coin" ]; then
	# Control will enter here if $DIRECTORY exists.
	echo -e -n "${COLOR_QUES}Do you want to re-install Ipopt 3.12.4? [y/N]: ${COLOR_RESET}"
	read ANSWER_IPOPT
	if [ "$ANSWER_IPOPT" == "Y" ] || [ "$ANSWER_IPOPT" == "y" ]; then
		install_ipopt
    fi
else
	echo -e -n "${COLOR_QUES}Do you want to install Ipopt 3.12.4? [y/N]: ${COLOR_RESET}"
	read ANSWER_IPOPT
	if [ "$ANSWER_IPOPT" == "Y" ] || [ "$ANSWER_IPOPT" == "y" ]; then
		install_ipopt
	fi
fi


##---------------------------------------------------------------##
##--------------------- Installing qpOASES ----------------------##
##---------------------------------------------------------------##
echo ""
echo -e "${COLOR_BOLD}Installing qpOASES ...${COLOR_RESET}"
if [ -d "$INSTALL_DEPS_PREFIX/include/qpOASES" ] || [ -d "$COMMON_INSTALL_PREFIX/include/qpOASES" ]; then
	# Control will enter here if $DIRECTORY exists.
	echo -e -n "${COLOR_QUES}Do you want to re-install qpOASES 3.2.0? [y/N]: ${COLOR_RESET}"
	read ANSWER_QPOASES
	if [ "$ANSWER_QPOASES" == "Y" ] || [ "$ANSWER_QPOASES" == "y" ]; then
		install_qpoases
    fi
else
	echo -e -n "${COLOR_QUES}Do you want to install qpOASES 3.2.0? [y/N]: ${COLOR_RESET}"
	read ANSWER_QPOASES
	if [ "$ANSWER_QPOASES" == "Y" ] || [ "$ANSWER_QPOASES" == "y" ]; then
		install_qpoases
	fi
fi


##---------------------------------------------------------------##
##--------------------- Installing CMA-ES -----------------------##
##---------------------------------------------------------------##
echo ""
echo -e "${COLOR_BOLD}Installing libcmaes ...${COLOR_RESET}"
if [ -d "$INSTALL_DEPS_PREFIX/include/libcmaes" ] || [ -d "$COMMON_INSTALL_PREFIX/include/yaml-cpp" ]; then
	# Control will enter here if $DIRECTORY exists.
	echo -e -n "${COLOR_QUES}Do you want to re-install libcmaes 0.9.5? [y/N]: ${COLOR_RESET}"
	read ANSWER_LIBCMAES
	if [ "$ANSWER_LIBCMAES" == "Y" ] || [ "$ANSWER_LIBCMAES" == "y" ]; then
		install_libcmaes
    fi
else
	echo -e -n "${COLOR_QUES}Do you want to install libcmaes 0.9.5? [y/N]: ${COLOR_RESET}"
	read ANSWER_LIBCMAES
	if [ "$ANSWER_LIBCMAES" == "Y" ] || [ "$ANSWER_LIBCMAES" == "y" ]; then
		install_libcmaes
	fi
fi


##---------------------------------------------------------------##
##-------------------- Installing PyADOLC -----------------------##
##---------------------------------------------------------------##
echo ""
echo -e "${COLOR_BOLD}Installing pyadolc ...${COLOR_RESET}"
if [ -d "/usr/local/lib/python2.7/dist-packages/adolc" ]; then
	# Control will enter here if $DIRECTORY exists.
	echo -e -n "${COLOR_QUES}Do you want to re-install pyadolc? [y/N]: ${COLOR_RESET}"
	read ANSWER_PYADOLC
	if [ "$ANSWER_LIBCMAES" == "Y" ] || [ "$ANSWER_LIBCMAES" == "y" ]; then
		install_pyadolc
    fi
else
	echo -e -n "${COLOR_QUES}Do you want to install pyadolc? [y/N]: ${COLOR_RESET}"
	read ANSWER_PYADOLC
	if [ "$ANSWER_LIBCMAES" == "Y" ] || [ "$ANSWER_LIBCMAES" == "y" ]; then
		install_pyadolc
	fi
fi


##---------------------------------------------------------------##
##--------------------- Installing Octomap ----------------------##
##---------------------------------------------------------------##
#echo ""
#echo -e "${COLOR_BOLD}Installing Octomap ...${COLOR_RESET}"
#if [ -d "/usr/local/include/octomap" ]; then
#	echo -e -n "${COLOR_QUES}Do you want to re-install Octomap 1.6.8? [y/N]: ${COLOR_RESET}"
#	read ANSWER_OCTOMAP
#	if [ "$ANSWER_OCTOMAP" == "Y" ] || [ "$ANSWER_OCTOMAP" == "y" ]; then
#		install_octomap
#    fi
#else
#	echo -e -n "${COLOR_QUES}Do you want to install Octomap 1.6.8? [y/N]: ${COLOR_RESET}"
#	read ANSWER_OCTOMAP
#	if [ "$ANSWER_OCTOMAP" == "Y" ] || [ "$ANSWER_OCTOMAP" == "y" ]; then
#		install_octomap
#	fi
#fi


##---------------------------------------------------------------##
##-------------------- Installing gnuplot ----------------------##
##---------------------------------------------------------------##
#echo ""
#echo -e "${COLOR_BOLD}Installing gnuplot ...${COLOR_RESET}"
#if [ -x "/usr/local/bin/gnuplot" ]; then
#	echo -e -n "${COLOR_QUES}Do you want to re-install gnuplot 5.0.3? [y/N]: ${COLOR_RESET}"
#	read ANSWER_GNUPLOT
#	if [ "$ANSWER_GNUPLOT" == "Y" ] || [ "$ANSWER_GNUPLOT" == "y" ]; then
#		install_gnuplot
#    fi
#else
#	echo -e -n "${COLOR_QUES}Do you want to install gnuplot 5.0.3? [y/N]: ${COLOR_RESET}"
#	read ANSWER_GNUPLOT
#	if [ "$ANSWER_GNUPLOT" == "Y" ] || [ "$ANSWER_GNUPLOT" == "y" ]; then
#		install_gnuplot
#	fi
#fi
