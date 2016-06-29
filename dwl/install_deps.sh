#!/bin/bash

# List of usefull colors
COLOR_RESET="\033[0m"
COLOR_INFO="\033[0;32m"
COLOR_ITEM="\033[1;34m"
COLOR_QUES="\033[1;32m"
COLOR_WARN="\033[0;33m"
COLOR_BOLD="\033[1m"
COLOR_UNDE="\033[4m"

DWL_INSTALL_PREFIX=/usr/local/dwl


function install_eigen
{
	# Remove old folder (sanity procedure)
	rm -rf eigen

	# Getting Eigen 3.2.7
	wget http://www.bitbucket.org/eigen/eigen/get/3.2.7.tar.bz2
	mkdir eigen && tar jxf 3.2.7.tar.bz2 -C eigen --strip-components 1
	rm -rf 3.2.7.tar.bz2
	cd eigen
	mkdir -p build
	cd build
	cmake -DCMAKE_INSTALL_PREFIX=$DWL_INSTALL_PREFIX -DEIGEN_INCLUDE_INSTALL_DIR=$DWL_INSTALL_PREFIX/include/eigen3 -Dpkg_config_libdir=$DWL_INSTALL_PREFIX/lib/ ../
	sudo make -j install
	cd ../../
}


function install_rbdl
{
	# Remove old folder (sanity procedure)
	rm -rf rbdl

	# Getting RBDL 2.4.0
	wget https://bitbucket.org/rbdl/rbdl/get/v2.4.0.zip
	unzip v2.4.0.zip
	rm v2.4.0.zip
	mv rbdl-rbdl-* rbdl
	cd rbdl
	mkdir -p build
	cd build
	cmake -D RBDL_BUILD_ADDON_URDFREADER:bool=ON -D CMAKE_INSTALL_LIBDIR:string=lib -DCMAKE_INSTALL_PREFIX=$DWL_INSTALL_PREFIX ../
	make -j
	sudo make -j install
	cd ../../
}


function install_urdfdom_headers
{
	# Remove old folder (sanity procedure)
	rm -rf urdf_headers

	# Getting urdfdom_headers 0.2.3
	wget https://github.com/ros/urdfdom_headers/archive/0.2.3.tar.gz
	mkdir urdfdom_headers && tar zxf 0.2.3.tar.gz -C urdfdom_headers --strip-components 1
	rm -rf 0.2.3.tar.gz
	cd urdfdom_headers
	mkdir -p build
	cd build
	cmake ../
	sudo make -j install
	cd ../../
}


function install_console_bridge
{
	# Remove old folder (sanity procedure)
	rm -rf console_bridge

	# Getting console_bridge 0.2.7
	wget https://github.com/ros/console_bridge/archive/0.2.7.tar.gz
	mkdir console_bridge && tar zxf 0.2.7.tar.gz -C console_bridge --strip-components 1
	rm -rf 0.2.7.tar.gz
	cd console_bridge
	mkdir -p build
	cd build
	cmake ../
	sudo make -j install
	cd ../../
}


function install_urdfdom
{
	# Remove old folder (sanity procedure)
	rm -rf urdfdom

	# Getting console_bridge 0.2.10
	wget https://github.com/ros/urdfdom/archive/0.2.10.tar.gz
	mkdir urdfdom && tar zxf 0.2.10.tar.gz -C urdfdom --strip-components 1
	rm -rf 0.2.10.tar.gz
	cd urdfdom
	mkdir -p build
	cd build
	cmake ../
	sudo make -j install
	cd ../../
}


function install_yamlcpp
{
	# Remove old folder (sanity procedure)
	rm -rf yaml-cpp

	# Getting the YAML-CPP 0.5.1
	wget https://github.com/jbeder/yaml-cpp/archive/release-0.5.1.zip
	unzip release-0.5.1.zip && rm -rf release-0.5.1.zip
	mv yaml-cpp-release-0.5.1 yaml-cpp
	cd yaml-cpp
	mkdir -p build
	cd build
	cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=$DWL_INSTALL_PREFIX ../
	sudo make -j install
	cd ../../
}


function install_lapack
{
	# Remove old folder (sanity procedure)
	rm -rf lapack

	# Getting the LAPACK 3.6.0
	wget http://www.netlib.org/lapack/lapack-3.6.0.tgz
	mkdir lapack && tar xzvf lapack-3.6.0.tgz -C lapack --strip-components 1
	rm -rf lapack-3.6.0.tgz
	cd lapack
	mkdir -p build
	cd build
	cmake -D BUILD_SHARED_LIBS:bool=ON -D CMAKE_INSTALL_LIBDIR:string=lib -D CMAKE_INSTALL_PREFIX=$DWL_INSTALL_PREFIX ../
	sudo make -j install
	cd ../../
}


function install_ipopt
{
	# Remove old folder (sanity procedure)
	rm -rf ipopt

	# Installing necessary packages
	sudo apt-get install f2c libf2c2-dev libf2c2 gfortran

	# Getting Ipopt 3.12.4
	wget http://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.4.tgz
	mkdir ipopt && tar xzvf Ipopt-3.12.4.tgz -C ipopt --strip-components 1
	rm -rf Ipopt-3.12.4.tgz
	# Documentation for Ipopt Third Party modules:
	# http://www.coin-or.org/Ipopt/documentation/node13.html

	# Installing LAPACK and BLAS
	if [ ! -f "$DWL_INSTALL_PREFIX/lib/liblapack.so" ]; then
		install_lapack
	fi

	cd ipopt/ThirdParty
	# Getting Metis dependency
	cd Metis
#	sed -i 's/metis\/metis/metis\/OLD\/metis/g' get.Metis
#	sed -i 's/metis-4\.0/metis-4\.0\.3/g' get.Metis
#	sed -i 's/mv metis/#mv metis/g' get.Metis
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
	../configure --enable-static --prefix $DWL_INSTALL_PREFIX --with-lapack="-L$DWL_INSTALL_PREFIX/lib -llapack" --with-blas="-L$DWL_INSTALL_PREFIX/lib -lblas"
	sudo make -j install
	cd ../../
}



function install_qpoases
{
	# Remove old folder (sanity procedure)
	rm -rf qpOASES

	# Getting the qpOASES 3.2.0
	wget http://www.coin-or.org/download/source/qpOASES/qpOASES-3.2.0.tgz
	tar xzfv qpOASES-3.2.0.tgz && rm qpOASES-3.2.0.tgz
	mv qpOASES-3.2.0 qpOASES

	# Installing LAPACK and BLAS
	if [ ! -f "$DWL_INSTALL_PREFIX/lib/liblapack.so" ]; then
		install_lapack
	fi

	cd qpOASES
	make -j REPLACE_LINALG=0 LIB_LAPACK=$DWL_INSTALL_PREFIX/lib/liblapack.so LIB_BLAS=$DWL_INSTALL_PREFIX/lib/libblas.so
	cd ../
}


function install_libcmaes
{
	# Remove old folder (sanity procedure)
	rm -rf libcmaes

	# Installing libcmaes dependecies
	sudo apt-get install autoconf automake libtool libgoogle-glog-dev libgflags-dev
	
	# Getting the current path
	CURRENT_PATH=$(pwd -P)
	
	# Compiling and installing Google unit test framework
	cd /usr/src/gtest
	sudo mkdir -p build
	cd build
	sudo cmake ../
	sudo make
	sudo cp *.a /usr/lib
	cd $CURRENT_PATH
	
	# Getting the libcmaes 0.9.5
	wget https://github.com/beniz/libcmaes/archive/0.9.5.tar.gz
	mkdir libcmaes && tar zxf 0.9.5.tar.gz -C libcmaes --strip-components 1
	rm -rf 0.9.5.tar.gz
	cd libcmaes
	./autogen.sh
	./configure --enable-gglog --prefix=$DWL_INSTALL_PREFIX --with-eigen3-include=$DWL_INSTALL_PREFIX/include/eigen3
	sudo make -j4 install
	cd ../
}


function install_octomap
{
	# Remove old folder (sanity procedure)
	rm -rf octomap

	# Getting Octomap 1.6.8
	wget https://github.com/OctoMap/octomap/archive/v1.6.8.tar.gz
	mkdir octomap && tar zxf v1.6.8.tar.gz -C octomap --strip-components 1
	rm -rf v1.6.8.tar.gz
	cd octomap
	mkdir -p build
	cd build
	cmake ../
	sudo make -j install
	cd ../../
}


function install_gnuplot
{
	# Remove old folder (sanity procedure)
	rm -rf gnuplot

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
}

##############################################  MAIN  ########################################################
# Getting the path of the install_deps.sh file
SELF_PATH=$(cd -P -- "$(dirname -- "$0")" && pwd -P)

# Printing the information of the shell script
echo -e "${COLOR_BOLD}install_deps.sh - DWL Installation Script for Ubuntu Precise Pangolin 12.04 and Ubuntu Trusty Tahr 14.04${COLOR_RESET}"
echo ""
echo "Copyright (C) 2015 Carlos Mastalli"
echo ""
echo "This program comes with ABSOLUTELY NO WARRANTY."
echo "This is free software, and you are welcome to redistribute it"
echo "under certain conditions; see the filecontent for more information."

# This file is part of DWL Installer.
#
# Psopt Installer is free software: you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# Psopt Installer is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with DWL Installer.  If not, see
# <http://www.gnu.org/licenses/>.

echo ""
read -s -p "Press enter to start the installation. " 
echo ""


mkdir -p ${SELF_PATH}/thirdparty
cd ${SELF_PATH}/thirdparty

##---------------------------------------------------------------##
##---------------------- Installing Eigen -----------------------##
##---------------------------------------------------------------##
echo ""
echo -e "${COLOR_BOLD}Installing Eigen ...${COLOR_RESET}"
if [ -d "$DWL_INSTALL_PREFIX/include/eigen3" ]; then
	echo -e -n "${COLOR_QUES}Do you want to re-install Eigen 3.2.7? [y/N]: ${COLOR_RESET}"
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
echo ""
echo -e "${COLOR_BOLD}Installing URDF facilities ...${COLOR_RESET}"
# Installing urdfdom_headers
if [ -d "/usr/include/urdf_model" ] || [ -d "/usr/local/include/urdf_model" ]; then
	echo -e -n "${COLOR_QUES}Do you want to re-install URDFDOM Headers? [y/N]: ${COLOR_RESET}"
	read ANSWER_URDF
	if [ "${ANSWER_URDF}" == "Y" ] || [ "${ANSWER_URDF}" == "y" ]; then
		install_urdfdom_headers
	fi
else
	install_urdfdom_headers
fi
# Installing console_bridge
if [ -d "/usr/include/console_bridge" ] || [ -d "/usr/local/include/console_bridge" ]; then
	echo -e -n "${COLOR_QUES}Do you want to re-install console bridge? [y/N]: ${COLOR_RESET}"
	read ANSWER_CONSOLE
	if [ "${ANSWER_CONSOLE}" == "Y" ] || [ "${ANSWER_CONSOLE}" == "y" ]; then
		install_console_bridge
	fi
else
	install_console_bridge
fi
# Installing urdfdom
if [ -d "/usr/include/urdf_parser" ] || [ -d "/usr/local/include/urdf_parser" ]; then
	echo -e -n "${COLOR_QUES}Do you want to re-install urdf parser? [y/N]: ${COLOR_RESET}"
	read ANSWER_URDFPARSER
	if [ "${ANSWER_URDFPARSER}" == "Y" ] || [ "${ANSWER_URDFPARSER}" == "y" ]; then
		install_urdfdom
	fi
else
	install_urdfdom
fi


##---------------------------------------------------------------##
##----------------------- Installing RBDL -----------------------##
##---------------------------------------------------------------##
echo ""
echo -e "${COLOR_BOLD}Installing RBDL ...${COLOR_RESET}"
if [ -d "$DWL_INSTALL_PREFIX/include/rbdl" ]; then
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
if [ -d "$DWL_INSTALL_PREFIX/include/yaml-cpp" ]; then
	echo -e -n "${COLOR_QUES}Do you want to re-install YAML-CPP 0.5.1? [y/N]: ${COLOR_RESET}"
	read ANSWER_YAMLCPP
	if [ "$ANSWER_YAMLCPP" == "Y" ] || [ "$ANSWER_YAMLCPP" == "y" ]; then
		install_yamlcpp
    fi
else
	install_yamlcpp
fi


##---------------------------------------------------------------##
##---------------------- Installing Ipopt -----------------------##
##---------------------------------------------------------------##
echo ""
echo -e "${COLOR_BOLD}Installing Ipopt ...${COLOR_RESET}"
if [ -d "$DWL_INSTALL_PREFIX/include/coin" ]; then
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
if [ -d "qpOASES" ]; then
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
if [ -d "$DWL_INSTALL_PREFIX/include/libcmaes" ]; then
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
##--------------------- Installing Octomap ----------------------##
##---------------------------------------------------------------##
echo ""
echo -e "${COLOR_BOLD}Installing Octomap ...${COLOR_RESET}"
if [ -d "/usr/local/include/octomap" ]; then
	echo -e -n "${COLOR_QUES}Do you want to re-install Octomap 1.6.8? [y/N]: ${COLOR_RESET}"
	read ANSWER_OCTOMAP
	if [ "$ANSWER_OCTOMAP" == "Y" ] || [ "$ANSWER_OCTOMAP" == "y" ]; then
		install_octomap
    fi
else
	echo -e -n "${COLOR_QUES}Do you want to install Octomap 1.6.8? [y/N]: ${COLOR_RESET}"
	read ANSWER_OCTOMAP
	if [ "$ANSWER_OCTOMAP" == "Y" ] || [ "$ANSWER_OCTOMAP" == "y" ]; then
		install_octomap
	fi
fi


##---------------------------------------------------------------##
##-------------------- Installing gnuplot ----------------------##
##---------------------------------------------------------------##
echo ""
echo -e "${COLOR_BOLD}Installing gnuplot ...${COLOR_RESET}"
if [ -x "/usr/local/bin/gnuplot" ]; then
	echo -e -n "${COLOR_QUES}Do you want to re-install gnuplot 5.0.3? [y/N]: ${COLOR_RESET}"
	read ANSWER_GNUPLOT
	if [ "$ANSWER_GNUPLOT" == "Y" ] || [ "$ANSWER_GNUPLOT" == "y" ]; then
		install_gnuplot
    fi
else
	echo -e -n "${COLOR_QUES}Do you want to install gnuplot 5.0.3? [y/N]: ${COLOR_RESET}"
	read ANSWER_GNUPLOT
	if [ "$ANSWER_GNUPLOT" == "Y" ] || [ "$ANSWER_GNUPLOT" == "y" ]; then
		install_gnuplot
	fi
fi
