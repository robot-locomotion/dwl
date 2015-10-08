#!/bin/bash

# List of usefull colors
COLOR_RESET="\033[0m"
COLOR_INFO="\033[0;32m"
COLOR_ITEM="\033[1;34m"
COLOR_QUES="\033[1;32m"
COLOR_WARN="\033[0;33m"
COLOR_BOLD="\033[1m"
COLOR_UNDE="\033[4m"


function install_eigen
{
	# Getting Eigen 3.2.4
	wget http://www.bitbucket.org/eigen/eigen/get/3.2.4.tar.bz2
	mkdir eigen && tar jxf 3.2.4.tar.bz2 -C eigen --strip-components 1
	cd eigen
	mkdir -p build
	cd build
	cmake ../
	sudo make install
	cd ../../
	rm -rf 3.2.4.tar.bz2
}


function install_rbdl
{
	# Getting RBDL 2.4.0
	wget https://bitbucket.org/rbdl/rbdl/get/default.zip
	unzip default.zip
	mv rbdl-rbdl-cfd302b3b418 rbdl
	cd rbdl
	mkdir -p build
	cd build
	cmake -D RBDL_BUILD_ADDON_URDFREADER:bool=ON ../
	cmake -D CMAKE_INSTALL_LIBDIR:string=lib ../
	sudo make install
	cd ../../
	rm default.zip
}


function install_urdfdom_headers
{
	# Getting urdfdom_headers 0.3.0
	wget https://github.com/ros/urdfdom_headers/archive/0.3.0.tar.gz
	mkdir urdfdom_headers && tar zxf 0.3.0.tar.gz -C urdfdom_headers --strip-components 1
	rm -rf 0.3.0.tar.gz
	cd urdfdom_headers
	mkdir -p build
	cd build
	cmake ../
	sudo make install
	cd ../../
}


function install_console_bridge
{
	# Getting console_bridge 0.2.7
	wget https://github.com/ros/console_bridge/archive/0.2.7.tar.gz
	mkdir console_bridge && tar zxf 0.2.7.tar.gz -C console_bridge --strip-components 1
	rm -rf 0.2.7.tar.gz
	cd console_bridge
	mkdir -p build
	cd build
	cmake ../
	sudo make install
	cd ../../
}


function install_urdfdom
{
	# Getting console_bridge 0.2.10
	wget https://github.com/ros/urdfdom/archive/0.2.10.tar.gz
	mkdir urdfdom && tar zxf 0.2.10.tar.gz -C urdfdom --strip-components 1
	rm -rf 0.2.10.tar.gz
	cd urdfdom
	mkdir -p build
	cd build
	cmake ../
	sudo make install
}


function install_odeint
{
	# Getting Odeint 2
	wget http://github.com/headmyshoulder/odeint-v2/tarball/master/headmyshoulder-odeint-v2-v2.4-141-g656e146.tar.gz
	mkdir odeint && tar zxf headmyshoulder-odeint-v2-v2.4-141-g656e146.tar.gz -C odeint --strip-components 1
	rm -rf headmyshoulder-odeint-v2-v2.4-141-g656e146.tar.gz
}


function install_yamlcpp
{
	# Getting the YAML-CPP 0.3.0
	wget https://yaml-cpp.googlecode.com/files/yaml-cpp-0.3.0.tar.gz
	tar zxf yaml-cpp-0.3.0.tar.gz
	cd yaml-cpp
	mkdir -p build
	cd build
	cmake -D BUILD_SHARED_LIBS:bool=ON ../
	sudo make install
	cd ../../
	rm -rf yaml-cpp-0.3.0.tar.gz
}


function install_ipopt
{
	# Installing necessary packages
	sudo apt-get install f2c libf2c2-dev libf2c2 gfortran

	# Getting Ipopt 3.12.2
	wget http://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.2.tgz
	mkdir ipopt && tar xzvf Ipopt-3.12.2.tgz -C ipopt --strip-components 1
	# Documentation for Ipopt Third Party modules:
	# http://www.coin-or.org/Ipopt/documentation/node13.html
	cd ipopt/ThirdParty
	# Getting Blas dependency
	cd Blas
	sed -i 's/ftp:/http:/g' get.Blas
	./get.Blas
	cd ..
	# Getting Lapack dependency
	cd Lapack
	sed -i 's/ftp:/http:/g' get.Lapack
	./get.Lapack
	cd ..
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
	../configure --enable-static --prefix ${SELF_PATH}/thirdparty/ipopt
	make install
	cd ../../
	rm -rf Ipopt-3.12.2.tgz
}


function install_octomap
{
	# Getting Octomap 1.6.8
	wget https://github.com/OctoMap/octomap/archive/v1.6.8.tar.gz
	mkdir octomap && tar zxf v1.6.8.tar.gz -C octomap --strip-components 1
	cd octomap
	mkdir -p build
	cd build
	cmake ../
	sudo make install
	cd ../../
	rm -rf v1.6.8.tar.gz
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
read -s -p "Press enter to start the installation." 
echo ""


mkdir -p ${SELF_PATH}/thirdparty
cd ${SELF_PATH}/thirdparty

##---------------------------------------------------------------##
##---------------------- Installing Eigen -----------------------##
##---------------------------------------------------------------##
echo ""
echo -e "${COLOR_BOLD}Installing Eigen ...${COLOR_RESET}"
echo ""
if [ -d "/usr/local/include/eigen3" ]; then
	echo -e "${COLOR_QUES}Do you want to re-install Eigen 3.2.4? (Y/N)${COLOR_RESET}"
	read ANSWER_EIGEN
	if [ "$ANSWER_EIGEN" == "Y" ] || [ "$ANSWER_EIGEN" == "y" ]; then
		install_eigen
    fi
else
	install_eigen
fi


##---------------------------------------------------------------##
##----------------------- Installing RBDL -----------------------##
##---------------------------------------------------------------##
echo ""
echo -e "${COLOR_BOLD}Installing RBDL ...${COLOR_RESET}"
echo ""
if [ -d "/usr/local/include/rbdl" ]; then
	echo -e "${COLOR_QUES}Do you want to re-install RBDL 2.4.0? (Y/N)${COLOR_RESET}"
	read ANSWER_RBDL
	if [ "$ANSWER_RBDL" == "Y" ] || [ "$ANSWER_RBDL" == "y" ]; then
		install_rbdl
    fi
else
	install_rbdl
fi


##---------------------------------------------------------------##
##----------------------- Installing URDF -----------------------##
##---------------------------------------------------------------##
echo ""
echo -e "${COLOR_BOLD}Installing URDF facilities ...${COLOR_RESET}"
echo ""
# Installing urdfdom_headers
if [ -d "/usr/include/urdf_model" ] || [ -d "/usr/local/include/urdf_model" ]; then
	echo -e "${COLOR_QUES}Do you want to re-install URDFDOM Headers? (Y/N)${COLOR_RESET}"
	read ANSWER_URDF
	if [ "${ANSWER_URDF}" == "Y" ] || [ "${ANSWER_URDF}" == "y" ]; then
		install_urdfdom_headers
	fi
else
	install_urdfdom_headers
fi
# Installing console_bridge
if [ -d "/usr/include/console_bridge" ] || [ -d "/usr/local/include/console_bridge" ]; then
	echo -e "${COLOR_QUES}Do you want to re-install console bridge? (Y/N)${COLOR_RESET}"
	read ANSWER_CONSOLE
	if [ "${ANSWER_CONSOLE}" == "Y" ] || [ "${ANSWER_CONSOLE}" == "y" ]; then
		install_console_bridge
	fi
else
	install_console_bridge
fi
# Installing urdfdom
if [ -d "/usr/include/urdf_parser" ] || [ -d "/usr/local/include/urdf_parser" ]; then
	echo -e "${COLOR_QUES}Do you want to re-install urdf parser? (Y/N)${COLOR_RESET}"
	read ANSWER_URDFPARSER
	if [ "${ANSWER_URDFPARSER}" == "Y" ] || [ "${ANSWER_URDFPARSER}" == "y" ]; then
		install_urdfdom
	fi
else
	install_urdfdom
fi


##---------------------------------------------------------------##
##--------------------- Installing Odeint -----------------------##
##---------------------------------------------------------------##
echo -e "${COLOR_BOLD}Installing Odeint ...${COLOR_RESET}"
echo ""
if [ -d "odeint" ]; then
	# Control will enter here if $DIRECTORY exists.
	echo -e "${COLOR_QUES}Do you want to re-install Odeint 2? (Y/N)${COLOR_RESET}"
	read ANSWER_ODEINT
	if [ "$ANSWER_ODEINT" == "Y" ] || [ "$ANSWER_ODEINT" == "y" ]; then
		install_odeint
    fi
else
	echo -e "${COLOR_QUES}Do you want to install Odeint 2? (Y/N)${COLOR_RESET}"
	read ANSWER_ODEINT
	if [ "$ANSWER_ODEINT" == "Y" ] || [ "$ANSWER_ODEINT" == "y" ]; then
		install_odeint
	fi
fi


##---------------------------------------------------------------##
##-------------------- Installing YAML-CPP ----------------------##
##---------------------------------------------------------------##
echo ""
echo -e "${COLOR_BOLD}Installing YAML-CPP ...${COLOR_RESET}"
echo ""
if [ -d "/usr/local/include/yaml-cpp" ]; then
	echo -e "${COLOR_QUES}Do you want to re-install YAML-CPP 0.3.0? (Y/N)${COLOR_RESET}"
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
echo -e "${COLOR_BOLD}Installing Ipopt ...${COLOR_RESET}"
echo ""
if [ -d "ipopt" ]; then
	# Control will enter here if $DIRECTORY exists.
	echo -e "${COLOR_QUES}Do you want to re-install Ipopt 3.12.2? (Y/N)${COLOR_RESET}"
	read ANSWER_IPOPT
	if [ "$ANSWER_IPOPT" == "Y" ] || [ "$ANSWER_IPOPT" == "y" ]; then
		install_ipopt
    fi
else
	echo -e "${COLOR_QUES}Do you want to install Ipopt 3.12.2? (Y/N)${COLOR_RESET}"
	read ANSWER_IPOPT
	if [ "$ANSWER_IPOPT" == "Y" ] || [ "$ANSWER_IPOPT" == "y" ]; then
		install_ipopt
	fi
fi


##---------------------------------------------------------------##
##---------------------- Installing Octomap -----------------------##
##---------------------------------------------------------------##
echo ""
echo -e "${COLOR_BOLD}Installing Octomap ...${COLOR_RESET}"
echo ""
if [ -d "/usr/local/include/octomap" ]; then
	echo -e "${COLOR_QUES}Do you want to re-install Octomap 1.6.8? (Y/N)${COLOR_RESET}"
	read ANSWER_OCTOMAP
	if [ "$ANSWER_OCTOMAP" == "Y" ] || [ "$ANSWER_OCTOMAP" == "y" ]; then
		install_octomap
    fi
else
	echo -e "${COLOR_QUES}Do you want to install Octomap 1.6.8? (Y/N)${COLOR_RESET}"
	read ANSWER_OCTOMAP
	if [ "$ANSWER_OCTOMAP" == "Y" ] || [ "$ANSWER_OCTOMAP" == "y" ]; then
		install_octomap
	fi
fi
