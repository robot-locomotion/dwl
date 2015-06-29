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
	# get Eigen 3.2.4
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
	# get RBDL 2.4.0
	wget https://bitbucket.org/rbdl/rbdl/get/default.zip
	unzip default.zip
	mv rbdl-rbdl-cfd302b3b418 rbdl
	cd rbdl
	mkdir -p build
	cd build
	cmake -D RBDL_BUILD_ADDON_URDFREADER:bool=ON ../
	sudo make install
	cd ../../
	rm default.zip
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

	# Getting Ipopt 3.11.8
	wget http://www.coin-or.org/download/source/Ipopt/Ipopt-3.11.8.tgz
	mkdir ipopt && tar xzvf Ipopt-3.11.8.tgz -C ipopt --strip-components 1
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
	rm -rf Ipopt-3.11.8.tgz
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
echo -e "${COLOR_BOLD}install_deps.sh - DWL Installation Script for Ubuntu Precise Pangolin 12.04${COLOR_RESET}"
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
	install_odeint
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
	echo -e "${COLOR_QUES}Do you want to re-install Ipopt 3.11.8? (Y/N)${COLOR_RESET}"
	read ANSWER_IPOPT
	if [ "$ANSWER_IPOPT" == "Y" ] || [ "$ANSWER_IPOPT" == "y" ]; then
		install_ipopt
    fi
else
	install_ipopt
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
	install_octomap
fi
