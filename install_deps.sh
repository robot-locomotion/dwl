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



##############################################  MAIN  ########################################################
# Printing the information of the shell script
echo -e "${COLOR_BOLD}install_deps.sh - DWL Installation Script for Ubuntu Trusty Tahr 14.04${COLOR_RESET}"
echo ""
echo "Copyright (C) 2015-2018 Carlos Mastalli, <carlos.mastalli@laas.fr>"
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

mkdir -p ${CURRENT_DIR}/thirdparty
cd ${CURRENT_DIR}/thirdparty


# Added doxygen install. TODO moved from here and tested for other OS (i.e. Mac OSX)
sudo apt-get install doxygen
sudo apt-get install python2.7-dev python-numpy
sudo pip install --user numpy


# Installing the dwl dependencies
if [[ $1 == 'default' ]]; then
	INSTALL_DEPS_PREFIX=$HOME/openrobots
	mkdir -p $INSTALL_DEPS_PREFIX

	# Installing all the dwl dependencies
	bash $CURRENT_DIR/install/install_eigen.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
	bash $CURRENT_DIR/install/install_rbdl.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
	bash $CURRENT_DIR/install/install_yamlcpp.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
	bash $CURRENT_DIR/install/install_swig.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
	bash $CURRENT_DIR/install/install_ipopt.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
	bash $CURRENT_DIR/install/install_qpoases.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
	bash $CURRENT_DIR/install/install_libcmaes.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
	bash $CURRENT_DIR/install/install_pyadolc.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
	bash $CURRENT_DIR/install/install_octomap.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
else
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

	# Checking if the installation folder has sudo rights
	INFO=( $(stat -L -c "%a %G %U" $INSTALL_DEPS_PREFIX) )
	PERM=${INFO[0]}
	GROUP=${INFO[1]}
	OWNER=${INFO[2]}

	##---------------------------------------------------------------##
	##---------------------- Installing Eigen -----------------------##
	##---------------------------------------------------------------##
	echo ""
	echo -e "${COLOR_BOLD}Installing Eigen ...${COLOR_RESET}"
	if [ -d "$INSTALL_DEPS_PREFIX/include/eigen3" ] || [ -d "$COMMON_INSTALL_PREFIX/include/eigen3" ]; then
		echo -e -n "${COLOR_QUES}Do you want to re-install Eigen 3.2.10? [y/N]: ${COLOR_RESET}"
		read ANSWER_EIGEN
		if [ "$ANSWER_EIGEN" == "Y" ] || [ "$ANSWER_EIGEN" == "y" ]; then
			bash $CURRENT_DIR/install/install_eigen.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
	    fi
	else
		bash $CURRENT_DIR/install/install_eigen.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
	fi


	##---------------------------------------------------------------##
	##----------------------- Installing RBDL -----------------------##
	##---------------------------------------------------------------##
	echo ""
	echo -e "${COLOR_BOLD}Installing RBDL ...${COLOR_RESET}"
	if [ -d "$INSTALL_DEPS_PREFIX/include/rbdl" ] || [ -d "$COMMON_INSTALL_PREFIX/include/rbdl" ]; then
		echo -e -n "${COLOR_QUES}Do you want to re-install RBDL 2.4.0? [y/N]: ${COLOR_RESET}"
		read ANSWER_RBDL
		if [ "$ANSWER_RBDL" == "Y" ] || [ "$ANSWER_RBDL" == "y" ]; then
			bash $CURRENT_DIR/install/install_rbdl.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
	    fi
	else
		bash $CURRENT_DIR/install/install_rbdl.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
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
			bash $CURRENT_DIR/install/install_yamlcpp.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
		fi
	else
		bash $CURRENT_DIR/install/install_yamlcpp.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
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
			bash $CURRENT_DIR/install/install_swig.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
		fi
	else
		bash $CURRENT_DIR/install/install_swig.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
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
			bash $CURRENT_DIR/install/install_ipopt.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
    	fi
	else
		echo -e -n "${COLOR_QUES}Do you want to install Ipopt 3.12.4? [y/N]: ${COLOR_RESET}"
		read ANSWER_IPOPT
		if [ "$ANSWER_IPOPT" == "Y" ] || [ "$ANSWER_IPOPT" == "y" ]; then
			bash $CURRENT_DIR/install/install_ipopt.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
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
			bash $CURRENT_DIR/install/install_qpoases.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
    	fi
	else
		echo -e -n "${COLOR_QUES}Do you want to install qpOASES 3.2.0? [y/N]: ${COLOR_RESET}"
		read ANSWER_QPOASES
		if [ "$ANSWER_QPOASES" == "Y" ] || [ "$ANSWER_QPOASES" == "y" ]; then
			bash $CURRENT_DIR/install/install_qpoases.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
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
			bash $CURRENT_DIR/install/install_libcmaes.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
	    fi
	else
		echo -e -n "${COLOR_QUES}Do you want to install libcmaes 0.9.5? [y/N]: ${COLOR_RESET}"
		read ANSWER_LIBCMAES
		if [ "$ANSWER_LIBCMAES" == "Y" ] || [ "$ANSWER_LIBCMAES" == "y" ]; then
			bash $CURRENT_DIR/install/install_libcmaes.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
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
		if [ "$ANSWER_PYADOLC" == "Y" ] || [ "$ANSWER_PYADOLC" == "y" ]; then
			bash $CURRENT_DIR/install/install_pyadolc.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
	    fi
	else
		echo -e -n "${COLOR_QUES}Do you want to install pyadolc? [y/N]: ${COLOR_RESET}"
		read ANSWER_PYADOLC
		if [ "$ANSWER_PYADOLC" == "Y" ] || [ "$ANSWER_PYADOLC" == "y" ]; then
			bash $CURRENT_DIR/install/install_pyadolc.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
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
			bash $CURRENT_DIR/install/install_octomap.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
		fi
	else
		echo -e -n "${COLOR_QUES}Do you want to install Octomap 1.6.8? [y/N]: ${COLOR_RESET}"
		read ANSWER_OCTOMAP
		if [ "$ANSWER_OCTOMAP" == "Y" ] || [ "$ANSWER_OCTOMAP" == "y" ]; then
			bash $CURRENT_DIR/install/install_octomap.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
		fi
	fi


	##---------------------------------------------------------------##
	##-------------------- Installing gnuplot ----------------------##
	##---------------------------------------------------------------##
#	echo ""
#	echo -e "${COLOR_BOLD}Installing gnuplot ...${COLOR_RESET}"
#	if [ -x "/usr/local/bin/gnuplot" ]; then
#		echo -e -n "${COLOR_QUES}Do you want to re-install gnuplot 5.0.3? [y/N]: ${COLOR_RESET}"
#		read ANSWER_GNUPLOT
#		if [ "$ANSWER_GNUPLOT" == "Y" ] || [ "$ANSWER_GNUPLOT" == "y" ]; then
#			bash $CURRENT_DIR/install/install_gnuplot.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
#		fi
#	else
#		echo -e -n "${COLOR_QUES}Do you want to install gnuplot 5.0.3? [y/N]: ${COLOR_RESET}"
#		read ANSWER_GNUPLOT
#		if [ "$ANSWER_GNUPLOT" == "Y" ] || [ "$ANSWER_GNUPLOT" == "y" ]; then
#			bash $CURRENT_DIR/install/install_gnuplot.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
#		fi
#	fi
fi
