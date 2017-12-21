#!/bin/bash


if [[ $# -ne 3 ]]; then
    echo "Please pass the following arguments:"
	echo "    1. The current Operating System (OSX, UBUNTU)"
	echo "    2. The installation directory"
	echo "    3. The verbosity level, True or False"
	exit 1
fi

# Setting the arguments
CURRENT_OS=$1
INSTALL_DEPS_PREFIX=$2
VERBOSITY=""
if [[ $3 == False ]]; then
	VERBOSITY="-DCMAKE_RULE_MESSAGES:BOOL=OFF"
fi
DWL_DIR="$( cd ../ "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
INFO=( $(stat -L -c "%a %G %U" $INSTALL_DEPS_PREFIX) )
OWNER=${INFO[2]}

# Remove old folder (sanity procedure)
cd $DWL_DIR/thirdparty
if [[ $OWNER == 'root' ]]; then
	sudo rm -rf eigen
else
	rm -rf eigen
fi

if [ "$CURRENT_OS" == "OSX" ]; then
	# Getting Eigen 3.2.10
	curl -L "http://www.bitbucket.org/eigen/eigen/get/3.2.10.tar.bz2" | tar xj
	mv eigen-eigen-*/ eigen
	cd eigen
	mkdir -p build
	cd build
	cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$INSTALL_DEPS_PREFIX -DPKGCONFIG_INSTALL_DIR=$INSTALL_DEPS_PREFIX/lib/pkgconfig $VERBOSITY ..
	make -j
	if [[ $OWNER == 'root' ]]; then
		sudo make -j install
	else
		make -j install
	fi
elif [ "$CURRENT_OS" == "UBUNTU" ]; then
	# Getting Eigen 3.2.10
	wget http://www.bitbucket.org/eigen/eigen/get/3.2.10.tar.bz2
	mkdir eigen && tar jxf 3.2.10.tar.bz2 -C eigen --strip-components 1
	rm -rf 3.2.10.tar.bz2
	cd eigen
	mkdir -p build
	cd build
	cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$INSTALL_DEPS_PREFIX -DPKGCONFIG_INSTALL_DIR=$INSTALL_DEPS_PREFIX/lib/pkgconfig $VERBOSITY ..
	make -j
	if [[ $OWNER == 'root' ]]; then
		sudo make -j install
	else
		make -j install
	fi
fi
