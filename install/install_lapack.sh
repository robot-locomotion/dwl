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
	sudo rm -rf lapack
else
	rm -rf lapack
fi

if [ "$CURRENT_OS" == "OSX" ]; then
	# Getting the LAPACK 3.6.0
	curl -L "http://www.netlib.org/lapack/lapack-3.6.0.tgz" | tar xz
	mv lapack-* lapack
	cd lapack
	mkdir -p build
	cd build
	#TODO this part doesn't work yet
	cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$INSTALL_DEPS_PREFIX -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_LIBDIR=lib $VERBOSITY ../
	make -j
	if [[ $OWNER == 'root' ]]; then
		sudo make -j install
	else
		make -j install
	fi
elif [ "$CURRENT_OS" == "UBUNTU" ]; then
	# Getting the LAPACK 3.6.0
	wget http://www.netlib.org/lapack/lapack-3.6.0.tgz
	mkdir lapack && tar xzvf lapack-3.6.0.tgz -C lapack --strip-components 1
	rm -rf lapack-3.6.0.tgz
	cd lapack
	mkdir -p build
	cd build
	cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$INSTALL_DEPS_PREFIX -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_LIBDIR=lib $VERBOSITY ../
	make -j
	if [[ $OWNER == 'root' ]]; then
		sudo make -j install
	else
		make -j install
	fi
fi
