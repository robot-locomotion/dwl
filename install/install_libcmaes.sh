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
sudo rm -rf libcmaes
if [[ $OWNER == 'root' ]]; then
	sudo rm -rf libcmaes
else
	rm -rf libcmaes
fi

# In case that eigen is installed through install_eigen.sh
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:${INSTALL_DEPS_PREFIX}/lib/pkgconfig

if [ "$CURRENT_OS" == "OSX" ]; then
	echo -e "${COLOR_WARN}Mac OSX installation not support yet${COLOR_RESET}"

	# Compiling and installing Google unit test framework
	curl -L https://github.com/google/googletest/archive/release-1.8.0.tar.gz | tar xz
	mv googletest-release-1.8.0/ gtest
	cd gtest
	mkdir -p build
	cd build
	cmake -D BUILD_SHARED_LIBS:bool=ON CMAKE_BUILD_TYPE=Release $VERBOSITY ../
	make -j
	if [[ $OWNER == 'root' ]]; then
		sudo make -j install
	else
		make -j install
	fi
	cd $CURRENT_DIR/thirdparty

	# Getting the libcmaes 0.9.5
	wget https://github.com/beniz/libcmaes/archive/0.9.5.tar.gz
	mkdir libcmaes && tar zxf 0.9.5.tar.gz -C libcmaes --strip-components 1
	rm -rf 0.9.5.tar.gz
	cd libcmaes
	./autogen.sh
	./configure --enable-gglog --prefix=$INSTALL_DEPS_PREFIX #--with-eigen3-include=$INSTALL_DEPS_PREFIX/include/eigen3
	make -j4
	if [[ $OWNER == 'root' ]]; then
		sudo make -j install
	else
		make -j install
	fi
elif [ "$CURRENT_OS" == "UBUNTU" ]; then
	# Installing libcmaes dependecies
	sudo apt-get install autoconf automake libtool libgoogle-glog-dev libgflags-dev

	# Compiling and installing Google unit test framework
	wget https://github.com/google/googletest/archive/release-1.8.0.tar.gz
	mkdir gtest && tar zxf release-1.8.0.tar.gz -C gtest --strip-components 1
	rm release-1.8.0.tar.gz
	cd gtest
	mkdir -p build
	cd build
	cmake -D BUILD_SHARED_LIBS:bool=ON CMAKE_BUILD_TYPE=Release $VERBOSITY ../
	make -j
	if [[ $OWNER == 'root' ]]; then
		sudo make -j install
	else
		make -j install
	fi

	# Getting the libcmaes 0.9.5
	cd $DWL_DIR/thirdparty
	wget https://github.com/beniz/libcmaes/archive/0.9.5.tar.gz
	mkdir libcmaes && tar zxf 0.9.5.tar.gz -C libcmaes --strip-components 1
	rm -rf 0.9.5.tar.gz
	cd libcmaes
	./autogen.sh
	./configure --enable-gglog --prefix=$INSTALL_DEPS_PREFIX #--with-eigen3-include=$INSTALL_DEPS_PREFIX/include/eigen3
	make -j4
	if [[ $OWNER == 'root' ]]; then
		sudo make -j install
	else
		make -j install
	fi
fi
