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
	sudo rm -rf yaml-cpp
else
	rm -rf yaml-cpp
fi

if [ "$CURRENT_OS" == "OSX" ]; then
	echo -e "${COLOR_WARN}Mac OSX installation not support yet${COLOR_RESET}"
	# Getting the YAML-CPP 0.5.2
	curl -L "https://github.com/jbeder/yaml-cpp/archive/release-0.5.2.zip" > release-0.5.2.zip
	unzip release-0.5.2.zip && rm -rf release-0.5.2.zip
	mv yaml-cpp-*/ yaml-cpp
	cd yaml-cpp
	mkdir -p build
	cd build
	cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=$INSTALL_DEPS_PREFIX $VERBOSITY ../
	make -j
	if [[ $OWNER == 'root' ]]; then
		sudo make -j install
	else
		make -j install
	fi
elif [ "$CURRENT_OS" == "UBUNTU" ]; then
	sudo apt-get install -qqy libboost-all-dev

	# Getting the YAML-CPP 0.5.2
	wget https://github.com/jbeder/yaml-cpp/archive/release-0.5.2.zip
	unzip release-0.5.2.zip && rm -rf release-0.5.2.zip
	mv yaml-cpp-*/ yaml-cpp
	cd yaml-cpp
	mkdir -p build
	cd build
	cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=$INSTALL_DEPS_PREFIX $VERBOSITY ../
	make -j
	if [[ $OWNER == 'root' ]]; then
		sudo make -j install
	else
		make -j install
	fi
fi
