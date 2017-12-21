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
	make -j
	if [[ $OWNER == 'root' ]]; then
		sudo make -j install
	else
		make -j install
	fi
fi
