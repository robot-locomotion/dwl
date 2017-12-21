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
	sudo rm -rf qpOASES
else
	rm -rf qpOASES
fi

if [ "$CURRENT_OS" == "OSX" ]; then
	echo -e "${COLOR_WARN}Mac OSX installation not support yet${COLOR_RESET}"
	# Getting the qpOASES 3.2.0
	wget http://www.coin-or.org/download/source/qpOASES/qpOASES-3.2.0.tgz
	tar xzfv qpOASES-3.2.0.tgz && rm qpOASES-3.2.0.tgz
	mv qpOASES-3.2.0 qpOASES
		
	# Installing LAPACK and BLAS
	if [ ! -f "$INSTALL_DEPS_PREFIX/lib/liblapack.so" ]; then
		bash $DWL_DIR/install/install_lapack.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
	fi

	cd $DWL_DIR/thirdparty
	cd qpOASES
	if [[ $OWNER == 'root' ]]; then
		sudo make -j BINDIR=$INSTALL_DEPS_PREFIX/lib REPLACE_LINALG=0 LIB_LAPACK=$INSTALL_DEPS_PREFIX/lib/liblapack.so LIB_BLAS=$INSTALL_DEPS_PREFIX/lib/libblas.so
	else
		make -j BINDIR=$INSTALL_DEPS_PREFIX/lib REPLACE_LINALG=0 LIB_LAPACK=$INSTALL_DEPS_PREFIX/lib/liblapack.so LIB_BLAS=$INSTALL_DEPS_PREFIX/lib/libblas.so
	fi
	mkdir build
	cd build
	cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_DEPS_PREFIX $VERBOSITY ../
	make -j4
	if [[ $OWNER == 'root' ]]; then
		sudo make -j install
	else
		make -j install
	fi
elif [ "$CURRENT_OS" == "UBUNTU" ]; then
	# Getting the qpOASES 3.2.0
	wget http://www.coin-or.org/download/source/qpOASES/qpOASES-3.2.0.tgz
	tar xzfv qpOASES-3.2.0.tgz && rm qpOASES-3.2.0.tgz
	mv qpOASES-3.2.0 qpOASES

	# Installing LAPACK and BLAS
	if [ ! -f "$INSTALL_DEPS_PREFIX/lib/liblapack.so" ]; then
		bash $DWL_DIR/install/install_lapack.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
	fi

	cd $DWL_DIR/thirdparty
	cd qpOASES
	if [[ $OWNER == 'root' ]]; then
		sudo make -j BINDIR=$INSTALL_DEPS_PREFIX/lib REPLACE_LINALG=0 LIB_LAPACK=$INSTALL_DEPS_PREFIX/lib/liblapack.so LIB_BLAS=$INSTALL_DEPS_PREFIX/lib/libblas.so
	else
		make -j BINDIR=$INSTALL_DEPS_PREFIX/lib REPLACE_LINALG=0 LIB_LAPACK=$INSTALL_DEPS_PREFIX/lib/liblapack.so LIB_BLAS=$INSTALL_DEPS_PREFIX/lib/libblas.so
	fi
	mkdir build
	cd build
	cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_DEPS_PREFIX $VERBOSITY ../
	make -j4
	if [[ $OWNER == 'root' ]]; then
		sudo make -j install
	else
		make -j install
	fi
fi
