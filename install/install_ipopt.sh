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
	sudo rm -rf ipopt
else
	rm -rf ipopt
fi

if [ "$CURRENT_OS" == "OSX" ]; then
	echo -e "${COLOR_WARN}Mac OSX installation not support yet${COLOR_RESET}"
	# Getting Ipopt 3.12.4
	curl -L "http://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.4.tgz" | tar xz
	mv Ipopt-* ipopt
	# Documentation for Ipopt Third Party modules:
	# http://www.coin-or.org/Ipopt/documentation/node13.html
	
	# Installing LAPACK and BLAS
	if [ ! -f "$INSTALL_DEPS_PREFIX/lib/liblapack.so" ]; then
		bash $DWL_DIR/install/install_lapack.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
	fi

	cd $DWL_DIR/thirdparty
	cd ipopt/ThirdParty
	# Getting Metis dependency
	cd Metis
#	sed -i 's/metis\/metis/metis\/OLD\/metis/g' get.Metis
#	sed -i 's/metis-4\.0/metis-4\.0\.3/g' get.Metis
#	sed -i 's/mv metis/#mv metis/g' get.Metis
	./get.Metis
	# Patching is necessary. See http://www.math-linux.com/mathematics/Linear-Systems/How-to-patch-metis-4-0-error
	wget http://www.math-linux.com/IMG/patch/metis-4.0.patch
	#curl -L "http://www.math-linux.com/IMG/patch/metis-4.0.patch" -O metis-4.0.patch
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
	sed -i '.original' 's/^rm/# rm/g' get.ASL
	sed -i '.original' 's/^tar /# tar/g' get.ASL
	sed -i '.original' 's/^$wgetcmd/# $wgetcmd/g' get.ASL
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
	../configure --enable-static --prefix $INSTALL_DEPS_PREFIX --with-lapack="-L$INSTALL_DEPS_PREFIX/lib -llapack" --with-blas="-L$INSTALL_DEPS_PREFIX/lib -lblas"
	make -j
	if [[ $OWNER == 'root' ]]; then
		sudo make -j install
	else
		make -j install
	fi
elif [ "$CURRENT_OS" == "UBUNTU" ]; then
	# Installing necessary packages
	sudo apt-get install f2c libf2c2-dev libf2c2 gfortran
	# Getting Ipopt 3.12.4
	wget http://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.4.tgz
	mkdir ipopt && tar xzvf Ipopt-3.12.4.tgz -C ipopt --strip-components 1
	rm -rf Ipopt-3.12.4.tgz
	# Documentation for Ipopt Third Party modules:
	# http://www.coin-or.org/Ipopt/documentation/node13.html
	# Installing LAPACK and BLAS
	if [ ! -f "$INSTALL_DEPS_PREFIX/lib/liblapack.so" ]; then
		bash $DWL_DIR/install/install_lapack.sh $CURRENT_OS $INSTALL_DEPS_PREFIX False
	fi

	cd $DWL_DIR/thirdparty
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
	../configure --enable-static --prefix $INSTALL_DEPS_PREFIX --with-lapack="-L$INSTALL_DEPS_PREFIX/lib -llapack" --with-blas="-L$INSTALL_DEPS_PREFIX/lib -lblas"
	make -j
	if [[ $OWNER == 'root' ]]; then
		sudo make -j install
	else
		make -j install
	fi
fi
