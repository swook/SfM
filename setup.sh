#!/bin/bash

DATA_DIR=data/

# Cleanup
# rm -rf $DATA_DIR

## BEGIN 3rd party libs
LIBDIR=""$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"/lib"
if [ ! -d lib/ ]; then
	mkdir lib/
fi
cd lib/

# Get OpenCV 3
CVDIR=$LIBDIR/opencv
if [ ! -d "$CVDIR" ]; then
	git clone https://github.com/Itseez/opencv.git
	git clone https://github.com/Itseez/opencv_contrib.git
fi
cd "$CVDIR"
git pull origin master
# START OPENCV BUILD
	rm -rf build/
	mkdir build/
	cd build/
	OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules cmake ..
	make -j$(nproc)
	cd ..
# END OPENCV BUILD
cd ..

# Get ceres
CERESDIR=$LIBDIR/ceres-solver
if [ ! -d "$CERESDIR" ]; then
	git clone https://ceres-solver.googlesource.com/ceres-solver
fi
cd "$CERESDIR"
git pull origin master
# START CERES BUILD
	rm -rf build/
	mkdir build/
	cd build/
	cmake -D CMAKE_BUILD_TYPE=RELEASE WITH_TBB=ON ..
	make -j$(nproc)
	cd ..
# END CERES BUILD
cd ..

cd ..
## END 3rd party

# Get datasets
wget -Nnv http://files.swook.net/sfm_data.zip
unzip -oqq sfm_data.zip

