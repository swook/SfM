#!/bin/bash

DATA_DIR=data/

# Cleanup
# rm -rf $DATA_DIR

## BEGIN 3rd party libs
# LIBDIR=""$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"/lib"
# if [ ! -d lib/ ]; then
# 	mkdir lib/
# fi
# cd lib/

# # Get ceres
# CERESDIR=$LIBDIR/ceres-solver
# if [ ! -d "$CERESDIR" ]; then
# 	git clone https://ceres-solver.googlesource.com/ceres-solver
# fi
# cd "$CERESDIR"
# git pull origin master
# # START CERES BUILD
# 	rm -rf build/
# 	mkdir build/
# 	cd build/
# 	cmake -D CMAKE_BUILD_TYPE=RELEASE WITH_TBB=ON ..
# 	make -j$(nproc)
# 	cd ..
# # END CERES BUILD
# cd ..

# cd ..
## END 3rd party

# Get datasets
wget -Nnv http://files.swook.net/sfm_data.zip
unzip -oqq sfm_data.zip

