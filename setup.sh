#!/bin/bash

DATA_DIR=data/

# Cleanup
# rm -rf $DATA_DIR

## BEGIN 3rd party libs
if [ ! -d lib/ ]; then
	mkdir lib/
fi
cd lib/

# Get vlfeat
if [ ! -d vlfeat/ ]; then
	git clone https://github.com/vlfeat/vlfeat.git
fi
cd vlfeat
git pull origin master
cd vlfeat
make
cd ../..

cd ..
## END 3rd party

# Get datasets
wget -Nnv http://files.swook.net/sfm_data.zip
unzip -oqq sfm_data.zip

