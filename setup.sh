#!/bin/bash

DATA_DIR=data/

# Cleanup
# rm -rf $DATA_DIR

# Get datasets
wget -Nnv http://files.swook.net/sfm_data.zip
unzip -oqq sfm_data.zip

