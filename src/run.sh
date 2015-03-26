#!/bin/bash

# Compile
make

OS=$(uname -s)

# Run
if [ "$OS" = "Darwin" ]; then
	DYLD_LIBRARY_PATH=$(dirname ../lib/**/bin/**/libvl.dyld) ./main ../data
elif [ "$OS" = "Linux" ]; then
	LD_LIBRARY_PATH=$(dirname ../lib/**/bin/**/libvl.so) ./main ../data
fi
