#!/bin/bash

make && LD_LIBRARY_PATH=$(dirname ../lib/**/bin/**/libvl.so) ./main ../data
