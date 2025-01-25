#!/bin/bash
export DYLD_LIBRARY_PATH=/usr/local/lib/hakoniwa:$DYLD_LIBRARY_PATH

cd cmake-build
cmake ..
make