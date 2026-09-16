#! /usr/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

PX4_DIR=$PWD
echo "Working directory set to: $PX4_DIR"

NUM_CORES=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
echo "Building with $NUM_CORES CPU cores..."

cd Micro-XRCE-DDS-Agent
# if build exist
if [ -d "build" ]; then
    echo "Micro-XRCE-DDS-Agent build exist"
    exit
else
    cmake -Bbuild -S.
    cmake --build build -j "$NUM_CORES"
    cmake --install build
fi

ldconfig /usr/local/lib/
