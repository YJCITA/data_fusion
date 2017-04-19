#!/usr/bin/env bash

build_tmp="build/"

compile="Release"
if [ $# -eq 1 ]; then
    compile=$1
fi

mkdir ${build_tmp} 2>/dev/null
cd ${build_tmp}



cmake --clean-first -DCMAKE_BUILD_TYPE=$compile ..
make -j4 --makefile Makefile

