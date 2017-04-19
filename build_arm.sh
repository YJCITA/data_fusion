#!/usr/bin/env bash

build_tmp="arm/"

compile="Release"
if [ $# -eq 1 ]; then
    compile=$1
fi

mkdir ${build_tmp} 2>/dev/null
cd ${build_tmp}


../android_cmake.sh $compile
make -j4 --makefile Makefile
cd ..

