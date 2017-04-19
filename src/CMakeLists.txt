cmake_minimum_required(VERSION 2.8.2)
cmake_policy(SET CMP0054 OLD) # cmake 3.1以后版本新特性

AUX_SOURCE_DIRECTORY(. SRC_LIST1)
AUX_SOURCE_DIRECTORY(data_fusion SRC_LIST2)
ADD_LIBRARY(imu STATIC ${SRC_LIST1} ${SRC_LIST2})

IF (WIN32)
ELSEIF(ANDROID_ABI)
    TARGET_LINK_LIBRARIES(imu halio miniglog gflags concurrency)
ELSEIF(UNIX)
    TARGET_LINK_LIBRARIES(imu halio glog gflags concurrency)
endif()

ADD_SUBDIRECTORY(data_fusion/)
# ADD_SUBDIRECTORY(VO/)
