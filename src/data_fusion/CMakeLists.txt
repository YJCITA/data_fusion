cmake_minimum_required(VERSION 3.0)
cmake_policy(SET CMP0054 OLD) # cmake 3.1以后版本新特性

AUX_SOURCE_DIRECTORY(. SRC_LIST)
ADD_LIBRARY(data_fusion STATIC ${SRC_LIST})

IF (WIN32)
ELSEIF(ANDROID_ABI)
    TARGET_LINK_LIBRARIES(data_fusion halio miniglog gflags concurrency)
ELSEIF(UNIX)
    TARGET_LINK_LIBRARIES(data_fusion halio glog gflags concurrency)
endif()

INCLUDE_DIRECTORIES(${PROJECT_ROOT_PATH})
