cmake_minimum_required(VERSION 2.8.2)
cmake_policy(SET CMP0054 OLD) # cmake 3.1以后版本新特性
#FIND_PACKAGE(OpenCV REQUIRED )

AUX_SOURCE_DIRECTORY(. SRC_LIST)
ADD_LIBRARY(VO STATIC ${SRC_LIST})

IF (WIN32)
ELSEIF(ANDROID_ABI)
#    TARGET_LINK_LIBRARIES(data_fusion halio miniglog gflags concurrency)
ELSEIF(UNIX)
   TARGET_LINK_LIBRARIES(VO ${OpenCV_LIBS})
endif()
