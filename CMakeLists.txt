cmake_minimum_required(VERSION 2.8.2)
PROJECT(DATA_FUSION) # PROJECT_ROOT_PATH
cmake_policy(SET CMP0054 OLD) # cmake 3.1以后版本新特性

IF (ANDROID_ABI)
    SET(CMAKE_CXX_FLAGS_RELEASE "-Wall -Wno-unused-result -Wno-unused-but-set-variable -Wno-unused-variable -O3 -std=gnu++0x")
    SET(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wno-unused-result -Wno-unused-but-set-variable -Wno-unused-variable -g -std=gnu++0x")
ELSEIF(UNIX)
    SET(CMAKE_CXX_FLAGS_RELEASE "-Wall -Wno-unused-result -Wno-unused-but-set-variable -Wno-unused-variable -O3 -std=gnu++0x")
    SET(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wno-unused-result -Wno-unused-but-set-variable -Wno-unused-variable -g -std=gnu++0x")
ENDIF ()

SET(PROJECT_ROOT_PATH "${CMAKE_SOURCE_DIR}")
MESSAGE(STATUS ${PROJECT_ROOT_PATH})
SET(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")
SET(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/lib/")

######################### configuration ###########################
OPTION(USE_GSENSOR_LOG_SWITCH "switch for using chip-gsensor-data instantly OR from gsensor-log" OFF)
IF (USE_GSENSOR_LOG_SWITCH)
    MESSAGE(STATUS "use gsensor data from log")
    ADD_DEFINITIONS(-DDATA_FROM_LOG) # read data from log
ELSE()
    MESSAGE(STATUS "use gsensor data from chip")
ENDIF()

OPTION(COMPILE_DEBUG_SWITCH "switch for copmile debug OR release" OFF)
IF (COMPILE_DEBUG_SWITCH)
    MESSAGE(STATUS "copmile debug")
    SET(CMAKE_BUILD_TYPE "Debug")
    SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
ELSE()
    MESSAGE(STATUS "copmile release")
ENDIF()

OPTION(SAVE_ALL_IMU_CALIBRATE_RESULTS "switch for save all the imu calibrate results OR not" OFF)
IF (SAVE_ALL_IMU_CALIBRATE_RESULTS)
    MESSAGE(STATUS "save imu calibrate results")
    ADD_DEFINITIONS(-DSAVE_ALL_IMU_CALIBRATE_RESULTS) # when the borard power on, imu will do calibrate, save all the results
ENDIF()

OPTION(PRINT_LOG_INFO_SWITCH "switch for print all the log data for debug" OFF)
IF (PRINT_LOG_INFO_SWITCH)
    MESSAGE(STATUS "print all the log data for debug")
    ADD_DEFINITIONS(-DLOG_STD_COUT) 
ENDIF()

# 是否开启摄像头选择，默认时开启的
OPTION(IMU_MODE_AUTO_SELECT_SWITCH "switch for old or new camera" ON)
IF (IMU_MODE_AUTO_SELECT_SWITCH)
    MESSAGE(STATUS "auto to select the imu mode: new or old camera!!")
    ADD_DEFINITIONS(-DIMU_MODE_AUTO_SELECT) 
ELSE()
	ADD_DEFINITIONS(-DIMU_MODE_SET_2)
	MESSAGE(STATUS "set the imu mode: new camera!!")
ENDIF()

ADD_DEFINITIONS(-DUSE_GLOG)

######################### packet ###########################
#LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_ROOT_PATH}/cmake) # 这个时ORB_SLAM2中的用法
#find_package(Eigen3 3.1.0 REQUIRED)
include(cmake/script_eigen.cmake REQUIRED)  # 添加Eigen3：linux和android都可以用
MESSAGE(STATUS "EIGEN3_INCLUDE_DIR = ${EIGEN3_INCLUDE_DIR}")

#root
IF ("${CMAKE_SIZEOF_VOID_P}" EQUAL "4")
    MESSAGE(STATUS "root: x32 platform")
    SET(PLATFORM x86)
ELSEIF ("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
    MESSAGE(STATUS "root: x64 platform")
    SET(PLATFORM x64)
ENDIF ()

IF (MSVC)
ELSEIF (ANDROID_ABI)
    MESSAGE(STATUS "DEFINE: ANDORID platform")
    ADD_DEFINITIONS(-DANDROID)
    ADD_CUSTOM_TARGET(MINIEYE_VERSION COMMAND ${SCRIPTS_PATH}/version.sh |tee ${PROJECT_ROOT_PATH}/version.h.in)
ELSEIF (UNIX)
    ADD_DEFINITIONS(-DDATA_FROM_LOG)
    MESSAGE(STATUS "DEFINE: UNIX platform")
    OPTION(SET_OPENCV_LIB_ADDR "switch for change opencv library address" OFF)
    IF (SET_OPENCV_LIB_ADDR)
        SET(OpenCV_DIR "/home/yj/bak/packet/opencv/opencv-3.1-install/share/OpenCV")
        MESSAGE(STATUS "set the new opencv library address: ${OpenCV_DIR}")
    ELSE()
        SET(OpenCV_DIR "/opt/opencv/default/share/OpenCV")
        MESSAGE(STATUS "OpenCV_DIR: ${OpenCV_DIR}")
    ENDIF()

    FIND_PACKAGE(OpenCV REQUIRED)
ENDIF ()

IF (WIN32)
ELSEIF (ANDROID_ABI)
    MESSAGE(STATUS "include: ANDORID platform")
    #include directory
    INCLUDE_DIRECTORIES(
            "${PROJECT_ROOT_PATH}"
            "${PROJECT_ROOT_PATH}/thirdparty/opencv/android/sdk/native/jni/include/"
            "${PROJECT_ROOT_PATH}/thirdparty/libusb-1.0/android_armv8/"
            "${PROJECT_ROOT_PATH}/thirdparty/libblis/armv8/include/blis"
            "${PROJECT_ROOT_PATH}/thirdparty/gflags/android/include/"
            "${PROJECT_ROOT_PATH}/thirdparty/miniglog/"
            "${PROJECT_ROOT_PATH}/thirdparty/rapidjson-1.1.0/include/"
            "${PROJECT_ROOT_PATH}/src/"
            "${PROJECT_ROOT_PATH}/src/data_fusion/"
    )
    LINK_DIRECTORIES(
            "${PROJECT_ROOT_PATH}/thirdparty/opencv/android/sdk/native/libs/arm64-v8a/"
            "${PROJECT_ROOT_PATH}/thirdparty/opencv/android/sdk/native/3rdparty/libs/arm64-v8a/"
            "${PROJECT_ROOT_PATH}/thirdparty/gflags/android/lib/${PLATFORM}/"
            "${PROJECT_ROOT_PATH}/thirdparty/libblis/armv8/lib"
            "${PROJECT_ROOT_PATH}/thirdparty/libusb-1.0/android_armv8/"
    )
    SET(ARM_LIBS relative_locate base time concurrency opencv_core opencv_imgproc opencv_hal opencv_highgui opencv_video opencv_videoio opencv_imgcodecs libpng libtiff libjpeg libjasper libwebp IlmImf c z log miniglog blis)
    SET(OpenCV_LIBS opencv_core opencv_imgproc opencv_hal opencv_highgui opencv_video opencv_videoio opencv_imgcodecs libpng libtiff libjpeg libjasper libwebp IlmImf c z log miniglog blis)
ELSEIF (UNIX)
    MESSAGE(STATUS "include: UNIX platform")
    INCLUDE_DIRECTORIES(
            "${PROJECT_ROOT_PATH}/thirdparty/gtest/linux/include/"
            "${OpenCV_INSTALL_PATH}/include"
            "${PROJECT_ROOT_PATH}/thirdparty/gflags/linux/include/"
            "${PROJECT_ROOT_PATH}/thirdparty/glog/linux/include/"
            "${PROJECT_ROOT_PATH}/thirdparty/rapidjson-1.1.0/include/"
            "${PROJECT_ROOT_PATH}/src/"
            "${PROJECT_ROOT_PATH}/src/data_fusion/"
            "${PROJECT_ROOT_PATH}/src/VO/"
            "${PROJECT_ROOT_PATH}"
    )
    LINK_DIRECTORIES(
            "${PROJECT_ROOT_PATH}/thirdparty/gtest/linux/static/x64/"
            "${OpenCV_INSTALL_PATH}/lib"
            "${PROJECT_ROOT_PATH}/thirdparty/gflags/linux/lib/x64/"
            "${PROJECT_ROOT_PATH}/thirdparty/glog/linux/static/x64/"
            "${PROJECT_ROOT_PATH}/thirdparty/gtest/linux/static/x64/"
    )
ENDIF ()

ADD_SUBDIRECTORY(common/time)
ADD_SUBDIRECTORY(common/base)
ADD_SUBDIRECTORY(common/system)
ADD_SUBDIRECTORY(common/string)
ADD_SUBDIRECTORY(common/relative_locate)
ADD_SUBDIRECTORY(common/concurrency)
ADD_SUBDIRECTORY(common/hal)
ADD_SUBDIRECTORY(common/memory)
IF (ANDROID_ABI)
    ADD_SUBDIRECTORY(thirdparty/miniglog)
ENDIF ()
#ADD_SUBDIRECTORY(common/tests)
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)
