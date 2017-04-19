#ifndef DATA_FUSION_MATH_H
#define DATA_FUSION_MATH_H

#define R2D 180.0f/M_PI
#define D2R M_PI/180.0f
#define ONE_G 9.80665f;
#define GET_ARRAY_LEN(array) (sizeof(array) / sizeof(array[0]))

// 设置关于VLOG的打印等级
#define VLOG_FATAL 0
#define VLOG_ERROR 1
#define VLOG_WARNING 2
#define VLOG_INFO 3
#define VLOG_DEBUG 4
#if defined(LOG_STD_COUT)
    #include <iostream>
    #define VLOG(serverity) std::cout
    #define VLOG_IF(verboselevel, condition) std::cout
#endif

// gflog
#include "gflags/gflags.h"
DECLARE_double(gyro_bias_x);
DECLARE_double(gyro_bias_y);
DECLARE_double(gyro_bias_z);

DECLARE_string(imu_init_addr); // imu校正参数文件存储地址
DECLARE_string(imu_parameter_log_addr); // imu每次上电后校准的结果，仅用于分析
DECLARE_string(log_data_addr); // 读取log.txt数据地址
DECLARE_string(jpg_data_addr); // 读取jpg数据文件夹

// 拨杆检测
DECLARE_string(turnlamp_detect_init_addr); // 拨杆检测的初始化配置


// data_fusion
#pragma pack(1)
    struct StructAtt
    {
        double timestamp;
        double att[3];
        double angle_z;
        double att_gyro[3];
        double acc[3];
        double gyro[3];
    };

    struct StructVehicleState
    {
        double timestamp;
        double pos[2];
        double vel[2];
        double yaw;
    };

    struct StructTurnRadius
    {
        double timestamp;
        double R;
        bool is_imu_value_ok;
    };

    struct StructImageFrameInfo
    {
        double timestamp;
        double index;
    };

    struct StructImuData
    {
        double timestamp;
        double acc_raw[3];
        double gyro_raw[3];
        double acc[3];
        double gyro[3];
        double temp;
    };

    struct StructCanSpeedData
    {
        double timestamp;
        double speed;
    };

    struct StructImuParameter
    {
       double gyro_A0[3];
       double gyro_A1[3][3];
       double acc_A0[3];
       double acc_A1[3][3];
    };


    typedef struct {
    	double timestamp;
    	double acc[3];
    }AccData;


#pragma pack()

// function
int LowpassFilter3f(double y_pre[3], const double x_new[3], double dt, const double filt_hz, double y_new[3] );


#endif  // DATA_FUSION_MATH_H
