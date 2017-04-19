#ifndef IMU_ATTITUDE_ESTIMATE_H
#define IMU_ATTITUDE_ESTIMATE_H

#include <fstream>
#include <string.h>
#include <sstream>
#include <iostream>
#include <vector>
#include <math.h>
#include <stdio.h>

#include "gflags/gflags.h"
#include "common/base/log_level.h"
#include "common/concurrency/rwlock.h"
#include "datafusion_math.h"

using namespace std;

namespace imu {

class ImuAttitudeEstimate
{
public:

    /// @brief construct  obj
    ImuAttitudeEstimate();

    ~ImuAttitudeEstimate() {}

    void Initialize( );

    /// att_new:新的姿态
    /// acc_data: 加速度数据
    /// gyro_data: 陀螺仪数据
    /// dt: 前后两次量测数据更新的时间差
    void UpdataAttitude( const double acc_data[3], const double gyro_data[3], double dt);

    void GetAttitude(double att[3]);

    void GetAttitudeAngleZ(double att[3], double *angle_z);

    // euler attitude to Rbn(from n to b)
    void CalculateAtt2Rotation(const double att[3], double R[3][3]);

    // 获取由gyro积分得到的姿态角度
    void GetAttitudeGyro(double att_gyro[3]);

    /// 一阶低通函数
    int LowpassFilter3f(double y_pre[3], const double x_new[3], double dt, const double filt_hz, double y_new[3] );

    int AccDataCalibation(const double acc_data_raw[3], double acc_data_ned[3] );

    int SetAccCalibationParam(double A0[3], double A1[3][3]);

    int GyrocDataCalibation(const double gyro_data_raw[3], double gyro_data_new[3] );

    // murata
    int GyrocDataCalibationMurata(const double gyro_data_raw[3], double gyro_data_new[3] );

    int AccDataCalibationMurata(const double acc_data_raw[3], double acc_data_ned[3] );

    void ResetState();

    void GetGyroBias( double gyro_A0[3] );

    void SetGyroBias( const double gyro_A0[3] );

    void ClearGyroBias();

    // 设置imu参数
    int SetImuParameter(const StructImuParameter imu_parameter);
    
    // imu参数清零
    int ResetImuParameter();


private:
    enum
    {
      X_AXIS = 0,
      Y_AXIS,
      Z_AXIS
    };

    double m_factor_acc_gyro[3]; // 加速度计修正的姿态的系数
    double m_att[3];
    double m_gyro_angle[3];
    double m_angle_z;
    int m_att_init_counter;// = 20;

    double m_accel_range_scale;
    double m_acc_A0[3];
    double m_acc_A1[3][3];
    double m_gyro_range_scale;
    double m_gyro_A0[3]; //  // 陀螺仪零偏，在线估计
    
    // 新老摄像头参数选择
    bool m_is_imu_mod_set;
	int m_imu_mode;  // 默认是0，1:旧摄像头 2：新摄像头

    // murata
    double m_accel_range_scale_murata;
    double m_acc_A0_murata[3];
    double m_acc_A1_murata[3][3];
    double m_gyro_range_scale_murata;
    double m_gyro_A0_murata[3]; //  // 陀螺仪零偏，在线估计

    // 读写锁
    RWLock m_rw_lock; 
};

}

#endif  // IMU_ATTITUDE_ESTIMATE_H
