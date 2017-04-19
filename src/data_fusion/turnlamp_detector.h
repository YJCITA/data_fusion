#ifndef TURNLAMPDETECTOR_H
#define TURNLAMPDETECTOR_H

#include <fstream>
#include <string>
#include <sstream>
#include <math.h>
#include <vector>
#include <unistd.h>
#include <sys/time.h>


#include "common/concurrency/work_thread.h"
#include "common/concurrency/rwlock.h"
#include "common/hal/halio.h"

#include "imu_attitude_estimate.h"
#include "can_vehicle_estimate.h"
#include "datafusion_math.h"

namespace imu {

#define ROD_ACC_QUEUE_SIZE 100
#define ROD_ACC_THRESHOLD_SIZE 30


class TurnlampDetector: public SingletonBase<TurnlampDetector>
{
    public:
        friend class SingletonBase<TurnlampDetector>;
        
        #pragma pack(1)
        typedef struct {
        	unsigned short rd_index;
        	unsigned short wr_index;
        	AccData	acc_data[ROD_ACC_QUEUE_SIZE];
        }AccDataQueue;
        #pragma pack()

        TurnlampDetector();
        virtual ~TurnlampDetector();

        void Init( );

        int ReadFmuDataFromLog( );

        // 两个IMU相互之间坐标转换矩阵
        void CalculateRotationFmu2Camera(const double acc_fmu[3], const double acc_camera[3]);

        // array_new = R*array_origin;
        void Array3Rotation(const double R[3][3], const double array_origin[3], double array_new[3]);

        // 仅利用拨杆上IMU检测拨杆是否可能被拨动
        // 1: 可能被拨动
        // 0: 没拨动
        int DetectRodShift();

        void StartTurnlampDetectTaskOnline();

        // 自身检测拨杆是否可能是被拨动了
        void DetectSelfRodShiftOnline();

        // 读取log日志，进行检测
        void RunDetectTurnlampOffline();

        int ReadRodDataOnline( );

        // -------------拨杆初始化------------
        // 收集rod acc数据
        int CollectInitRodAccData();

        // 用于外部调用接口，开启初始对准
        void SartRotationInitDataCollect();

        // 重置初始化对准的状态
        void ResetInitMatchState();

        // 读取初始化配置文件
        int ReadRodInitParameter();

        // 计算自检测的阈值
        int CalculateRodSelfDetectThreshold( );

        void StartCalculateRodAccThreshold( );

        void GetRodSelfDetectThreshold(double threshold_t[3], double weight_t[3]);

        // 设置自检测阈值
        void SetRodSelfDetectThreshold(const double threshold_t[3], const double weight_t[3]);

        // 读取 m_R_rod2camera
        void GetRod2CameraRotation(double R[3][3] );

        // 设置 m_R_rod2camera
        void SetRod2CameraRotation(const double R[3][3] );

        // 获取拨杆的数据
        void GetRodAccData( StructImuData *rod_acc_data);

    public:
        // 线程
        WorkThread m_turnlamp_detect_thread;
        bool m_is_turnlamp_detect_running;

        // 读入log
        string buffer_log;
        stringstream ss_log;
        stringstream ss_tmp;
        ifstream infile_log;

        ImuAttitudeEstimate m_rod_attitude_estimate;
        bool m_is_first_read_rod_acc;
        double m_rod_sample_hz;  // fmu采样频率
        double m_R_rod2camera[3][3];
        StructImuData m_rod_imu_data;
        StructImuData m_rod_imu_data_pre;
        
        // 低通滤波
        bool m_is_first_rod_acc_filter; // 是否是第一次进行低通滤波
        double m_rod_acc_pre[3];
        double m_acc_data_timestamp_pre; 
        
        int m_turnlamp_state; //  转向灯信号 -1: right 1: left
        int m_turnlamp_state_pre; // 上一时刻的数据 转向灯信号 -1: right 1: left
        bool m_turnlamp_state_change_CAN; // CAN 数据上显示turnlamp状态变化

        double m_accel_range_scale;
        double m_acc_A0[3];
        double m_acc_A1[3][3];

        // 数据读写锁
        RWLock m_rod_acc_rw_lock;
        
        // 计算rod自检测的阈值
        double m_d_rod_acc[3]; // 拨杆前后两帧的差值
        bool m_is_rod_self_detect_threshold_start; // 是否开始计算拨杆m_d_acc_threshold的计算
        int m_rod_self_detect_threshold_counter; // 拨杆m_d_acc_threshold的数据计数
        AccData m_rod_detect_acc_data[ROD_ACC_THRESHOLD_SIZE];
       
        int m_rod_shift_state; // 拨杆可能被拨动的标志位
        bool m_is_first_detect_rod_shift; // 第一次进入拨杆拨动判断
        double m_rod_self_detect_threshold[3]; // 检测拨杆拨动的acc前后变化阈值
        double m_rod_self_detect_threshold_weight[3]; // 对应个阈值的权值
        double m_d_gyro_threshold[3]; // 检测拨杆拨动的gyro前后变化阈值

        // 拨杆imu和摄像头imu初始对准
        bool m_is_init_rod_acc_collect_start; // 是否开始拨杆imu和摄像头坐标对准的关系
        bool m_is_init_road_acc_collect_ok; // rod acc collect ok or not
        AccDataQueue m_rod_acc_queue;
        AccDataQueue m_camera_acc_queue;
        double m_mean_init_rod_acc[3];
};


}

#endif // TURNLAMPDETECTOR_H

