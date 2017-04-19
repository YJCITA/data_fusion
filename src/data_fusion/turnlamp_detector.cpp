#include "turnlamp_detector.h"

namespace imu {

TurnlampDetector::TurnlampDetector()
{
    Init( );
}

TurnlampDetector::~TurnlampDetector()
{
    #if defined(DATA_FROM_LOG)
        infile_log.close();
    #endif
    m_is_turnlamp_detect_running = false;
    m_turnlamp_detect_thread.StopAndWaitForExit();
}

void TurnlampDetector::Init( )
{
    m_is_first_read_rod_acc = true;
    m_rod_sample_hz = 10.0/400.0; // 从飞控板子上读取的数据的采样频率

    memset(&m_rod_imu_data, 0, sizeof(m_rod_imu_data));
    memset(&m_rod_imu_data_pre, 0, sizeof(m_rod_imu_data_pre));
    m_is_first_detect_rod_shift = true;
    m_rod_shift_state = 0;
    
    //低通滤波
    m_is_first_rod_acc_filter = true; // 是否是第一次进行低通滤波
    memset(m_rod_acc_pre, 0, sizeof(m_rod_acc_pre));
    m_acc_data_timestamp_pre = 0.0;

    // 计算自检测阈值
    m_rod_self_detect_threshold[0] = 5.0;
    m_rod_self_detect_threshold[1] = 5.0;
    m_rod_self_detect_threshold[2] = 9.8;

    m_rod_self_detect_threshold_weight[0] = 1;
    m_rod_self_detect_threshold_weight[1] = 1;
    m_rod_self_detect_threshold_weight[2] = 1;
    
    memset(m_d_rod_acc, 0, sizeof(m_d_rod_acc));
    m_is_rod_self_detect_threshold_start = false;
    m_rod_self_detect_threshold_counter = 0;
    memset(m_rod_detect_acc_data, 0, sizeof(m_rod_detect_acc_data));

    m_d_gyro_threshold[0] = 2.0;
    m_d_gyro_threshold[1] = 1.0;
    m_d_gyro_threshold[2] = 0.0;

    // imu
    m_accel_range_scale = 8.0f/32768;

    m_acc_A0[0] = 0;
    m_acc_A0[1] = 0;
    m_acc_A0[2] = 0;

    m_acc_A1[0][0] = 1;
    m_acc_A1[0][1] = 0;
    m_acc_A1[0][2] = 0;
    m_acc_A1[1][0] = 0;
    m_acc_A1[1][1] = 1;
    m_acc_A1[1][2] = 0;
    m_acc_A1[2][0] = 0;
    m_acc_A1[2][1] = 0;
    m_acc_A1[2][2] = 1;

    memset(m_R_rod2camera, 0 , sizeof(m_R_rod2camera));
    m_R_rod2camera[0][0] = 1.0;
    m_R_rod2camera[1][1] = 1.0;
    m_R_rod2camera[2][2] = 1.0;

    m_is_turnlamp_detect_running = false;

    m_turnlamp_state = 0;
    m_turnlamp_state_pre = 0;
    m_turnlamp_state_change_CAN = false;

    // 对准
    m_is_init_rod_acc_collect_start = false; // 是否开始对准
    m_is_init_road_acc_collect_ok = false;
    memset(&m_rod_acc_queue, 0, sizeof(m_rod_acc_queue));
    memset(&m_camera_acc_queue, 0, sizeof(m_camera_acc_queue));
    memset(m_mean_init_rod_acc, 0, sizeof(m_mean_init_rod_acc));
    ReadRodInitParameter();
        
    
    #if defined(DATA_FROM_LOG)
        // read data from log
        infile_log.open("./data/doing/log.txt"); // ifstream
        LOG(ERROR) << "try open " << FLAGS_log_data_addr;
        if(!infile_log)
            LOG(ERROR) << "open " << FLAGS_log_data_addr << " ERROR!!";
        else
            LOG(ERROR) << "open " << FLAGS_log_data_addr << " OK!";
    #endif

}

// 开始线程
void TurnlampDetector::StartTurnlampDetectTaskOnline()
{
    m_turnlamp_detect_thread.Start();
    m_is_turnlamp_detect_running = true;
    Closure<void>* cls = NewClosure(this, &TurnlampDetector::DetectSelfRodShiftOnline);
    m_turnlamp_detect_thread.AddTask(cls);
}

// 读取log日志，进行检测
void TurnlampDetector::RunDetectTurnlampOffline()
{
    bool is_read_new_fmu = false;
    while(!is_read_new_fmu){
        int fmu_data_update = ReadFmuDataFromLog();
        if(fmu_data_update){
            VLOG(VLOG_INFO)<<"RunDetectTurnlamp--"<<"new fmu data"<<endl;
            m_rod_shift_state = DetectRodShift();
            is_read_new_fmu = true;
        }
    }
}


// 自身检测拨杆是否可能是被拨动了
void TurnlampDetector::DetectSelfRodShiftOnline()
{
    while (m_is_turnlamp_detect_running) {
        int fmu_data_update = ReadRodDataOnline(); // 直到读到rod的acc数据
        if(fmu_data_update){
            if(m_is_init_rod_acc_collect_start){
                // 进行match初始化
                int state_t = CollectInitRodAccData(); // 数据读取完成会清空状态
            }else{
                m_rod_shift_state = DetectRodShift();

                // 计算阈值
                if(m_is_rod_self_detect_threshold_start && m_rod_shift_state)
                    CalculateRodSelfDetectThreshold();
            }
                
            VLOG(VLOG_INFO)<<"RunDetectTurnlamp--"<<"new fmu data"<<endl;
            usleep(10000);  // 控制整个while循环在100hz左右
        }else{
            usleep(1000); // 等待1ms，再次读取数据
        }
    }
}


// 仅利用拨杆上IMU检测拨杆是否可能被拨动
// 1: 可能被拨动
// 0: 没拨动
int TurnlampDetector::DetectRodShift()
{
    if(m_is_first_detect_rod_shift){
        memcpy(&m_rod_imu_data_pre, &m_rod_imu_data, sizeof(m_rod_imu_data_pre));
        m_is_first_detect_rod_shift = false;
    }else{
        for(int i=0; i<3; i++){
            m_d_rod_acc[i] = m_rod_imu_data.acc[i] - m_rod_imu_data_pre.acc[i];
        }  
        memcpy(&m_rod_imu_data_pre, &m_rod_imu_data, sizeof(m_rod_imu_data_pre));

        // 判断是否可能被拨动
        if(fabs(m_d_rod_acc[0])*m_rod_self_detect_threshold_weight[0]>=m_rod_self_detect_threshold[0] || fabs(m_d_rod_acc[1])*m_rod_self_detect_threshold_weight[1]>=m_rod_self_detect_threshold[1]
            || fabs(m_d_rod_acc[2])*m_rod_self_detect_threshold_weight[2]>=m_rod_self_detect_threshold[2]){
            VLOG(VLOG_INFO)<<"DetectRodShift--"<<"rod_d_acc = "<<m_d_rod_acc[0]<<" "<<m_d_rod_acc[1]<<" "<<m_d_rod_acc[2]<<" "<<endl;
//             printf("Detect Rod Shift: m_d_rod_acc= %f %f %f\n",m_d_rod_acc[0], m_d_rod_acc[1], m_d_rod_acc[2]);
            return 1;
        }
    }
    return 0;
}

// 0: 没有读到新数据
// 1: fmu数据更新
int TurnlampDetector::ReadFmuDataFromLog( )
{
    double log_data[2], timestamp_raw[2];
    string data_flag;
    struct StructImuData imu_data;
    bool is_fmu_data_update = false;

    getline(infile_log, buffer_log);
    ss_tmp.clear();
    ss_tmp.str(buffer_log);
    ss_tmp>>log_data[0]>>log_data[1]>>data_flag;
    ss_log.clear();
    ss_log.str(buffer_log);

    //  拨杆imu的数据
    if(data_flag == "FMU"){
        double acc_data_raw[3], acc_data_ned[3], acc_data_filter[3];
        static double acc_data_filter_pre[3]; // 保存fliter变量
        double gyro_data_raw[3], gyro_data_ned[3], gyro_data_filter[3];
        static double gyro_data_filter_pre[3];
        double fmu_temperature, fmu_timestamp;
        string fmu_flag, tmp_str[5];

        ss_log>>timestamp_raw[0]>>timestamp_raw[1]>>fmu_flag>>tmp_str[0]>>tmp_str[1]>>tmp_str[2]>>tmp_str[3]
              >>acc_data_raw[0]>>acc_data_raw[1]>>acc_data_raw[2]
              >>gyro_data_raw[0]>>gyro_data_raw[1]>>gyro_data_raw[2]>>fmu_temperature;
        fmu_timestamp = timestamp_raw[0] + timestamp_raw[1]*1e-6;

        // 比例因子缩放 和 坐标系变换
        acc_data_ned[0] = acc_data_raw[2]/100.0;
        acc_data_ned[1] = acc_data_raw[0]/100.0;
        acc_data_ned[2] = acc_data_raw[1]/100.0;

        gyro_data_ned[0] = gyro_data_raw[2]/10.0*D2R;
        gyro_data_ned[1] = gyro_data_raw[0]/10.0*D2R;
        gyro_data_ned[2] = gyro_data_raw[1]/10.0*D2R;

        double acc_data_ned_new[3], gyro_data_ned_new[3];
        Array3Rotation(m_R_rod2camera, acc_data_ned, acc_data_ned_new);
        Array3Rotation(m_R_rod2camera, gyro_data_ned, gyro_data_ned_new);

        m_rod_imu_data.timestamp = fmu_timestamp;
        memcpy(m_rod_imu_data.acc, acc_data_ned_new, sizeof(m_rod_imu_data.acc));
        memcpy(m_rod_imu_data.gyro, gyro_data_ned_new, sizeof(m_rod_imu_data.gyro));

        // 低通滤波
//        if(m_is_first_read_rod_acc){
//            m_is_first_read_rod_acc = false;
//            m_pre_fmu_timestamp = fmu_timestamp;
//            memcpy(acc_data_filter_pre, acc_data_ned, sizeof(double)*3);
//            memcpy(gyro_data_filter_pre, gyro_data_ned, sizeof(double)*3);
//        }else{
//            double dt_fmu = fmu_timestamp - m_pre_fmu_timestamp;
//            dt_fmu = 1/m_rod_sample_hz; //
//            m_rod_attitude_estimate.LowpassFilter3f(acc_data_filter_pre, acc_data_ned, dt_imu, m_acc_filt_hz, acc_data_filter);
//            memcpy(acc_data_filter_pre, acc_data_filter, sizeof(double)*3);
//
//            m_rod_attitude_estimate.LowpassFilter3f(gyro_data_filter_pre, gyro_data_ned, dt_imu, m_gyro_filt_hz, gyro_data_filter);
//            memcpy(gyro_data_filter_pre, gyro_data_filter, sizeof(double)*3);
//            m_pre_fmu_timestamp = fmu_timestamp;
//        }
        is_fmu_data_update = true;
    }else if(data_flag == "turnlamp"){
        string raw_flag;
        double turnlamp_timestamp;

        m_turnlamp_state_pre = m_turnlamp_state; // 保存上一次的state数据
        ss_log>>timestamp_raw[0]>>timestamp_raw[1]>>raw_flag>>m_turnlamp_state;
        turnlamp_timestamp = timestamp_raw[0] + timestamp_raw[1]*1e-6;
        if(m_turnlamp_state == 2)
            m_turnlamp_state = -1;

        // 判断turnlamp状态是否发生变化
        if(m_turnlamp_state != m_turnlamp_state_pre){
            VLOG(VLOG_INFO)<<"ReadFmuDataFromLog--"<<"!!!new turnlamp state = old:"<<m_turnlamp_state_pre<<" new:"<<m_turnlamp_state<<endl;
            m_turnlamp_state_change_CAN = true;
        }else{
            m_turnlamp_state_change_CAN = false;
        }
        
    }
    if(is_fmu_data_update)
        return 1;
    else
        return 0;
}

// 0: 没有读到新数据
// 1: fmu数据更新
int TurnlampDetector::ReadRodDataOnline( )
{
    double acc_data_raw[3],acc_data_ned_new[3];
    
    ROD_DATA rod_data[20];
    int read_rod_state = HalIO::Instance().ReadRodData(rod_data, 20);
    if(read_rod_state){
        // 比例因子缩放 和 坐标系变换
        for(int k=0; k<3; k++){
            acc_data_raw[k] = rod_data[read_rod_state-1].acc[k]*m_accel_range_scale*ONE_G;  // scale: fmu /100
        }
        double time_cur = rod_data[read_rod_state-1].tv.tv_sec + rod_data[read_rod_state-1].tv.tv_usec*1e-6;

        Array3Rotation(m_R_rod2camera, acc_data_raw, acc_data_ned_new);
        // LowpassFilter3f
        if(m_is_first_rod_acc_filter){
            memcpy(m_rod_acc_pre, acc_data_ned_new, sizeof(acc_data_ned_new));
            m_is_first_rod_acc_filter = false;
            m_acc_data_timestamp_pre = time_cur;
        }

        double dt = time_cur - m_acc_data_timestamp_pre;
        double acc_data_filter[3];
        LowpassFilter3f(m_rod_acc_pre, acc_data_ned_new, dt, 10, acc_data_filter);  
        memcpy(m_rod_imu_data.acc, acc_data_filter, sizeof(acc_data_filter));
        m_rod_imu_data.timestamp = time_cur;
        
        //update
        m_acc_data_timestamp_pre = time_cur;
        memcpy(m_rod_acc_pre, acc_data_filter, sizeof(acc_data_filter));
        return 1;        
    }
    return 0;
}


// 两个IMU相互之间坐标转换矩阵
void TurnlampDetector::CalculateRotationFmu2Camera(const double acc_fmu[3], const double acc_camera[3])
{
    double att_fmu[3], att_camera[3];
    double R_fmu_n2b[3][3], R_camera_n2b[3][3]; // n2b 坐标转换矩阵
    att_fmu[0] = atan2f(-acc_fmu[1], -acc_fmu[2]); // roll
    att_fmu[1] = atan2f(acc_fmu[0], sqrtf(acc_fmu[1]*acc_fmu[1] + acc_fmu[2]*acc_fmu[2])); // pitch
    att_fmu[2] = 0.0;

    att_camera[0] = atan2f(-acc_camera[1], -acc_camera[2]); //roll
    att_camera[1] = atan2f(acc_camera[0], sqrtf(acc_camera[1]*acc_camera[1] + acc_camera[2]*acc_camera[2]));
    att_camera[2] = 0.0;

    m_rod_attitude_estimate.CalculateAtt2Rotation(att_fmu, R_fmu_n2b);
    m_rod_attitude_estimate.CalculateAtt2Rotation(att_camera, R_camera_n2b);

    // R_fmu2camera = R_camera_n2b * R_fmu_n2b'
    for(int  k=0; k<3; k++){
        for(int j=0; j<3; j++){
            m_R_rod2camera[k][j] = R_camera_n2b[k][0]*R_fmu_n2b[j][0] + R_camera_n2b[k][1]*R_fmu_n2b[j][1] + R_camera_n2b[k][2]*R_fmu_n2b[j][2];
        }
    }
}

// 用于外部调用接口，开启初始对准
void TurnlampDetector::SartRotationInitDataCollect()
{
    // 重置转换矩阵
    memset(m_R_rod2camera, 0 , sizeof(m_R_rod2camera));
    m_R_rod2camera[0][0] = 1.0;
    m_R_rod2camera[1][1] = 1.0;
    m_R_rod2camera[2][2] = 1.0;
    m_is_init_rod_acc_collect_start = true;
    printf("Sart rotation init acc data collect!\n");
}

// 收集acc数据
// 1: 正在收集
// 2: 收集完成
//输出: m_mean_init_rod_acc[3]
int TurnlampDetector::CollectInitRodAccData()
{
    memcpy(&m_rod_acc_queue.acc_data[m_rod_acc_queue.wr_index].acc, m_rod_imu_data.acc, sizeof(m_rod_imu_data.acc));
    m_rod_acc_queue.acc_data[m_rod_acc_queue.wr_index].timestamp = m_rod_imu_data.timestamp;

    // 更新读写index
    m_rod_acc_queue.wr_index = (m_rod_acc_queue.wr_index + 1);
    if(m_rod_acc_queue.wr_index == ROD_ACC_QUEUE_SIZE){
         // 数据收集完成进行计算 m_rod_acc_queue
        int mean_num_counter = 0;
        for(int i=m_rod_acc_queue.rd_index; i<m_rod_acc_queue.wr_index; i++){
            for(int k=0; k<3; k++)
                m_mean_init_rod_acc[k] += m_rod_acc_queue.acc_data[i].acc[k];
            mean_num_counter++;                            
        }
        
        for(int i=0; i<3; i++)
            m_mean_init_rod_acc[i] = m_mean_init_rod_acc[i]/mean_num_counter;
        m_is_init_rod_acc_collect_start = false;
        m_is_init_road_acc_collect_ok = true; // 拨杆acc的初始化均值读取ok
        
        return 2;
    }
    return 1;
}

// 重置初始化对准的状态
void TurnlampDetector::ResetInitMatchState()
{
    // reset state
    m_is_init_rod_acc_collect_start = false;
    m_is_init_road_acc_collect_ok = false;
    memset(&m_rod_acc_queue, 0, sizeof(m_rod_acc_queue));
    memset(&m_camera_acc_queue, 0, sizeof(m_camera_acc_queue));
}



// array_new = R*array_origin;
void TurnlampDetector::Array3Rotation(const double R[3][3], const double array_origin[3], double array_new[3])
{
    // array_new = R*array_origin;
    for(int  k=0; k<3; k++)
        array_new[k] = R[k][0]*array_origin[0] + R[k][1]*array_origin[1] + R[k][2]*array_origin[2];
}

// 读取初始化配置文件
int TurnlampDetector:: ReadRodInitParameter()
{
    string buffer_log;
    stringstream ss_log;
    string data_flag;
    double data_value[3];

    ifstream file_turnlamp (FLAGS_turnlamp_detect_init_addr.c_str());
    if(file_turnlamp.is_open()){
        while(!file_turnlamp.eof()){
            getline(file_turnlamp, buffer_log);
            ss_log.clear();
            ss_log.str(buffer_log);
            ss_log>>data_flag>>data_value[0]>>data_value[1]>>data_value[2];

            if(data_flag == "d_acc_threshold"){
                memcpy(m_rod_self_detect_threshold, data_value, sizeof(m_rod_self_detect_threshold));
                LOG(ERROR)<<"TD:ReadRodSelfShiftParameter--"<<" "<<data_flag.c_str()<<": "<<data_value[0]<<", "
                    <<data_value[1]<<", "<<data_value[2]<<endl;
            }else if(data_flag == "d_acc_threshold_weight"){
                memcpy(m_rod_self_detect_threshold_weight, data_value, sizeof(m_rod_self_detect_threshold_weight));
                LOG(ERROR)<<"TD:ReadRodSelfShiftParameter--"<<" "<<data_flag.c_str()<<": "<<data_value[0]<<", "
                    <<data_value[1]<<", "<<data_value[2]<<endl;
            }
        }
    }else{
        LOG(ERROR)<<"TD:ReadRodSelfShiftParameter--"<<"open read turnlamp detect init parameter file error!!!"<<endl;
        return -1;
    }
    file_turnlamp.close();
    return 1;
}

// 用于外部调用接口，开启初始对准
void TurnlampDetector::StartCalculateRodAccThreshold()
{
    m_is_rod_self_detect_threshold_start = true;
    printf("Sart Calculate Rod Acc Threshold!\n");
}


// 计算自检测的阈值
// 1: 正在收集数据 计算阈值
// 2: 阈值计算完成
int TurnlampDetector:: CalculateRodSelfDetectThreshold( )
{
    if(m_rod_self_detect_threshold_counter < ROD_ACC_THRESHOLD_SIZE){
        memcpy(m_rod_detect_acc_data[m_rod_self_detect_threshold_counter].acc, m_d_rod_acc, sizeof(m_d_rod_acc));   
        printf("new index %d rod acc: %f %f %f\n", m_rod_self_detect_threshold_counter, m_rod_detect_acc_data[m_rod_self_detect_threshold_counter].acc[0], 
                    m_rod_detect_acc_data[m_rod_self_detect_threshold_counter].acc[1], m_rod_detect_acc_data[m_rod_self_detect_threshold_counter].acc[2]);
        m_rod_self_detect_threshold_counter++;
    }else{
        double rod_acc_t[3] = {0, 0, 0};
        for(int i=0; i<ROD_ACC_THRESHOLD_SIZE; i++){
            for(int k=0; k<3; k++)
                rod_acc_t[k] += fabs(m_rod_detect_acc_data[i].acc[k]);
        }

        for(int i=0; i<3; i++)
            m_rod_self_detect_threshold[i] = rod_acc_t[i]/ROD_ACC_THRESHOLD_SIZE/8; // 为了均衡
        m_is_rod_self_detect_threshold_start = false;

        // 挑选最大的权值对应的轴
        double max_threshold = max(m_rod_self_detect_threshold[0], max(m_rod_self_detect_threshold[1], m_rod_self_detect_threshold[2]));
        for(int i=0; i<3; i++){
            if(max_threshold == m_rod_self_detect_threshold[i])
                m_rod_self_detect_threshold_weight[i] = 1;
            else
                m_rod_self_detect_threshold_weight[i] = 0;
        }
            
        printf("!!!!---new rod_self_detect_threshold: %f %f %f\n", m_rod_self_detect_threshold[0], m_rod_self_detect_threshold[1], m_rod_self_detect_threshold[2]);
        printf("!!!!---new rod_self_detect_threshold_weight: %f %f %f\n", m_rod_self_detect_threshold_weight[0], m_rod_self_detect_threshold_weight[1], m_rod_self_detect_threshold_weight[2]);
        return 1;
    }
    return 0;
}

// 获取拨杆的数据
void TurnlampDetector:: GetRodAccData( StructImuData *rod_acc_data)
{
    m_rod_acc_rw_lock.ReaderLock();
    memcpy(rod_acc_data, &m_rod_imu_data, sizeof(StructImuData));
    m_rod_acc_rw_lock.Unlock();
}


// 获取自检测阈值
void TurnlampDetector:: GetRodSelfDetectThreshold(double threshold_t[3], double weight_t[3] )
{
    memcpy(threshold_t, m_rod_self_detect_threshold, sizeof(m_rod_self_detect_threshold));
    memcpy(weight_t, m_rod_self_detect_threshold_weight, sizeof(m_rod_self_detect_threshold));
}

// 设置自检测阈值
void TurnlampDetector:: SetRodSelfDetectThreshold(const double threshold_t[3], const double weight_t[3] )
{
    memcpy(m_rod_self_detect_threshold, threshold_t, sizeof(m_rod_self_detect_threshold));
    memcpy(m_rod_self_detect_threshold_weight, weight_t, sizeof(m_rod_self_detect_threshold));

    printf("set new m_rod_self_detect_threshold = %f %f %f\n", m_rod_self_detect_threshold[0], m_rod_self_detect_threshold[1], m_rod_self_detect_threshold[2]);
    printf("set new m_rod_self_detect_threshold_weight = %f %f %f\n", m_rod_self_detect_threshold_weight[0], m_rod_self_detect_threshold_weight[1], m_rod_self_detect_threshold_weight[2]);
}

// 读取 m_R_rod2camera
void TurnlampDetector:: GetRod2CameraRotation(double R[3][3] )
{
    memcpy(R, m_R_rod2camera, sizeof(m_R_rod2camera));

    printf("get new m_R_rod2camera = %f %f %f\n", m_R_rod2camera[0][0], m_R_rod2camera[0][1], m_R_rod2camera[0][2]);
    printf("                         %f %f %f\n", m_R_rod2camera[1][0], m_R_rod2camera[1][1], m_R_rod2camera[1][2]);
    printf("                         %f %f %f\n", m_R_rod2camera[2][0], m_R_rod2camera[2][1], m_R_rod2camera[2][2]);
}


// 设置 m_R_rod2camera
void TurnlampDetector:: SetRod2CameraRotation(const double R[3][3] )
{
    memcpy(m_R_rod2camera, R, sizeof(m_R_rod2camera));

    printf("set new m_R_rod2camera = %f %f %f\n", m_R_rod2camera[0][0], m_R_rod2camera[0][1], m_R_rod2camera[0][2]);
    printf("                         %f %f %f\n", m_R_rod2camera[1][0], m_R_rod2camera[1][1], m_R_rod2camera[1][2]);
    printf("                         %f %f %f\n", m_R_rod2camera[2][0], m_R_rod2camera[2][1], m_R_rod2camera[2][2]);
}





}
