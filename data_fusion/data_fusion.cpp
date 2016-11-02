#include "data_fusion.h"


//# define DATA_FROM_LOG 1  // 从log中读取数据
//# define DATA_FROM_ONLINE 0 //在线读取数据

using namespace common;
DataFusion::DataFusion()
{
    Init();    
}

DataFusion::~DataFusion()
{
    #if !defined(ANDROID)
    {
        infile_log.close();        
    }
    #endif
    
    m_is_running = false;
    m_fusion_thread.StopAndWaitForExit();    
}


void DataFusion::Init( )
{
    // read data from log
    #if !defined(ANDROID)
    {
        #if defined(USE_RADIUS)
        {
            infile_log.open("data/radius/log.txt");       // ofstream 
            if(!infile_log)
            {
                printf("open \"data/doing/log.txt\" ERROR!!\n");
            }
        }
        #else
        {
            infile_log.open("data/doing/log.txt");       // ofstream   
            if(!infile_log)
            {
                printf("open \"data/doing/log.txt\" ERROR!!\n");
            }
        }
        #endif
    }
    #endif

    #if defined(ANDROID)
    {
        int stste = init_gsensor();
        if(stste < 0)
        {
            printf("DataFusion::Init--init_gsensor ERROR!!!\n");
        }
    }       
    #endif
            
    m_pre_vehicle_timestamp = 0.0f; /// CAN;

    // IMU
    m_imu_sample_hz = 100.0;
    m_imu_dt_set = 1/m_imu_sample_hz;
    m_acc_filt_hz = 5.0; // 加速度计的低通截止频率
    m_gyro_filt_hz = 20.0;
    m_isFirstTime_att = 1; // 是否是第一次进入
    m_pre_imu_timestamp = 0.0f; // IMU数据上次得到的时刻     
    m_is_first_speed_data = 1; //  1: 第一次获取到speed数据 0:不是第一次
    m_is_print_imu_data = 0; // 是否打印IMU数据
    m_is_print_speed_data = 0;

    // 转弯半径R
    m_gyro_R_filt_hz = 2.0;
    m_can_speed_R_filt_hz = 2.0;
    m_call_radius_timestamp = 0;
    
    // read data
    m_is_first_read_gsensor = 1; 
    m_data_gsensor_update = 0;
    m_data_speed_update = 0;
    m_data_image_update = 0;

    
    // 读取数据控制
    m_is_first_fusion_timestamp = 1; // 第一次更新
    m_is_first_data_timestamp = 1; // 第一次更新
    m_data_save_length = 2; // 保存历史数据的长度(时间为单位: s)
    m_is_continue_read_data = 1; // 1:继续读取数据  2:暂停读取数据 由fusion控制   
    m_cur_data_timestamp = 0;
    m_call_predict_timestamp = 0;
    m_is_first_run_read_data = 1; // 第一次运行读取数据

    


}

// 开始线程
void DataFusion::StartDataFusionTask()
{
    m_fusion_thread.Start();
    m_is_running= 1;

    Closure<void>* cls = NewClosure(this, &DataFusion::RunFusion);
    m_fusion_thread.AddTask(cls);
}

// 线程循环执行的函数
void DataFusion::RunFusion( )
{
    while(1)
    {
        int read_state = ReadData(); // 数据保存在  m_vector_imu_data
        while (!m_vector_imu_data.empty())
        {
            //memcpy(m_imu_data, m_vector_imu_data.back(), sizeof(StructImuData))
            m_imu_data = m_vector_imu_data.back();
            m_vector_imu_data.pop_back();

            EstimateAtt();
            EstimateVehicelState(); 
            CalculateVehicleTurnRadius();
        }
            
        DeleteOldData();  
        DeleteOldRadiusData();
        usleep(1); // 1us       
    }
}


// 读取数据,两种方式:
//  1.离线从log   
//  2.在线
int DataFusion::ReadData( )
{ 
    int rtn_state;    
    #if !defined(ANDROID)
    {
        //从log中离线读取数据，需要通过时间戳来确定是否要继续读取数据，更新is_continue_read_data
        UpdateRreadDataState();
        rtn_state = ReadDataFromLog();
        return rtn_state;
    }
    #else 
    {
        int imu_state = ReadImuOnline();
        int speed_state = ReadSpeedOnline();
        //TODO: rtn_state
        return imu_state;
    }
    #endif
}

int DataFusion::ReadDataFromLog( )
{
    double log_data[2], timestamp_raw[2];
    string data_flag;
    struct StructImuData imu_data;
    // 第一次运行程序，初始化m_cur_data_timestamp
    if(m_is_first_run_read_data)
    {
        getline(infile_log, buffer_log);
        ss_tmp.clear();
        ss_tmp.str(buffer_log);
        ss_tmp>>log_data[0]>>log_data[1]>>data_flag; 
        m_cur_data_timestamp = log_data[0] + log_data[1]*1e-6;
        m_is_first_run_read_data = 0;
    }
    
    if(m_is_continue_read_data )
    {
        getline(infile_log, buffer_log);
        ss_tmp.clear();
        ss_tmp.str(buffer_log);
        ss_tmp>>log_data[0]>>log_data[1]>>data_flag;
        ss_log.clear();
        ss_log.str(buffer_log);

        if(data_flag == "cam_frame")
        {
            string camera_flag, camera_add, image_index_str;
            string image_name;
            int log_image_index;
            ss_log>>timestamp_raw[0]>>timestamp_raw[1]>>camera_flag>>camera_add>>log_image_index;
            
            m_image_frame_info.timestamp = timestamp_raw[0] + timestamp_raw[1]*1e-6;
            m_image_frame_info.index = log_image_index;
            m_data_image_update = 1;                
        }
        else if(data_flag == "Gsensor")
        {            
            double acc_data_raw[3]; // acc原始坐标系下的
            double acc_data_ned[3]; // 大地坐标系
            static double acc_data_filter[3]; // 一阶低通之后的数据
            double gyro_data_raw[3];
            double gyro_data_ned[3];  
            static double gyro_data_filter[3]; // 一阶低通之后的数据
            double imu_temperature, imu_timestamp;    
            string imu_flag;

            m_vector_imu_data.clear(); // 清空vector

            ss_log>>timestamp_raw[0]>>timestamp_raw[1]>>imu_flag>>acc_data_raw[0]>>acc_data_raw[1]>>acc_data_raw[2]
                    >>gyro_data_raw[0]>>gyro_data_raw[1]>>gyro_data_raw[2]>>imu_temperature;
            imu_timestamp = timestamp_raw[0] + timestamp_raw[1]*1e-6;                 
            m_imu_attitude_estimate.AccDataCalibation(acc_data_raw, acc_data_ned);// 原始数据校正
            m_imu_attitude_estimate.GyrocDataCalibation(gyro_data_raw, gyro_data_ned);

            if(m_is_first_read_gsensor)
            {
                m_is_first_read_gsensor = 0;
                m_pre_imu_timestamp = imu_timestamp;
                acc_data_filter[0] = acc_data_ned[0];
                acc_data_filter[1] = acc_data_ned[1];
                acc_data_filter[2] = acc_data_ned[2];
                gyro_data_filter[0] = gyro_data_ned[0];
                gyro_data_filter[1] = gyro_data_ned[1];
                gyro_data_filter[2] = gyro_data_ned[2]; 
            }
            else
            {
                double dt_imu = imu_timestamp - m_pre_imu_timestamp; 
                dt_imu = 1/m_imu_sample_hz; // 100hz
                m_imu_attitude_estimate.LowpassFilter3f(acc_data_filter, acc_data_ned, dt_imu, m_acc_filt_hz, acc_data_filter);    
                m_imu_attitude_estimate.LowpassFilter3f(gyro_data_filter, gyro_data_ned, dt_imu, m_gyro_filt_hz, gyro_data_filter);       
                m_pre_imu_timestamp = imu_timestamp;    
            }

            // 更新数据
            for(int i = 0; i<3; i++)
            {
                imu_data.acc[i] = acc_data_filter[i];
                imu_data.gyro[i] = gyro_data_filter[i];                    
                imu_data.acc_raw[i] = acc_data_ned[i];
                imu_data.gyro_raw[i] = gyro_data_ned[i];                    
            }
            imu_data.timestamp = imu_timestamp;
            imu_data.temp = imu_temperature;
            m_vector_imu_data.push_back(imu_data); // 保存IMU data的 vector
            UpdateCurrentDataTimestamp(imu_timestamp);
            m_data_gsensor_update = 1;    
           
        }else if(data_flag == "brake_signal")
        {
            string str_t[10],str_speed;
            int data_t[10];
            double raw_timestamp[2];  
            double speed_can, speed_timestamp;
            ss_log>>raw_timestamp[0]>>raw_timestamp[1]
                >>str_t[0]>>data_t[0]>>str_t[1]>>data_t[1]>>str_t[2]>>data_t[2]>>str_t[3]>>data_t[3]>>str_t[4]>>data_t[4]
                >>str_t[5]>>data_t[5]>>str_t[6]>>data_t[6]>>str_t[7]>>data_t[7]>>str_t[8]>>data_t[8]>>str_t[9]>>data_t[9]
                >>str_speed>>speed_can;  
            
            speed_timestamp = raw_timestamp[0] + raw_timestamp[1]*1e-6;   
            speed_can = speed_can/3.6;// km/h-->m/s 

            // 更新数据
            m_can_speed_data.timestamp = speed_timestamp;
            m_can_speed_data.speed = speed_can; 
            UpdateCurrentDataTimestamp(speed_timestamp);
            m_data_speed_update = 1;  
        }
    }
}



// 在线读取imu数据
int DataFusion::ReadImuOnline( )
{   
    int imu_fifo_total;
    int fifo_reseted = 0;
    double time_imu_read; // 从IMU的FIFO中读取数据的当前系统时间
    struct timeval time_imu;
    struct GsensorData data_raw[80];
    struct StructImuData imu_data;

    m_vector_imu_data.clear(); // 清空vector
    imu_fifo_total = read_gsensor(data_raw, sizeof(data_raw)/sizeof(data_raw[0]));
    gettimeofday(&time_imu, NULL);
    time_imu_read = time_imu.tv_sec + time_imu.tv_usec*1e-6;

    // IMU数据读取异常判断
    if (imu_fifo_total <= 0) 
    {
        if( GSENSOR_READ_AGAIN == imu_fifo_total)
        {
            return GSENSOR_READ_AGAIN;
        }
        else if (GSENSOR_FIFO_RESET == imu_fifo_total) 
        {
            fifo_reseted = 1 ;// IMU FIFO 溢出
            printf("ReadImuOnline: IMU fifo leak \n");
            return GSENSOR_FIFO_RESET;
        }
        else
        {
            return GSENSOR_READ_FAIL;    
        }           
    }

    for (int imu_data_index = 0; imu_data_index < imu_fifo_total; imu_data_index++) 
    {
        double acc_data_raw[3]; // acc原始坐标系下的
        double acc_data_ned[3]; // 大地坐标系
        static double acc_data_filter[3]; // 一阶低通之后的数据            
        double gyro_data_raw[3];
        double gyro_data_ned[3];  
        static double gyro_data_filter[3]; // 一阶低通之后的数据
        double imu_timestamp, imu_temperature;

        for(int k=0; k<3; k++)
        {
            acc_data_raw[k] = data_raw[imu_data_index].accel[k];
            gyro_data_raw[k] = data_raw[imu_data_index].gyro[k];
        }
        imu_temperature = Raw2Degree(data_raw[imu_data_index].temp);
        // 利用FIFO数据长度和当前的时间戳，进行IMU时间逆推
        imu_timestamp = time_imu_read - (imu_fifo_total-1-imu_data_index)*m_imu_dt_set;
         
        m_imu_attitude_estimate.AccDataCalibation(acc_data_raw, acc_data_ned);// 原始数据校正
        m_imu_attitude_estimate.GyrocDataCalibation(gyro_data_raw, gyro_data_ned);

        if(m_is_first_read_gsensor) 
        { 
            // 第一次进入函数的初始化
            for(int k=0; k<3; k++)
            {
                acc_data_filter[k] = acc_data_ned[k];
                gyro_data_filter[k] = gyro_data_ned[k];
            }
            m_is_first_read_gsensor = 0;
        }
        else
        {
            m_imu_attitude_estimate.LowpassFilter3f(acc_data_filter, acc_data_ned, m_imu_dt_set, m_acc_filt_hz, acc_data_filter);    
            m_imu_attitude_estimate.LowpassFilter3f(gyro_data_filter, gyro_data_ned, m_imu_dt_set, m_gyro_filt_hz, gyro_data_filter);        
        }

        // 更新数据
        imu_data.timestamp = imu_timestamp;
        imu_data.temp = imu_temperature;
        for(int i = 0; i<3; i++)
        {
            imu_data.acc[i] = acc_data_filter[i];
            imu_data.gyro[i] = gyro_data_filter[i];                    
            imu_data.acc_raw[i] = acc_data_ned[i];
            imu_data.gyro_raw[i] = gyro_data_ned[i];                    
        }
        m_vector_imu_data.push_back(imu_data); // 保存IMU data的 vector
        m_data_gsensor_update = 1;     
        fifo_reseted = 0;

        // test
        if(m_is_print_imu_data)
        {
            char buf1[256];
            snprintf(buf1, sizeof(buf1), "imu %f %f %f %f %f %f %f %f %d %d", imu_data.timestamp,
                        imu_data.acc[0], imu_data.acc[1], imu_data.acc[2],
                        imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2], imu_data.temp, imu_fifo_total, fifo_reseted);

            if (imu_fifo_total >= 1)
            {                
                printf("#%02d %s\n", imu_data_index, buf1);
            }
        }
    }
    UpdateCurrentDataTimestamp(time_imu_read);

    return imu_fifo_total;    
}

// 读取speed数据
int DataFusion::ReadSpeedOnline( )
{
    struct timeval time_speed;
    gettimeofday(&time_speed, NULL);
    double speed_time_t = time_speed.tv_sec + time_speed.tv_usec*1e-6;
    double speed_can = HalIO::Instance().GetSpeed()/3.6;

    // 更新数据
    m_can_speed_data.timestamp = speed_time_t;
    m_can_speed_data.speed = speed_can; 

    if(m_is_print_speed_data)
    {
        char buf1[256];
        snprintf(buf1, sizeof(buf1), "speed %f %f", m_can_speed_data.timestamp, speed_can);              
        printf("# %s\n",  buf1);        
    }

    return 0;    
}


// 更新当前data的时间戳，用于控制读取数据的长度
void DataFusion::UpdateCurrentDataTimestamp( double data_timestample)
{
    if(m_is_first_data_timestamp)
    {
        m_cur_data_timestamp = data_timestample;
        m_is_first_data_timestamp = 0;
    }else
    {
        if(m_cur_data_timestamp < data_timestample)
        {
            m_cur_data_timestamp = data_timestample;
        }
    }
}


// 判断是否还要继续读取数据，如果run_fusion在进行操作的时候，不读数据
bool  DataFusion::UpdateRreadDataState( )
{
    double dt  = m_cur_data_timestamp - m_call_predict_timestamp;
    // 提前读取data_save_length长度的数据
    if(dt >= m_data_save_length)  // 时间超过了
    {
        m_is_continue_read_data = 0; //  暂停读取数据
    }else
    {
        m_is_continue_read_data = 1;
    }

    return m_is_continue_read_data;
}

// 根据设定最长记忆时间的历史数据，删除多余数据
void DataFusion::DeleteOldData( )
{
    double dt;
    int att_data_length = m_vector_att.size();    
    int delete_conter = 0;
    double cur_timestamp = 0;
    
    #if !defined(ANDROID)
    {
        feature_rw_lock.ReaderLock();
        cur_timestamp = m_call_predict_timestamp;
        feature_rw_lock.Unlock();
    }
    #else 
    {
        struct timeval time_speed;
        gettimeofday(&time_speed, NULL);
        cur_timestamp = time_speed.tv_sec + time_speed.tv_usec*1e-6;
    }
    #endif 
    
    for(int i=0; i<att_data_length; i++)
    {
        dt = (m_vector_att.begin()+i)->timestamp - cur_timestamp;
        if(dt < -m_data_save_length)
            delete_conter++;
        else
            break;
    }    
    if(delete_conter > 0)
    {
        // 检测出比当前m_call_predict_timestamp早data_save_length秒前的数据，并删除
        m_vector_att.erase(m_vector_att.begin(), m_vector_att.begin()+delete_conter);
    }    

    // 汽车运动数据
    int vehicle_data_length = m_vector_vehicle_state.size(); 
    delete_conter = 0;
    for(int i=0; i<vehicle_data_length; i++)
    {
        dt = (m_vector_vehicle_state.begin()+i)->timestamp - cur_timestamp;
        if(dt < -m_data_save_length)
            delete_conter++;
        else
            break;
    }    
    if(delete_conter > 0)
    {
        // 检测出比当前m_call_predict_timestamp早data_save_length秒前的数据，并删除
        m_vector_vehicle_state.erase(m_vector_vehicle_state.begin(), m_vector_vehicle_state.begin()+delete_conter);  
    }
    
}


// 根据设定最长记忆时间的历史数据，删除多余转弯半径的数据
void DataFusion::DeleteOldRadiusData( )
{
    double dt;
    int R_data_length = m_vector_turn_radius.size();    
    int R_delete_conter = 0;
    double R_cur_timestamp = 0;

    #if !defined(ANDROID)
    {
        radius_rw_lock.ReaderLock();
        R_cur_timestamp = m_call_predict_timestamp;
        radius_rw_lock.Unlock();
    }
    #else 
    {
        struct timeval time_R;
        gettimeofday(&time_R, NULL);
        R_cur_timestamp = time_R.tv_sec + time_R.tv_usec*1e-6;
    }
    #endif
    
    for(int i=0; i<R_data_length; i++)
    {
        dt = (m_vector_turn_radius.begin()+i)->timestamp - R_cur_timestamp;
        if(dt < -m_data_save_length)
            R_delete_conter++;
        else
            break;
    }
    
    if(R_delete_conter > 0)
    {
        // 检测出比当前m_call_predict_timestamp早data_save_length秒前的数据，并删除
        m_vector_turn_radius.erase(m_vector_turn_radius.begin(), m_vector_turn_radius.begin()+R_delete_conter);
    }    
}


void DataFusion::EstimateAtt()
{
    StructImuData imu_data;
    memcpy(&imu_data, &m_imu_data, sizeof(StructImuData));     
    
    double cur_att_timestamp = imu_data.timestamp;
    if(m_isFirstTime_att)
    {
        m_isFirstTime_att = 0;
        m_pre_att_timestamp = cur_att_timestamp; 
    }else
    {  
        VLOG(VLOG_DEBUG)<<"run fusion imu" <<endl;
        
        double dt_att = cur_att_timestamp - m_pre_att_timestamp;
        dt_att = 1/m_imu_sample_hz;
        m_imu_attitude_estimate.UpdataAttitude(imu_data.acc, imu_data.gyro, dt_att);
        m_pre_att_timestamp = cur_att_timestamp;

        // save att
        m_imu_attitude_estimate.GetAttitudeAngleZ(m_struct_att.att, &(m_struct_att.angle_z));
        m_struct_att.timestamp = cur_att_timestamp;
        m_vector_att.push_back(m_struct_att);
    }     
    
}


void DataFusion::EstimateVehicelState()
{  
    StructCanSpeedData can_speed_data;
    memcpy(&can_speed_data, &m_can_speed_data, sizeof(StructCanSpeedData));

    // 利用imu+speed计算汽车运动
    double dt;
    double cur_vehicle_timestamp = m_struct_att.timestamp; 
    if(m_is_first_speed_data)
    {
        m_is_first_speed_data = 0;
        m_pre_vehicle_timestamp = cur_vehicle_timestamp;  
        
    }else
    {
        //dt = cur_vehicle_timestamp - m_pre_vehicle_timestamp; // 暂时没用
        dt = 1/m_imu_sample_hz; // 每次IMU更新数据便计算一次
        m_can_vehicle_estimate.UpdateVehicleStateImu(m_struct_att.angle_z, can_speed_data.speed, dt );
        m_pre_vehicle_timestamp = cur_vehicle_timestamp;

        // save vehicle state            
        m_can_vehicle_estimate.GetVehicleState(m_struct_vehicle_state.vel, m_struct_vehicle_state.pos, &(m_struct_vehicle_state.yaw));
        m_struct_vehicle_state.timestamp = cur_vehicle_timestamp;
        m_vector_vehicle_state.push_back(m_struct_vehicle_state);
    }

}


// 根据时间戳查找对应的数据
int DataFusion::GetTimestampData(double timestamp_search, double vehicle_pos[2], double att[3], double *angle_z )
{
    // m_vector_att
    bool att_data_search_ok = 0;
    bool vehicle_data_search_ok = 0;
    int att_data_length = m_vector_att.size();
    double timestamp_cur, timestamp_pre, dt_t_cur, dt_t_pre, dt_t;

    if(att_data_length >= 2)
    {
        for(int i = 1; i<att_data_length; i++)
        {
            timestamp_cur = (m_vector_att.end()-i)->timestamp;
            timestamp_pre = (m_vector_att.end()-i-1)->timestamp;
            dt_t = timestamp_cur - timestamp_pre;
            dt_t_cur = timestamp_cur - timestamp_search;
            dt_t_pre = timestamp_pre - timestamp_search;
     
            if(dt_t_pre<0 && dt_t_cur>0 && dt_t>=0)
            {
                // 方法: 线性差插值
                double att_pre[3], att_cur[3], d_att[3];            
                for(int k = 0; k<3; k++)
                {
                    att_pre[k] = (m_vector_att.end()-i-1)->att[k];
                    att_cur[k] = (m_vector_att.end()-i)->att[k];
                    d_att[k] = att_cur[k] - att_pre[k];                
                    att[k] = att_pre[k] + (fabs(dt_t_pre)/fabs(dt_t))*d_att[k];// 线性插值                
                }

                double angle_z_pre, angle_z_cur, d_angle_z;
                angle_z_pre = (m_vector_att.end()-i-1)->angle_z;
                angle_z_cur = (m_vector_att.end()-i)->angle_z;
                d_angle_z = angle_z_cur - angle_z_pre;                
                *angle_z = angle_z_pre + (fabs(dt_t_pre)/fabs(dt_t))*d_angle_z;// 线性插值

                att_data_search_ok = 1;
                break;
            }
        }
        if(!att_data_search_ok)
        {
            VLOG(VLOG_INFO)<<"DF:GetTimestampData--"<<"att, begin_time= "<<(m_vector_att.end()-1)->timestamp<<", end_time= "<<m_vector_att.begin()->timestamp<<endl; 
            VLOG(VLOG_INFO)<<"DF:GetTimestampData--"<<"att_length= "<<att_data_length<<", att:(ms) "<<"dt_t_cur= "<<dt_t_cur*1000<<", dt_t_pre= "<<dt_t_pre*1000<<endl; 
        }
    }
    
    // m_vector_vehicle_state
    int vehicle_data_length = m_vector_vehicle_state.size();
    if(vehicle_data_length>=2)
    {
        for(int i = 1; i<vehicle_data_length; i++)
        {        
            timestamp_cur = (m_vector_vehicle_state.end()-i)->timestamp;
            timestamp_pre = (m_vector_vehicle_state.end()-i-1)->timestamp;
            dt_t = timestamp_cur - timestamp_pre;
            dt_t_cur = timestamp_cur - timestamp_search;
            dt_t_pre = timestamp_pre - timestamp_search;
            if(dt_t_pre<0 && dt_t_cur>0)
            {
                 // 方法: 线性差插值
                double pos_pre[2], pos_cur[2], d_pos[2] ;
                for(int k=0; k<2; k++)
                {
                    pos_pre[k] = (m_vector_vehicle_state.end()-i-1)->pos[k];
                    pos_cur[k] = (m_vector_vehicle_state.end()-i)->pos[k];
                    d_pos[k] = pos_cur[k] - pos_pre[k];               
                    vehicle_pos[k] = pos_pre[k] + (fabs(dt_t_pre)/fabs(dt_t))*d_pos[k];     // 线性插值            
                }            
                vehicle_data_search_ok = 1;
                break;
            }
        }

        if(!vehicle_data_search_ok)
        {
            VLOG(VLOG_INFO)<<"DF:GetTimestampData--"<<"vehicle_state, begin_time= "<<(m_vector_vehicle_state.end()-1)->timestamp<<", end_time= "<<m_vector_vehicle_state.begin()->timestamp<<endl; 
            VLOG(VLOG_INFO)<<"DF:GetTimestampData--"<<"vehicle_length= "<<vehicle_data_length<<", pos:(ms) "<<"dt_t_cur= "<<dt_t_cur*1000<<", dt_t_pre= "<<dt_t_pre*1000<<endl; 
        }
    }
    
    if(att_data_search_ok && vehicle_data_search_ok)
        return 1;
    else
        return 0;    
}


// 给外部调用的接口:特征点预测
int DataFusion::GetPredictFeature( const std::vector<cv::Point2f>& vector_feature_pre ,int64 image_timestamp_pre, int64 image_timestamp_cur, 
                                               std::vector<cv::Point2f>* vector_feature_predict)
{
    bool is_data_search_cur_ok; // 搜索指定时间戳的数据是否成功
    bool is_data_search_pre_ok;    
    double att_cur[3], att_pre[3], vehicle_pos_cur[2], vehicle_pos_pre[2];
    double image_timestamp_cur_t = image_timestamp_cur/1000.0;
    double image_timestamp_pre_t = image_timestamp_pre/1000.0;    

    feature_rw_lock.WriterLock();
    m_call_predict_timestamp = image_timestamp_cur/1000.0; // 更新时间戳
    feature_rw_lock.Unlock();
   
    // 寻找跟需求的 timestamp对应的att,vehicle数据
    is_data_search_pre_ok = GetTimestampData( image_timestamp_pre_t, vehicle_pos_pre, att_pre, &m_angle_z_pre);
    is_data_search_cur_ok = GetTimestampData( image_timestamp_cur_t, vehicle_pos_cur, att_cur, &m_angle_z_cur);

    VLOG(VLOG_DEBUG)<<"DF:GetPredictFeature--"<<"call: pre = "<<image_timestamp_pre<<", cur: "<<image_timestamp_cur<<endl;    
    VLOG(VLOG_DEBUG)<<"DF:GetPredictFeature--"<<"call: dt(ms) = "<<image_timestamp_cur - image_timestamp_pre<<endl; 


    if(is_data_search_cur_ok && is_data_search_pre_ok)
    {
        FeaturePredict( vector_feature_pre , vehicle_pos_pre, att_pre, m_angle_z_pre, vehicle_pos_cur, att_cur, m_angle_z_pre, vector_feature_predict);
        return 1; 
    }
    else if(!is_data_search_pre_ok && !is_data_search_cur_ok)
    {
        VLOG(VLOG_WARNING)<<"DF:GetPredictFeature--"<<"warning cur & pre-camera both timestamp dismatch"<<endl;
    }
    else if(!is_data_search_pre_ok && is_data_search_cur_ok)
    {
        VLOG(VLOG_WARNING)<<"DF:GetPredictFeature--"<<"warning pre-camera timestamp dismatch"<<endl;
    }
    else if( is_data_search_pre_ok && !is_data_search_cur_ok)
    {
        VLOG(VLOG_WARNING)<<"DF:GetPredictFeature--"<<"warning cur-camera timestamp dismatch"<<endl;
    }
    return 0;    
    
}


// 特征点预测
int DataFusion::FeaturePredict( const std::vector<cv::Point2f>& vector_feature_pre , double vehicle_pos_pre[2], double att_pre[3], double angle_z_pre, 
                                         double vehicle_pos_cur[2], double att_cur[3], double angle_z_cur, std::vector<cv::Point2f>* vector_feature_predict)
{
    // 计算汽车在两帧之间的状态变化    
    // 对pos进行坐标系转换，转到以pre时刻为初始坐标
    double d_pos_tmp[2]; // 在初始时刻的汽车坐标系下，前后两帧的运动位置变化
    double d_pos_new_c[2]; // 在以pre为坐标下的汽车运动
    d_pos_tmp[0] = vehicle_pos_cur[0] - vehicle_pos_pre[0];
    d_pos_tmp[1] = vehicle_pos_cur[1] - vehicle_pos_pre[1];         
//    d_pos_new_c[0] = cosf(att_pre[2])*d_pos_tmp[0] + sinf(att_pre[2])*d_pos_tmp[1];
//    d_pos_new_c[1] = -sinf(att_pre[2])*d_pos_tmp[0] + cos(att_pre[2])*d_pos_tmp[1];
    d_pos_new_c[0] = cosf(angle_z_pre)*d_pos_tmp[0] + sinf(angle_z_pre)*d_pos_tmp[1];
    d_pos_new_c[1] = -sinf(angle_z_pre)*d_pos_tmp[0] + cos(angle_z_pre)*d_pos_tmp[1];


    VLOG(VLOG_DEBUG)<<"DF:FeaturePredict--"<<"vehicle_pos_pre: "<<vehicle_pos_pre[0]<<", "<<vehicle_pos_pre[1]; 
    VLOG(VLOG_DEBUG)<<"DF:FeaturePredict--"<<"vehicle_pos_cur: "<<vehicle_pos_cur[0]<<", "<<vehicle_pos_cur[1]; 
    VLOG(VLOG_DEBUG)<<"DF:FeaturePredict--"<<"d_pos: "<<d_pos_new_c[0]<<" "<<d_pos_new_c[1];
   
    double dyaw = att_cur[2] - att_pre[2]; 
    // test
    double dangle_z = m_angle_z_cur - m_angle_z_pre; 

    VLOG(VLOG_DEBUG)<<"DF:FeaturePredict--"<<"att_pre: "<<att_pre[0]*180/M_PI<<", "<<att_pre[1]*180/M_PI<<", "<<att_pre[2]*180/M_PI; 
    VLOG(VLOG_DEBUG)<<"DF:FeaturePredict--"<<"att_cur: "<<att_cur[0]*180/M_PI<<", "<<att_cur[1]*180/M_PI<<", "<<att_cur[2]*180/M_PI; 
    VLOG(VLOG_DEBUG)<<"DF:FeaturePredict--"<<"dyaw: "<<dyaw*180/M_PI; 
    VLOG(VLOG_DEBUG)<<"DF:FeaturePredict--"<<"dangle_z: "<<dangle_z*180/M_PI; 

    dyaw = dangle_z;
    
    double Rn2c_kT[2][2];
    Rn2c_kT[0][0] = cosf(dyaw);
    Rn2c_kT[0][1] = sinf(dyaw);
    Rn2c_kT[1][0] = -Rn2c_kT[0][1];
    Rn2c_kT[1][1] = Rn2c_kT[0][0];  
    
    // 预测特征点
    double feature_points_nums = vector_feature_pre.size();  
    cv::Point2f XY_pre, XY_predict;
    vector_feature_predict->clear(); // 清空数据
    for(int points_index = 0; points_index<feature_points_nums; points_index++)
    {        
        XY_pre.x = (vector_feature_pre.begin()+points_index)->x;
        XY_pre.y = (vector_feature_pre.begin()+points_index)->y;
        
        double dx = XY_pre.x - d_pos_new_c[0];// NED坐标系下的
        double dy = XY_pre.y - d_pos_new_c[1];
        XY_predict.x = Rn2c_kT[0][0]*dx + Rn2c_kT[0][1]*dy;
        XY_predict.y = Rn2c_kT[1][0]*dx + Rn2c_kT[1][1]*dy;

        vector_feature_predict->push_back(XY_predict);
//        VLOG(VLOG_DEBUG)<<"DF:FeaturePredict- "<<"xy_feature_pre: "<<"x="<<XY_pre.x<<", y="<< XY_pre.y<< endl; 
//        VLOG(VLOG_DEBUG)<<"DF:FeaturePredict- "<<"xy_feature_predict: "<<"x="<<XY_predict.x<<", y="<< XY_predict.y<< endl; 
    } 

    return 1;
}



// 计算汽车的转弯半径
void DataFusion::CalculateVehicleTurnRadius()
{
    double R;
    // IMU
    StructImuData imu_data;
    static double gyro_filter_R[3]; // 一阶低通之后的数据 
    memcpy(&imu_data, &m_imu_data, sizeof(StructImuData));    
    double dt_imu= 1/m_imu_sample_hz;
    m_imu_attitude_estimate.LowpassFilter3f(gyro_filter_R, imu_data.gyro_raw, dt_imu, m_gyro_R_filt_hz, &gyro_filter_R[0]);    

    // CAN speed
    StructCanSpeedData can_speed_data;
    memcpy(&can_speed_data, &m_can_speed_data, sizeof(StructCanSpeedData));
   
    if(fabs(gyro_filter_R[2])>0.01 && fabs(can_speed_data.speed)>15/3.6) 
    {
        R = can_speed_data.speed/gyro_filter_R[2];
    }
    else // 太小的速度和角速度
    {        
        R = 0;
    }
    
    // save R
    m_struct_turn_radius.timestamp = imu_data.timestamp;
    m_struct_turn_radius.R = R;
    m_vector_turn_radius.push_back(m_struct_turn_radius);
    
}


// 给外部调用的接口:特征点预测
int DataFusion::GetTurnRadius( const int64 &int_timestamp_search, double *R)
{
    bool R_search_ok = 0;
    int R_data_length = m_vector_turn_radius.size();
    double time_cur, time_pre, dt_t_cur, dt_t_pre, dt_t;
    double R_t;
    double timestamp_search = int_timestamp_search/1000.0;
    
    radius_rw_lock.WriterLock();
    m_call_radius_timestamp = timestamp_search;// 更新时间戳
    radius_rw_lock.Unlock();
        
    for(int i = 1; i<R_data_length; i++)
    {
       time_cur = (m_vector_turn_radius.end()-i)->timestamp;
       time_pre = (m_vector_turn_radius.end()-i-1)->timestamp;
       dt_t = time_cur - time_pre;
       dt_t_cur = time_cur - timestamp_search;
       dt_t_pre = time_pre - timestamp_search;

       if(dt_t_pre<0 && dt_t_cur>0 && dt_t>=0)
       {
           // 方法: 线性差插值
           double R_pre, R_cur, d_R;            
           R_pre = (m_vector_turn_radius.end()-i-1)->R;
           R_cur = (m_vector_turn_radius.end()-i)->R;
           d_R = R_cur - R_pre;                
           R_t = R_pre + (fabs(dt_t_pre)/fabs(dt_t))*d_R;// 线性插值                

           R_search_ok = 1;
           break;
       }
    }
    
    VLOG(VLOG_INFO)<<"DF:GetTurnRadius--"<<"R:(ms) "<<"dt_t_cur= "<<dt_t_cur*1000<<", dt_t_pre= "<<dt_t_pre*1000<<endl; 
    if(R_search_ok)
    {
        *R = R_t;
        return 0;
    }
    else
    {
        *R = 0;
        return -1;
    }
    

}


// 拟合曲线
int DataFusion::Polyfit(const cv::Mat& xy_feature, int order, std::vector<float>* lane_coeffs )
{  
    int feature_points_num = xy_feature.cols;
    std::vector<float> x(feature_points_num);
    std::vector<float> y(feature_points_num);
    cv::Mat A = cv::Mat(feature_points_num, order + 1, CV_32FC1);
    cv::Mat b = cv::Mat(feature_points_num+1, 1, CV_32FC1);

         for (int i = 0; i < feature_points_num; i++) 
        {
            x[i] = xy_feature.at<float>(0, i);
            y[i] = xy_feature.at<float>(1, i); 
    
            for (int j = 0; j <= order; j++) 
            {
                A.at<float>(i, j) = pow(y[i], j);
            }
            b.at<float>(i) = x[i];
        }
        
        cv::Mat coeffs;
        int ret = cv::solve(A, b, coeffs, CV_SVD);
        if(ret<=0)
        {    
            VLOG(VLOG_INFO)<<"cv:solve error!!!"<<endl;
            return -1;
        }
        
        for(int i=0; i<order+1; i++)
        {
            lane_coeffs->push_back(coeffs.at<float>(i,0));
        }    
        return 1;
}


float DataFusion::Raw2Degree(short raw)
{
    return (float (raw))/340 + 36.53;
}  


void DataFusion::PrintImuData(const int is_print_imu)
{
    m_is_print_imu_data = is_print_imu;
}

void DataFusion::PrintSpeedData(const int is_print_speed)
{
    m_is_print_speed_data = is_print_speed;
}

