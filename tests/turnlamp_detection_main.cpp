#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <math.h>
#include <vector>
#include <queue>
#include <dirent.h>
#include <time.h>
#include <signal.h> 
#include <stdlib.h>


#include "opencv2/opencv.hpp"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "common/base/stdint.h"
#include "common/base/log_level.h"
#include "common/time/time_utils.h"

#include "data_fusion.h"
#include "datafusion_math.h"
#include "turnlamp_detector.h"

using namespace imu;

#define MACH_INIT 11
#define ROD_ACC_THRESHOLD 12
#define L_LAMP 13
#define R_LAMP 14
#define SAVE_PARAMETER 15 // 保存在线标定结果
#define READ_PARAMETER 16 // 读取在线标定结果

#define RESET_ALL 10 // 重置所有变脸

 
#define CAMERA_ACC_QUEUE_SIZE 40
#pragma pack(1)
    typedef struct {
    	unsigned short rd_index;
    	unsigned short wr_index;
    	AccData	acc_data[CAMERA_ACC_QUEUE_SIZE];
    }AccDataQueue;
#pragma pack()

//----------------------------------------------------------------------------//
void init();

// 拨杆初始化坐标对准
int CalculateRodRotationInitMatch();

void DoCommondSwitch();

int CalculateDiffAcc();

// 获取当前循环需要读取数据的时间
void GetReadAccDataTime();

// 计算左右转向灯阈值
void CalculateTurnlampThreshold();

// 判断转向灯的位置
void DiagnoseLeftRightTurnlamp();

// 保存计算结果
void SaveTurnlampResult();

void CollectLeftTurnlampAccData(int sig);  

void CollectRightTurnlampAccData(int sig);  

void CalculateRodCameraRotation(int sig);  

void CalculateRodSelfShiftThreshold(int sig);

void SaveTurnlampDetectParameter(int sig);

void ReadTurnlampDetectParameter(int sig) ;

void ResetTurnlampDetectParameter(int sig);

//----------------------------------------------------------------------------//
bool g_is_start_init_match = false;
bool g_set_rod_init_match_start = false;
bool g_start_d_rod_acc_calculate = false; // 开始rod自检测的阈值计算
int g_left_turnlamp_init_state = 0; // 开始左转向灯标定 0: 没有数据  1:  开始采集数据 2: 数据采集完毕，计算阈值
int g_right_turnlamp_init_state = 0; // 开始右转向灯标定  0: 没有数据  1:  开始采集数据 2: 数据采集完毕，计算阈值

bool g_is_camera_init_acc_data_collect_ok = false; // camera acc的初始化数据收集是否已经ok
double g_timestamp_search = 0;
int g_camera_init_match_acc_data_counter = 0;
double g_mean_init_camera_acc[3] = {0, 0, 0};

// init match
bool g_rotation_init_ok = false; // 初始化的坐标对准是否OK
int g_mean_init_camera_acc_counter = 0; // 进行初始化校正的时候camera的数据计数 
double g_diff_acc_average[3] = {0.0, 0.0, 0.0}; // 检测到可能的波动后进行绝对位置判断
bool g_calculate_new_acc_diff_mean = true; // 是否是新的diff_acc_average数据
double g_diff_acc[3];

// rod self shif setect
bool g_rod_self_shift_threshold_ok = false; // 拨杆自身的阈值计算是否ok

// 左右转向灯检测阈值计算
int g_turnlamp_detect_delay_num = 10; // 当第一次检测到拨杆可能被拨动，然后就持续检测后面的Ｎ个数据，就行比较判断当前拨杆的绝对位置
int g_turnlamp_detect_delay_counter = g_turnlamp_detect_delay_num;
double g_diff_acc_max = 100; // 最大的acc差值是2,大于这个数据不能用于进行拨杆绝对位置判断
int g_average_num_counter = 0; // 有效的用于计算均值的acc_diff

bool g_turnlamp_threshold_ok = false; // 左右拨杆的阈值是否计算ok
double g_acc_diff_left_threshold[3] = {0, 0, 0};
double g_acc_diff_right_threshold[3] = {0, 0, 0};
double g_acc_diff_middle_threshold[3] = {0, 0, 0};
double g_acc_diff_weight[3] = {1.0, 1.0, 1.0};
int g_acc_diff_key_index = -1; // 变化最大的值
bool g_is_left_threshold_update = false;
bool g_is_right_threshold_update = false;
#define DIFF_ACC_ERROR_RATIO 0.5 // 判断左右转向灯阈值的缩放系数

// 保存参数
ofstream  g_turnlamp_detect_parameter;
string g_turnlamp_detect_parameter_addr = "./parameter_turnlamp_detect.ini"; 
bool g_is_save_turnlamp_detect_parameter = false; // 是否保存参数

// 保存检测结果
ofstream  g_turnlamp_detect_log;
string g_turnlamp_detect_log_addr = "./log_turnlamp_detect.txt"; 
bool g_is_save_turnlamp_detect_log = false; // 是否保存参数
int g_turnlamp_setect_state = 5;


#if defined(ANDROID)
    DEFINE_string(log_dir, "./log/", " ");
    DEFINE_int32(v, 0, " ");
#endif

int main(int argc, char *argv[])
{
    // init
    google::InitGoogleLogging(argv[0]);
    init(); 

    // 进行数据融合的类
    DataFusion &data_fusion = DataFusion::Instance();
    data_fusion.Init();

    TurnlampDetector &m_turnlamp_detector = TurnlampDetector::Instance();
    #if !defined(DATA_FROM_LOG)
        m_turnlamp_detector.StartTurnlampDetectTaskOnline();
        printf("!!! Start Turnlamp Detect Task Online\n");
    #endif

    
    while(1){
        GetReadAccDataTime();
        DoCommondSwitch();
        int rod_shift_state = TurnlampDetector::Instance().m_rod_shift_state;
        if(rod_shift_state){
            int cal_state = CalculateDiffAcc();
            if(cal_state){
                DiagnoseLeftRightTurnlamp();
            }
        }
        CalculateTurnlampThreshold();

        SaveTurnlampResult();
        usleep(10000);
    }
    printf("over!!\n");

    return 0;
}

void init()
{
    #if defined(USE_GLOG)
        #if defined(ANDROID)
            FLAGS_v = 0;
        #else
            FLAGS_v = VLOG_DEBUG; // 设置VLOG打印等级;
            FLAGS_log_dir = "./log/";
        #endif
    #endif

    #if !defined(DATA_FROM_LOG)
        HalIO &halio = HalIO::Instance();
        // string config_file_path = "./golf.json";
        //const char config_file_path[20] = "./golf.json";
        //HalioInitInfo info;
        //int state = halio.LoadCANSignalConfig(config_file_path, &info);
        //bool res = halio.Init(&info, 1);
        bool res = halio.Init(NULL, 1);
        if (!res) {
            std::cerr << "HALIO init fail" << std::endl;
            exit(0);
        }
        halio.EnableFMU(); // 使能CAN对应的filter 

        // 记录检测数据
        // 清空文件内容
        g_turnlamp_detect_log.open(g_turnlamp_detect_log_addr.c_str());
        if (!g_turnlamp_detect_log.is_open() || g_turnlamp_detect_log.fail()){
            g_turnlamp_detect_log.close();
            printf("Error : failed to open file: %s !\n", g_turnlamp_detect_log_addr.c_str());
        }
    #endif

    // 设置PC端控制信号
    (void) signal(MACH_INIT, CalculateRodCameraRotation); 
    (void) signal(ROD_ACC_THRESHOLD, CalculateRodSelfShiftThreshold);     
    (void) signal(L_LAMP, CollectLeftTurnlampAccData);  
    (void) signal(R_LAMP, CollectRightTurnlampAccData); 
    (void) signal(SAVE_PARAMETER, SaveTurnlampDetectParameter); 
    (void) signal(READ_PARAMETER, ReadTurnlampDetectParameter); 
    (void) signal(RESET_ALL, ResetTurnlampDetectParameter); 
    
}

// 拨杆初始化坐标对准
// 1:对准结束
// 0: 正在对准
int CalculateRodRotationInitMatch()
{
    if(!g_is_camera_init_acc_data_collect_ok){ // 收集camera acc的数据
        usleep(20000);
        double vehicle_pos[2], att[3], angle_z, att_gyro[3], acc_camera[3], gyro_camera[3];

        int search_state = DataFusion::Instance().GetTimestampData(g_timestamp_search, vehicle_pos, att, &angle_z, att_gyro, acc_camera, gyro_camera);
        if(search_state == 1){
            if(g_camera_init_match_acc_data_counter++ < CAMERA_ACC_QUEUE_SIZE){
                for(int k=0; k<3; k++)
                    g_mean_init_camera_acc[k] += acc_camera[k];
                printf("index: %d amera acc data: %f %f %f\n", g_camera_init_match_acc_data_counter, acc_camera[0], acc_camera[1], acc_camera[2]);
            }else{
                for(int k=0; k<3; k++)
                    g_mean_init_camera_acc[k] = g_mean_init_camera_acc[k]/CAMERA_ACC_QUEUE_SIZE;
                g_is_camera_init_acc_data_collect_ok = true;
            }
        }else{
            return -1; 
        }
    }else{
        // camera acc数据已经ok，判断rod的数据
        bool is_rod_init_acc_data_ok = TurnlampDetector::Instance().m_is_init_road_acc_collect_ok;
        if(is_rod_init_acc_data_ok){
            TurnlampDetector::Instance().CalculateRotationFmu2Camera(TurnlampDetector::Instance().m_mean_init_rod_acc, g_mean_init_camera_acc);
            g_is_start_init_match = false;
            g_rotation_init_ok = true;
            printf("!!!!rod acc init data = %f %f %f\n", TurnlampDetector::Instance().m_mean_init_rod_acc[0], TurnlampDetector::Instance().m_mean_init_rod_acc[1], TurnlampDetector::Instance().m_mean_init_rod_acc[2]);
            printf("!!!!camera acc init data = %f %f %f\n", g_mean_init_camera_acc[0], g_mean_init_camera_acc[1], g_mean_init_camera_acc[2]);

            // reset state for next run
            g_is_camera_init_acc_data_collect_ok = false;
            g_camera_init_match_acc_data_counter = 0;
            memset(g_mean_init_camera_acc, 0 ,sizeof(*g_mean_init_camera_acc));
            TurnlampDetector::Instance().ResetInitMatchState();

            return 1;
        }else{
            usleep(100000);
        }
    }
    return 0; 
}

// 执行PC发出的指令
void DoCommondSwitch()
{
     // 初始化对准
    // 开启rod init match数据采集
    if(g_set_rod_init_match_start){
        TurnlampDetector::Instance().SartRotationInitDataCollect();
        g_set_rod_init_match_start = false;
    }

    // 开始拨杆拨动自检测
    if(g_start_d_rod_acc_calculate){
        TurnlampDetector::Instance().StartCalculateRodAccThreshold();
        g_start_d_rod_acc_calculate = false;
    }

    if(g_is_start_init_match){
        CalculateRodRotationInitMatch( );   
    }
}

// 计算rod和camera的diff acc
int CalculateDiffAcc()
{
    double vehicle_pos[2], att[3], angle_z, att_gyro[3], acc_camera[3], gyro_camera[3];
    // 拨杆可能被重新波动，重新计算均值
    g_turnlamp_detect_delay_counter = 0;
    g_average_num_counter = 0;
    memset(g_diff_acc_average, 0 , sizeof(g_diff_acc_average));
    g_calculate_new_acc_diff_mean = true;
    
    while(g_calculate_new_acc_diff_mean){
        // 在计算diff均值的过程中，一旦拨杆检测到可能被拨动，则清零重新判断
        if(TurnlampDetector::Instance().m_rod_shift_state){
             // 拨杆可能被重新波动，重新计算均值
            g_turnlamp_detect_delay_counter = 0;
            g_average_num_counter = 0;
            memset(g_diff_acc_average, 0 , sizeof(g_diff_acc_average));
            g_calculate_new_acc_diff_mean = true;

            // 为了第一个能读到数据，增加延时，错过前面波动的数据
            usleep(50000);
    
        }else{
            // 分析检测到可能的波动之后 对拨杆的绝对位置进行判断
            if(g_turnlamp_detect_delay_counter < g_turnlamp_detect_delay_num){
                StructImuData rod_acc_data;
                TurnlampDetector::Instance().GetRodAccData(&rod_acc_data);
                usleep(10000); // 为了获取匹配的数据
                int search_state = DataFusion::Instance().GetTimestampData(rod_acc_data.timestamp, vehicle_pos, att, &angle_z, att_gyro, acc_camera, gyro_camera);
                if(search_state == 1){
                    printf("camera acc data: %f %f %f\n", acc_camera[0], acc_camera[1], acc_camera[2] );
                    for(int i=0; i<3; i++ )
                        g_diff_acc[i] = rod_acc_data.acc[i] - acc_camera[i];

                    // 在校准之前，设置比较大的值让检测数据都可以通过
                    double diff_acc_max = 100;
                    if(g_turnlamp_threshold_ok)
                        diff_acc_max = fabs(g_acc_diff_left_threshold[g_acc_diff_key_index]) + fabs(g_acc_diff_right_threshold[g_acc_diff_key_index]);

                    if(fabs(g_diff_acc[0])*g_acc_diff_weight[0] < diff_acc_max && fabs(g_diff_acc[1])*g_acc_diff_weight[1] < diff_acc_max && fabs(g_diff_acc[2])*g_acc_diff_weight[2] < diff_acc_max){
                         for(int i=0; i<3; i++ )
                            g_diff_acc_average[i] += g_diff_acc[i]; // 计算均值
                         g_average_num_counter++;
                    }
                    printf("rod acc data: %f %f %f\n", rod_acc_data.acc[0], rod_acc_data.acc[1], rod_acc_data.acc[2] );
//                    printf("camera acc data: %f %f %f\n", acc_camera[0], acc_camera[1], acc_camera[2] );
                    printf("Rod&Camera_diff_acc--loop index: %03d, value index: %03d, data: %f %f %f\n", g_turnlamp_detect_delay_counter, g_average_num_counter, g_diff_acc[0], g_diff_acc[1], g_diff_acc[2] );
                }else{
                    printf("!!! no new camera acc data\n");
                }
                g_turnlamp_detect_delay_counter += 1;
            }else {
                if(g_average_num_counter>=3){ // 打印均值
                    for(int i=0; i<3; i++ )
                        g_diff_acc_average[i] = g_diff_acc_average[i]/g_average_num_counter;
                }else{
                    // 没有足够符合规则的数据
                    memset(g_diff_acc_average, 0, sizeof(g_diff_acc_average));
                    printf("!!!! no enough data to calculate diff_acc_average\n ");
                    return -1;                    
                }
                g_calculate_new_acc_diff_mean = false; // 结束这次的计算

                VLOG(VLOG_INFO)<<"Main--"<<"average num: "<<g_average_num_counter<<endl;
                VLOG(VLOG_INFO)<<"Main--"<<"mean Fmu&Camera_diff_acc = "<<g_diff_acc_average[0]<<", "<<g_diff_acc_average[1]<<", "<<g_diff_acc_average[2]<<endl;
                printf("mean nums = %02d, g_diff_acc = %f %f %f\n", g_average_num_counter, g_diff_acc_average[0], g_diff_acc_average[1], g_diff_acc_average[2]);
                
                return 1;
            }

            // 为了防止无限循环
            if(g_turnlamp_detect_delay_counter > g_turnlamp_detect_delay_num*2){
                g_calculate_new_acc_diff_mean = false;
            }
            usleep(20000);
        }        
    }
    return 0;
}

// 获取当前循环需要读取数据的时间
void GetReadAccDataTime()
{
       // 读取camera imu数据
    #if defined(DATA_FROM_LOG)
        TurnlampDetector::Instance().RunDetectTurnlampOffline();
        double R_cur; // 在读取log日志的时候 推进data_fusion的时间戳;
        g_timestamp_search = TurnlampDetector::Instance().m_rod_imu_data.timestamp;
        DataFusion::Instance().GetTurnRadius( (int64_t)(g_timestamp_search*1000), &R_cur);
    #else
        struct timeval time_imu;
        gettimeofday(&time_imu, NULL);
        g_timestamp_search = time_imu.tv_sec + time_imu.tv_usec*1e-6;
    #endif
}

void CalculateTurnlampThreshold()
{
    // 计算左右转向灯的阈值
    if(g_left_turnlamp_init_state == 1 || g_right_turnlamp_init_state == 1){
        int cal_state = CalculateDiffAcc();
        printf("run CalculateTurnlampThreshold, cal_state=%d\n", cal_state);
        
        if(cal_state){
            if(g_left_turnlamp_init_state == 1){
                memcpy(g_acc_diff_left_threshold, g_diff_acc_average, sizeof(g_acc_diff_left_threshold));
                g_left_turnlamp_init_state = 2;
                printf("temp g_acc_diff_left_threshold: %f %f %f\n", g_acc_diff_left_threshold[0], g_acc_diff_left_threshold[1], g_acc_diff_left_threshold[2]);
            }else if(g_right_turnlamp_init_state == 1){
                memcpy(g_acc_diff_right_threshold, g_diff_acc_average, sizeof(g_acc_diff_right_threshold));
                g_right_turnlamp_init_state = 2;
                printf("temp g_acc_diff_right_threshold: %f %f %f\n", g_acc_diff_right_threshold[0], g_acc_diff_right_threshold[1], g_acc_diff_right_threshold[2]);
            }
        }
    
        if(g_left_turnlamp_init_state ==2 && g_right_turnlamp_init_state != 2){
            printf("!!!!!! please do the right turnlamp threshold init\n");
        }else if(g_left_turnlamp_init_state !=2 && g_right_turnlamp_init_state == 2){
            printf("!!!!!! please do the left turnlamp threshold init\n");
        }else if(g_left_turnlamp_init_state ==2 && g_right_turnlamp_init_state == 2){
            double dacc_abs_t[3];
            for(int i=0; i<3; i++)
                dacc_abs_t[i] = fabs(g_acc_diff_left_threshold[i] - g_acc_diff_right_threshold[i]);
    
            // 挑选最大的权值对应的轴
            double max_threshold = max(dacc_abs_t[0], max(dacc_abs_t[1], dacc_abs_t[2]));
            for(int i=0; i<3; i++){
                if(max_threshold == dacc_abs_t[i]){
                    g_acc_diff_weight[i] = 1;
                    g_acc_diff_key_index = i;
                }else{
                    g_acc_diff_weight[i] = 0;
                }
            }
            g_turnlamp_threshold_ok = true;
            // reset state
            g_left_turnlamp_init_state = 0;
            g_right_turnlamp_init_state = 0;

            printf("!!!!---new g_acc_diff_left_threshold: %f %f %f\n", g_acc_diff_left_threshold[0], g_acc_diff_left_threshold[1], g_acc_diff_left_threshold[2]);
            printf("!!!!---new g_acc_diff_right_threshold: %f %f %f\n", g_acc_diff_right_threshold[0], g_acc_diff_right_threshold[1], g_acc_diff_right_threshold[2]);
            printf("!!!!---new g_acc_diff_weight: %f %f %f\n", g_acc_diff_weight[0], g_acc_diff_weight[1], g_acc_diff_weight[2]);
        }
    }
    
}

// 判断是左还是右转向
void DiagnoseLeftRightTurnlamp()
{
    if(g_rotation_init_ok && g_rod_self_shift_threshold_ok && g_turnlamp_threshold_ok){
        //根据所处的区间进行判断
        if(g_acc_diff_left_threshold[g_acc_diff_key_index] >= g_acc_diff_right_threshold[g_acc_diff_key_index]){
            // 分布情况: right|middle|left
            if(g_diff_acc_average[g_acc_diff_key_index] > DIFF_ACC_ERROR_RATIO*g_acc_diff_left_threshold[g_acc_diff_key_index]){
                g_turnlamp_setect_state = 1;
                printf("!!!!------LEFT------!!!!!\n");
            }else if(g_diff_acc_average[g_acc_diff_key_index] < DIFF_ACC_ERROR_RATIO*g_acc_diff_right_threshold[g_acc_diff_key_index]){
                g_turnlamp_setect_state = -1;
                printf("!!!!------RIGHT------!!!!!\n");
            }else{
                g_turnlamp_setect_state = 0;
                printf("!!!!------Middle------!!!!!\n");
            }
        }else if(g_acc_diff_left_threshold[g_acc_diff_key_index] < g_acc_diff_left_threshold[g_acc_diff_key_index]){
            // 分布情况: left|middle|right
            if(g_diff_acc_average[g_acc_diff_key_index] < DIFF_ACC_ERROR_RATIO*g_acc_diff_left_threshold[g_acc_diff_key_index]){
                g_turnlamp_setect_state = 1;
                printf("!!!!------LEFT------!!!!!\n");
            }else if(g_diff_acc_average[g_acc_diff_key_index] > DIFF_ACC_ERROR_RATIO*g_acc_diff_right_threshold[g_acc_diff_key_index]){
                g_turnlamp_setect_state = -1;
                printf("!!!!------RIGHT------!!!!!\n");
            }else{
                g_turnlamp_setect_state = 0;
                printf("!!!!------Middle------!!!!!\n");
            }
        }

        // 基于差值判断  这个不是很合理
        // 判断left or right
//        double d_left[3], d_right[3], d_middle[3];
//        double d_left_key, d_right_key;
//        d_left_key = g_diff_acc_average[g_acc_diff_key_index] - g_acc_diff_left_threshold[g_acc_diff_key_index];
//        d_right_key = g_diff_acc_average[g_acc_diff_key_index] - g_acc_diff_right_threshold[g_acc_diff_key_index];
//        printf("d_left_key = %f, d_right_key = %f\n", d_left_key, d_right_key);
//        
//        if(fabs(d_left_key) <= DIFF_ACC_ERROR_RATIO*fabs(g_acc_diff_left_threshold[g_acc_diff_key_index])){
//            printf("!!!!------LEFT------!!!!!\n");
//        }else if(fabs(d_right_key) <= DIFF_ACC_ERROR_RATIO*fabs(g_acc_diff_right_threshold[g_acc_diff_key_index])){
//            printf("!!!!------RIGHT------!!!!!\n");
//        }else{
//            printf("!!!!------Middle------!!!!!\n");
//        }
    }
}

void CollectLeftTurnlampAccData(int sig)  
{  
    g_left_turnlamp_init_state = 1;
    printf("start left turnlamp collect !!!!\n");    
}  

void CollectRightTurnlampAccData(int sig)  
{  
    g_right_turnlamp_init_state = 1;
    printf("start right turnlamp collect !!!!\n");    
}  

void CalculateRodCameraRotation(int sig)  
{  
    g_is_start_init_match = true;
    g_set_rod_init_match_start = true;
    printf("start rod init match !!!!\n");    
} 


void CalculateRodSelfShiftThreshold(int sig)  
{  
    g_start_d_rod_acc_calculate = true;
    g_rod_self_shift_threshold_ok = true;
    printf("start rod acc calculate !!!!\n");    
} 

void ResetTurnlampDetectParameter(int sig)  
{   
    printf("try to reset ruenlamp detection parameter!!!\n");
    //1. init rotation
    double R[3][3];
    memset(R, 0 , sizeof(R));
    R[0][0] = 1.0;
    R[1][1] = 1.0;
    R[2][2] = 1.0;    
    TurnlampDetector::Instance().SetRod2CameraRotation(R);
    g_rotation_init_ok = false;
    
    //2. rod self detect threshold
    double rod_self_detect_threshold_t[3], rod_self_detect_weight_t[3];
    rod_self_detect_threshold_t[0] = 3.0;
    rod_self_detect_threshold_t[1] = 3.0;
    rod_self_detect_threshold_t[2] = 3.0;

    rod_self_detect_weight_t[0] = 1;
    rod_self_detect_weight_t[1] = 1;
    rod_self_detect_weight_t[2] = 1;
    
    TurnlampDetector::Instance().SetRodSelfDetectThreshold(rod_self_detect_threshold_t, rod_self_detect_weight_t);
    g_rod_self_shift_threshold_ok = false;

    //3.diff acc
    memset(g_acc_diff_left_threshold, 0.0, sizeof(g_acc_diff_left_threshold));
    memset(g_acc_diff_right_threshold, 0.0, sizeof(g_acc_diff_right_threshold));
    memset(g_acc_diff_weight, 0.0, sizeof(g_acc_diff_weight));
    g_turnlamp_threshold_ok = false;
    
} 

void SaveTurnlampDetectParameter(int sig)  
{  
    if(g_rotation_init_ok && g_rod_self_shift_threshold_ok && g_turnlamp_threshold_ok){
        printf("try to save turnlamp detect parameter...\n");
        ofstream  offile_turnlamp_detect_parameter;

        offile_turnlamp_detect_parameter.open(g_turnlamp_detect_parameter_addr.c_str(), std::ifstream::out | std::ifstream::trunc );
        if (!offile_turnlamp_detect_parameter.is_open() || offile_turnlamp_detect_parameter.fail()){
            offile_turnlamp_detect_parameter.close();
            printf("Error : failed to erase: %s !", g_turnlamp_detect_parameter_addr.c_str());
        }
        offile_turnlamp_detect_parameter.close();
        
        offile_turnlamp_detect_parameter.open(g_turnlamp_detect_parameter_addr.c_str());  
        if(offile_turnlamp_detect_parameter.is_open()){
            char buffer[200];
            double R_rod2camera[3][3];
            TurnlampDetector::Instance().GetRod2CameraRotation(R_rod2camera);
            memset(buffer, 0, sizeof(buffer));
            sprintf(buffer, "init_rotation_rod2camera %f %f %f %f %f %f %f %f %f\n", R_rod2camera[0][0], R_rod2camera[0][1], R_rod2camera[0][2], 
                R_rod2camera[1][0], R_rod2camera[1][1], R_rod2camera[1][2], R_rod2camera[2][0], R_rod2camera[2][1], R_rod2camera[2][2]);
            offile_turnlamp_detect_parameter << buffer; 

            memset(buffer, 0, sizeof(buffer));
            double rod_self_detect_threshold[3], rod_self_detect_threshold_weight[3];
            TurnlampDetector::Instance().GetRodSelfDetectThreshold(rod_self_detect_threshold, rod_self_detect_threshold_weight);
            sprintf(buffer, "rod_self_detect_threshold %f %f %f\n", rod_self_detect_threshold[0], rod_self_detect_threshold[1], rod_self_detect_threshold[2]);
            offile_turnlamp_detect_parameter << buffer; 

            memset(buffer, 0, sizeof(buffer));
            sprintf(buffer, "rod_self_detect_weight %f %f %f\n", rod_self_detect_threshold_weight[0], rod_self_detect_threshold_weight[1], rod_self_detect_threshold_weight[2]);
            offile_turnlamp_detect_parameter << buffer; 

            memset(buffer, 0, sizeof(buffer));
            sprintf(buffer, "turnlamp_left_threshold %f %f %f\n", g_acc_diff_left_threshold[0], g_acc_diff_left_threshold[1], g_acc_diff_left_threshold[2]);
            offile_turnlamp_detect_parameter << buffer; 

            memset(buffer, 0, sizeof(buffer));
            sprintf(buffer, "turnlamp_right_threshold %f %f %f\n", g_acc_diff_right_threshold[0], g_acc_diff_right_threshold[1], g_acc_diff_right_threshold[2]);
            offile_turnlamp_detect_parameter << buffer; 

            memset(buffer, 0, sizeof(buffer));
            sprintf(buffer, "turnlamp_detect_weight %f %f %f\n", g_acc_diff_weight[0], g_acc_diff_weight[1], g_acc_diff_weight[2]);
            offile_turnlamp_detect_parameter << buffer; 

            printf("save turnlamp detect parameter succcess!!\n");
        }else{
            printf("write turnlamp detect parameter error, the parameter file cannot opened!!!\n");
        }
    }else{
        printf("---!!!!---turn lamp init is not over, rotation_init = %d,  rod_self_shift_threshold = %d, turnlamp_threshold = %d\n",
                g_rotation_init_ok, g_rod_self_shift_threshold_ok, g_turnlamp_threshold_ok);
    } 
}


void ReadTurnlampDetectParameter(int sig)  
{  
    printf("try read turnlamp detect parameter...\n");
    ifstream infile_turnlamp;
    string buffer_log, data_flag;
    stringstream ss_log;
    stringstream ss_tmp;
    double log_data[3];

    double rod_self_detect_threshold[3], rod_self_detect_weight[3];
    bool is_new_rod_self_detect_threshold = false;
    bool is_rod_self_detect_weight = false;
        
    infile_turnlamp.open(g_turnlamp_detect_parameter_addr.c_str()); // ifstream
    if(infile_turnlamp.is_open()){
        while(!infile_turnlamp.eof()){
            getline(infile_turnlamp, buffer_log);
            ss_tmp.clear();
            ss_tmp.str(buffer_log);
            ss_tmp>>data_flag>>log_data[0]>>log_data[1]>>log_data[2];
            ss_log.clear();
            ss_log.str(buffer_log);

            if(data_flag == "init_rotation_rod2camera"){
                double R[3][3];
                ss_log>>data_flag>>R[0][0]>>R[0][1]>>R[0][2]>>R[1][0]>>R[1][1]>>R[1][2]>>R[2][0]>>R[2][1]>>R[2][2];
                TurnlampDetector::Instance().SetRod2CameraRotation(R);
                g_rotation_init_ok = true;
            }else if(data_flag == "rod_self_detect_threshold"){
                memcpy(rod_self_detect_threshold, log_data, sizeof(rod_self_detect_threshold));
                is_new_rod_self_detect_threshold = true;
            }else if(data_flag == "rod_self_detect_weight"){
                memcpy(rod_self_detect_weight, log_data, sizeof(rod_self_detect_weight));
                is_rod_self_detect_weight = true;
            }else if(data_flag == "turnlamp_left_threshold"){
                memcpy(g_acc_diff_left_threshold, log_data, sizeof(g_acc_diff_left_threshold));
                printf("set new g_acc_diff_left_threshold: %f %f %f\n", g_acc_diff_left_threshold[0], g_acc_diff_left_threshold[1], g_acc_diff_left_threshold[2]);
            }else if(data_flag == "turnlamp_right_threshold"){
                memcpy(g_acc_diff_right_threshold, log_data, sizeof(g_acc_diff_right_threshold));
                printf("set new g_acc_diff_right_threshold: %f %f %f\n", g_acc_diff_right_threshold[0], g_acc_diff_right_threshold[1], g_acc_diff_right_threshold[2]);
            }else if(data_flag == "turnlamp_detect_weight"){
                memcpy(g_acc_diff_weight, log_data, sizeof(g_acc_diff_weight));

                // 挑选最大的权值对应的轴
                for(int i=0; i<3; i++){
                    if(g_acc_diff_weight[i] == 1)
                        g_acc_diff_key_index = i;
                }                    
                g_turnlamp_threshold_ok = true;
                printf("set new g_acc_diff_weight: %f %f %f\n", g_acc_diff_weight[0], g_acc_diff_weight[1], g_acc_diff_weight[2]);
            }

            if(is_new_rod_self_detect_threshold && is_rod_self_detect_weight){
                TurnlampDetector::Instance().SetRodSelfDetectThreshold(rod_self_detect_threshold, rod_self_detect_weight);
                is_new_rod_self_detect_threshold = false;
                is_rod_self_detect_weight = false;
                g_rod_self_shift_threshold_ok = true;
            }
        }
    }
}


void SaveTurnlampResult()
{
    if(g_turnlamp_detect_log.is_open()){
        char buffer[20];
        memset(buffer, 0, sizeof(buffer));
        // g_turnlamp_setect_state
        int CAN_turnlamp = HalIO::Instance().GetTurnLamp();
        if(CAN_turnlamp == 2)
            CAN_turnlamp = -1;

        struct timeval time_imu;
        gettimeofday(&time_imu, NULL);
        double timestamp_t= time_imu.tv_sec + time_imu.tv_usec*1e-6;
        
        sprintf(buffer, "%f turnlamp_can %d turnlamp_detect %d\n", timestamp_t, g_turnlamp_setect_state, CAN_turnlamp);
        g_turnlamp_detect_log << buffer; 
    }
}

