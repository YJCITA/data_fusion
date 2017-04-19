#include <dirent.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>

#include "gflags/gflags.h"
#include "common/hal/halio.h"
#include "common/hal/camctl.h"
#include "common/base/log_level.h"
#include "common/time/time_utils.h"

#include <Eigen/Dense>  
#include "data_fusion.h"
#include "datafusion_math.h"
#include "imu_module.h"
#include "turnlamp_detector.h"


using namespace imu;
using namespace Eigen;  
using namespace std; 

void GetTimeS(double* timestamp);

void TryPrintData(int sig);

void SaveImuAttData();

void SaveVn300Data( );

void SaveExData( );

void SaveRodData( );

void init();

//---------------------variable--------------------------------//
string g_att_log = "/storage/sdcard1/att.txt";  // = "./gflags.flag";
fstream g_file_log;

bool g_is_print_data = false;

MatrixXd g_acc_calibrate_sequence(6,3);// 校准的acc的哪个面

#if defined(ANDROID)
    DEFINE_string(log_dir, "./log/", " ");
    DEFINE_int32(v, 0, " ");
#endif
    
// 低通滤波
bool g_is_first_rod_acc_filter = true; // 是否是第一次进行低通滤波
double g_rod_acc_pre[3];
double g_rod_acc_timestamp_pre;
double g_rod_acc_range_scale = 8.0/32768;

int main(int argc, char *argv[])
{
    // 初始化
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);

    init();

    // 进行数据融合的类
    DataFusion &data_fusion = DataFusion::Instance();
    data_fusion.Init();

    #if !defined(DATA_FROM_LOG)
        while(1){
            //SaveVn300Data();
            //SaveExData();
            SaveRodData();
            SaveImuAttData();
            usleep(10000);
        }
    #endif
    
    g_file_log.close();
    printf("exe over\n");
    return 0;
}

void init()
{
    #if defined(USE_GLOG)
        #if defined(ANDROID)
            FLAGS_v = 0;
        #else
            FLAGS_v = 0; //VLOG_DEBUG; // 设置VLOG打印等级;
            FLAGS_log_dir = "./log/";
        #endif
    #endif

    // SIGTSTP: Ctrl+Z   SIGTINI: Ctrl+C
    (void) signal(SIGTSTP, TryPrintData);  
    
    // log
    g_file_log.open(g_att_log.c_str());
    if (!g_file_log.is_open() || g_file_log.fail())
    {
        g_file_log.close();
        printf("Error: failed to open log file !\n");
        exit(0);
    }

    
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
}

// 获取当前循环需要读取数据的时间
void GetTimeS(double* timestamp)
{
   // 读取camera imu数据
    struct timeval time_imu;
    gettimeofday(&time_imu, NULL);
    *timestamp = time_imu.tv_sec + time_imu.tv_usec*1e-6;
}

void TryPrintData(int sig)
{ 
    g_is_print_data = !g_is_print_data;
        
}

void SaveImuAttData()
{
    double timestamp;
    GetTimeS(&timestamp);
    double vehicle_pos[2], att[3], angle_z, att_gyro[3], acc_camera[3], acc_camera_pre[3], gyro_camera[3];
    int search_state = DataFusion::Instance().GetTimestampData(timestamp-0.01, vehicle_pos, att, &angle_z, att_gyro, acc_camera, gyro_camera);
    if(search_state == 1){
        // ACC to angle
        double acc_angle[3];
        acc_angle[0] = (atan2f(-acc_camera[1], -acc_camera[2]));       // Calculating pitch ACC angle
        acc_angle[1] = (atan2f(acc_camera[0], sqrtf(acc_camera[2]*acc_camera[2] + acc_camera[1]*acc_camera[1])));   //Calculating roll ACC angle    
//        printf("gyro: %5.2f %5.2f %5.2f ", gyro_camera[0]*R2D,  gyro_camera[1]*R2D,  gyro_camera[2]*R2D);
//        printf("att: %5.2f %5.2f %5.2f att_acc: %5.2f %5.2f\n", att[0]*R2D, att[1]*R2D, att[2]*R2D, acc_angle[0]*R2D, acc_angle[1]*R2D);
//        printf("att: %5.2f %5.2f %5.2f att_acc: %5.2f %5.2f\n", att[0]*R2D, att[1]*R2D, att[2]*R2D, acc_angle[0]*R2D, acc_angle[1]*R2D);
        if (g_file_log.is_open()) {
           char buffer[100];
//            sprintf(buffer, "att %f %5.2f %5.2f %5.2f\n", timestamp-0.01, att[0]*R2D, att[1]*R2D, att[2]*R2D);
//            g_file_log << buffer;
           
           memset(buffer, 0, sizeof(buffer));
           sprintf(buffer, "imu %f %f %f %f %f %f %f\n", timestamp-0.01, acc_camera[0], acc_camera[1], acc_camera[2], gyro_camera[0], gyro_camera[1], gyro_camera[2]);
           g_file_log << buffer;
           
        }

//         if(g_is_print_data)
//             printf("att: %5.2f %5.2f %5.2f att_acc: %5.2f %5.2f\n", att[0]*R2D, att[1]*R2D, att[2]*R2D, acc_angle[0]*R2D, acc_angle[1]*R2D);

    }
}


void SaveVn300Data( )
{
    ROD_DATA rod_data[20];
    double VN300_att[3];
    int read_rod_state = HalIO::Instance().ReadRodData(rod_data, 20);
    if(read_rod_state){
        memcpy(VN300_att, rod_data[read_rod_state-1].acc, sizeof(VN300_att));
        for(int i = 0; i < 3; i++)
            VN300_att[i] = rod_data[read_rod_state-1].acc[i]/100.0;
        if (g_file_log.is_open()) {
            char buffer[100];
            struct timeval time_t;
            time_t = rod_data[read_rod_state-1].tv;
            double time_rod = time_t.tv_sec + time_t.tv_usec*1e-6;
            sprintf(buffer, "VN300 %f %5.2f %5.2f %5.2f\n",time_rod, VN300_att[0], VN300_att[1], VN300_att[2]);
            g_file_log << buffer;
        }

        if(g_is_print_data)
            printf("VN300 att: %5.2f %5.2f %5.2f\n", VN300_att[0], VN300_att[1], VN300_att[2]);
    }
}

void SaveExData( )
{
    EX_DATA ex_data[20];
    double att[3];
    int read_state = HalIO::Instance().ReadExData(ex_data, 20);
    if(read_state){
        memcpy(att, ex_data[read_state-1].data, sizeof(att));
        for(int i = 0; i < 3; i++)
            att[i] = ex_data[read_state-1].data[i]/10.0;
        if (g_file_log.is_open()) {
            char buffer[100];
            struct timeval time_t;
            time_t = ex_data[read_state-1].tv;
            double time_rod = time_t.tv_sec + time_t.tv_usec*1e-6;
            sprintf(buffer, "fmu %f %5.2f %5.2f %5.2f\n",time_rod, att[0], att[1], att[2]);
            g_file_log << buffer;
        }

        if(g_is_print_data)
            printf("fmu: %5.2f %5.2f %5.2f\n", att[0], att[1], att[2]);
    }
}

void SaveRodData( )
{
    double acc_data_ned_new[3];
     
    ROD_DATA rod_data[20];
    int read_rod_state = HalIO::Instance().ReadRodData(rod_data, 20);
    if(read_rod_state){
        // 比例因子缩放 和 坐标系变换
        for(int k=0; k<3; k++){
            acc_data_ned_new[k] = rod_data[read_rod_state-1].acc[k]*g_rod_acc_range_scale*ONE_G;  // scale: fmu /100
        }
        double time_cur = rod_data[read_rod_state-1].tv.tv_sec + rod_data[read_rod_state-1].tv.tv_usec*1e-6;

        // LowpassFilter3f
        if(g_is_first_rod_acc_filter){
            memcpy(g_rod_acc_pre, acc_data_ned_new, sizeof(acc_data_ned_new));
            g_is_first_rod_acc_filter = false;
            g_rod_acc_timestamp_pre = time_cur;
        }

        double dt = time_cur - g_rod_acc_timestamp_pre;
        double acc_data_filter[3];
        LowpassFilter3f(g_rod_acc_pre, acc_data_ned_new, dt, 10, acc_data_filter);      
        
        if (g_file_log.is_open()) {
            char buffer[100];
                 sprintf(buffer, "rod %f %5.2f %5.2f %5.2f\n",time_cur, acc_data_filter[0], acc_data_filter[1], acc_data_filter[2]);
            g_file_log << buffer;
        }          
        if(g_is_print_data)
            printf("rod: %5.2f %5.2f %5.2f\n", acc_data_filter[0], acc_data_filter[1], acc_data_filter[2]);
                
        //update
        g_rod_acc_timestamp_pre = time_cur;
        memcpy(g_rod_acc_pre, acc_data_filter, sizeof(acc_data_filter));      
    }
}

