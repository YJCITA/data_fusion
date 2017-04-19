#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <math.h>
#include <vector>
#include <queue>
#include <dirent.h>
#include <time.h>

#include "opencv2/opencv.hpp"
#include "gflags/gflags.h"
#include "common/base/stdint.h"
#include "common/base/log_level.h"
#include "common/time/time_utils.h"

#include "data_fusion.h"
#include "datafusion_math.h"
#include "imu_module.h"

using namespace imu;

double GetTimeS( );

#if defined(ANDROID)
    DEFINE_string(log_dir, "./log/", " ");
    DEFINE_int32(v, 0, " ");
#endif

int main(int argc, char *argv[])
{
    double R_cur;    
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = "./log/";  
    #if defined(USE_GLOG)
        FLAGS_v = 0;
    #endif

    #if !defined(DATA_FROM_LOG)
        HalIO &halio = HalIO::Instance();
        string config_file_path = "./golf.json";
        HalioInitInfo info;
        int state = LoadCANSignalConfig(config_file_path.c_str(), &info);
        bool res = halio.Init(&info, 1);
//         bool res = halio.Init(NULL, 1);
        if (!res) {
            std::cerr << "HALIO init fail" << std::endl;
            exit(0);
        }
        
        halio.EnableFMU(); // 使能CAN对应的filter 
    #endif
    // 进行数据融合的类
    //DataFusion data_fusion;
    ImuModule::Instance().Init();

    while(1){
            // 执行查询转弯半径
            double image_timestamp = GetTimeS()*1000; // ms
            int64_t image_timestamp_cur_int = (int64_t)(image_timestamp*1000);
            int r_1 = -1;
            //r_1 = data_fusion.GetTurnRadius( image_timestamp_cur_int, &R_cur);
            r_1 = ImuModule::Instance().GetTurnRadius( image_timestamp_cur_int*1000, &R_cur);
            if(r_1 == 1){
                printf("R = %f\n", R_cur);
            }
           
        }
        usleep(100000);
    return 0;
}

// 获取当前循环需要读取数据的时间
double GetTimeS( )
{
   // 读取camera imu数据
    struct timeval time_imu;
    gettimeofday(&time_imu, NULL);
    double timestamp = time_imu.tv_sec + time_imu.tv_usec*1e-6;
    return timestamp;
}
