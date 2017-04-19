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

#if defined(ANDROID)
    DEFINE_string(log_dir, "./log/", " ");
    DEFINE_int32(v, 0, " ");
#endif

int main(int argc, char *argv[])
{
    //解析
    google::ParseCommandLineFlags(&argc, &argv, true);
//    printf("log_data_addr cur: %s\n", FLAGS_log_data_addr.c_str());

    // 读入log
    ifstream infile_log("data/doing/log.txt");       // 指定log的路径
    string buffer_log;
    string data_flag;
    stringstream ss_log;
    stringstream ss_tmp;

    // 写R
    ofstream  offile_log;
    string of_log_addr = "./data/doing/radius.txt";
    bool is_save_R = false; //true; // 是否将R保存为txt

    TimeUtils f_time_counter;
    double R_cur;
    double image_timestamp;
    
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = "./log/";  
    #if defined(USE_GLOG)
//        FLAGS_v = VLOG_DEBUG; // 设置VLOG打印等级;
        FLAGS_v = 0;
    #endif

    // 进行数据融合的类
    DataFusion data_fusion;
//     data_fusion.Init();
   ImuModule::Instance().Init();

    // 清空文件内容
    offile_log.open(of_log_addr.c_str(), std::ifstream::out | std::ifstream::trunc );
    if (!offile_log.is_open() || offile_log.fail()){
        offile_log.close();
        printf("Error : failed to erase file content !");
    }
    offile_log.close();

    offile_log.open(of_log_addr.c_str());
    string log_data_t[2];
    while(!infile_log.eof()){
        getline(infile_log, buffer_log);
        ss_tmp.clear();
        ss_tmp.str(buffer_log);
        ss_tmp>>log_data_t[0]>>log_data_t[1]>>data_flag;
        ss_log.clear();
        ss_log.str(buffer_log);

        if(data_flag == "cam_frame"){
            int camera_raw_timestamp[2];
            string camera_flag, camera_add, image_index_str, image_name;
            ss_log>>camera_raw_timestamp[0]>>camera_raw_timestamp[1]>>camera_flag>>camera_add>>image_index_str;
            image_timestamp = camera_raw_timestamp[0] + camera_raw_timestamp[1]*1e-6;

            // 执行查询转弯半径
            int64_t t_1, t_2;
            int64_t image_timestamp_cur_int = (int64_t)(image_timestamp*1000);
            int r_1 = -1;
            t_1 = f_time_counter.Microseconds();// 测试运行时间
//             r_1 = data_fusion.GetTurnRadius( image_timestamp_cur_int, &R_cur);
            r_1 = ImuModule::Instance().GetTurnRadius( image_timestamp_cur_int*1000, &R_cur);
            t_2 = f_time_counter.Microseconds();
            int64_t R_cal_dt = (t_2 - t_1) ;

            if(r_1 <= 0){
                // printf("main timestamp diamatch, match state= %d, so sleep\n", r_1);
                sleep(1);
            }

            if(is_save_R){
                char buffer[100];
                if (offile_log.is_open()) {
                    sprintf(buffer, "%d %d %s %s %f\n", camera_raw_timestamp[0], camera_raw_timestamp[1],
                            camera_add.c_str(), image_index_str.c_str(), R_cur);
                    offile_log << buffer;
                }else{
                    printf("write log file is not opened!");
                }
            }
           printf("R = %f\n", R_cur);
        }
        usleep(10);
    }
    printf("radius calculate over!!\n");
    offile_log.close();
    infile_log.close();

    return 0;
}

