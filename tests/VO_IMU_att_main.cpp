
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <math.h>
#include <vector>
#include <queue>
#include <dirent.h>
#include <time.h>
#include <iomanip>
#include <stdexcept>
#include <unistd.h> // sleep header

#include "opencv2/opencv.hpp"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "common/base/stdint.h"
// #include "common/math/polyfit.h"
#include "common/base/log_level.h"
#include "common/relative_locate/relative_locate.h"
#include "common/relative_locate/bird_perspective_mapping.h"
#include "common/time/time_utils.h"

#include "data_fusion.h"
#include "datafusion_math.h"
#include "imu_module.h"
#include "visual_odometry.h"

using namespace std;
using namespace cv;
using namespace imu;

DEFINE_string(image_name, "./1.jpg", "image_name");
DEFINE_double(fu, 1482.0, "fu");
DEFINE_double(fv, 1475.874, "fv");
DEFINE_double(cu, 685.044, "cu");
DEFINE_double(cv, 360.02380, "cv");
DEFINE_double(camera_height, 1.25, "camera height mm");    // ??? mm
DEFINE_double(pitch, -0.3, "pitch angle (degree)"); // -1.8
DEFINE_double(yaw, 10.3, "yaw angle (degree)");
DEFINE_int32(image_width, 1280, "image width");
DEFINE_int32(image_height, 720, "image height");
DEFINE_double(x_start_offset, -7.0, "x start offset");
DEFINE_double(x_end_offset, 7.0, "x start offset");
DEFINE_double(y_start_offset, 1.0, "y start offset");
DEFINE_double(y_end_offset, 70.0, "y end offset");
DEFINE_double(x_res, 0.03, "x resolution");
DEFINE_double(y_res, 0.08, "y resolution");

//DEFINE_string(flagfile, "./data/doing/frame/detect.flag", " parameter address ");
void Init();

void LoadImage(cv::Mat* image, string image_name);

// 对图片进行IPM变化
void image_IPM(cv::Mat &ipm_image, cv::Mat org_image, IPMPara ipm_para);

void image_IPM_color(cv::Mat &ipm_image, cv::Mat org_image, IPMPara ipm_para);

// 获取指定路径下所有文件
string get_file_name(string file_path);

// 获取图片文件夹中所有的图片的最大最小序号
bool get_max_min_image_index(int &max_index, int &min_index, string file_path);

// 在IPM图上画车道线
void mark_IPM_radius(const IPMPara ipm_para, const double R, const float val,  cv::Mat &ipm_image );

void mark_IPM_radius_color(const IPMPara ipm_para, const double R, const Vec3b value_vec,  cv::Mat &ipm_image );

void do_get_turn_radius();

TimeUtils f_time_counter;
double g_R_cur;
double g_image_timestamp;
double vehicle_L = 2.637;

int main(int argc, char *argv[])
{   
    google::ParseCommandLineFlags(&argc, &argv, true); //解析
    google::InitGoogleLogging(argv[0]);// 初始化
    
    Init();

    // 读入log和图片路径
    string str_image_frame_add = FLAGS_jpg_data_addr; // jpg
    ifstream infile_log(FLAGS_log_data_addr.c_str());       // 指定log的路径
    string buffer_log;
    string data_flag;
    stringstream ss_log;
    stringstream ss_tmp;

    CameraPara camera_para;
    camera_para.fu = FLAGS_fu;
    camera_para.fv = FLAGS_fv;
    camera_para.cu = FLAGS_cu;
    camera_para.cv = FLAGS_cv;
    camera_para.height = FLAGS_camera_height; // m
    camera_para.pitch = FLAGS_pitch * CV_PI / 180;
    camera_para.yaw = -FLAGS_yaw * CV_PI / 180;
    camera_para.image_width = FLAGS_image_width;
    camera_para.image_height = FLAGS_image_height;
    BirdPerspectiveMapping bp_mapping;
    bp_mapping.Initialize(camera_para);

    IPMPara ipm_para;
    ipm_para.x_limits[0] = FLAGS_x_start_offset;
    ipm_para.x_limits[1] = FLAGS_x_end_offset;
    ipm_para.y_limits[0] = FLAGS_y_start_offset;
    ipm_para.y_limits[1] = FLAGS_y_end_offset;
    ipm_para.x_scale = FLAGS_x_res;
    ipm_para.y_scale = FLAGS_y_res;
    bp_mapping.GetUVLimitsFromXY(&ipm_para);

    // 进行数据融合的类
    DataFusion &data_fusion = DataFusion::Instance();
    data_fusion.StartDataFusionTask();
    
    //定义相机
	double width = 1280;
	double height = 720;
	double fx = 1430.55007;
	double fy = 1430.33280;
	double cx = 646.9; 
	double cy = 370.9;
    PinholeCamera *cam = new PinholeCamera(width, height, fx, fy, cx, cy);
	VisualOdometry vo(cam);
 
    string frame_file_name = get_file_name(str_image_frame_add); // 读取图像所在文件夹名字
    string frame_file_addr = str_image_frame_add + "/" + frame_file_name;// 获取图片的max,min index
    int max_frame_index, min_frame_index;
    get_max_min_image_index(max_frame_index, min_frame_index, frame_file_addr);

    // 外部循环控制
    int image_index_start = 1;
    int image_cal_step = 5;// 每隔多少帧计算一次
    std::cout<<"please input the image_index_step: "<<endl;
    std::cin >> image_cal_step;
    std::cout<< "the step: "<<image_cal_step<<endl;
    bool is_camera_index_mached = 0; // 是否已经从log中寻找到当前图像的匹配的时间戳
    for(int image_index = image_index_start; image_index <= max_frame_index; image_index += image_cal_step){
        is_camera_index_mached = 0;
        double log_data_t[2];
        while(!is_camera_index_mached){
            getline(infile_log, buffer_log);
            ss_tmp.clear();
            ss_tmp.str(buffer_log);
            ss_tmp>>log_data_t[0]>>log_data_t[1]>>data_flag;
            ss_log.clear();
            ss_log.str(buffer_log);
            
            if(data_flag == "gsensor"){
                // TODO:
                
            }else if(data_flag == "cam_frame"){
                double camera_raw_timestamp[2];
                string camera_flag, camera_add, image_index_str;
                string image_name;
                int log_image_index;
                ss_log>>camera_raw_timestamp[0]>>camera_raw_timestamp[1]>>camera_flag>>camera_add>>log_image_index;
                g_image_timestamp = camera_raw_timestamp[0] + camera_raw_timestamp[1]*1e-6;

                // 匹配图片的时间戳
                int pos1 = camera_add.find_last_of('_');
                int pos2 = camera_add.find_last_of('.');
                string log_str_file_name = camera_add.substr(pos1+1, pos2-1-pos1);
                if(log_str_file_name.compare(frame_file_name) == 0 && log_image_index ==  image_index){
                    is_camera_index_mached = 1;
                    VLOG(VLOG_INFO)<<"image_index: "<<image_index;

                    // 读取图片
                    char str_iamge_name_index[20]; // 新数据格式，补全8位
                    sprintf(str_iamge_name_index, "%06d", image_index);
                    image_name = frame_file_addr + "/" + str_iamge_name_index + ".png";
                    cv::Mat img(cv::imread(image_name.c_str(), 0));
                    assert(!img.empty());                    
                    vo.ProcessNewImage(img, image_index);// 处理帧
                    
                    double diff_euler[3];
                    vo.GetCurrentDiffEuler(diff_euler);
                    std::cout<<"diff_angle: roll = "<<diff_euler[0]<<" pitch = "<<diff_euler[1]<<" yaw = "<<diff_euler[2]<<std::endl;

                    cv::imshow("Road facing camera", img);        
                    // 按键跳出循环
                    while(-1 == cvWaitKey(2)){
                        usleep(10000);
                    }
                    
                    // 执行查询转弯半径
                    do_get_turn_radius();
                    
                    // color IPM
//                     cv::Mat org_image_color = cv::imread(image_name, cv::IMREAD_COLOR);
//                     cv::Mat ipm_image_color(ipm_para.height+1, ipm_para.width+1, CV_8UC3);
//                     ipm_image_color.setTo(Scalar(0));
//                     image_IPM_color(ipm_image_color, org_image_color, ipm_para);    
//                     cv::imshow("ipm", ipm_image_color);
//                     //按键事件，空格暂停，其他跳出循环
//                    int temp = cvWaitKey(100);
//                    if (temp == 32)
//                        while (cvWaitKey() == -1);
//                    else if (temp >= 0)
//                        break;
                }
            }
        }
    }

    delete cam;
    return 0;
}

void Init()
{
    FLAGS_log_dir = "./log/";
    
    // 设置VLOG打印等级
    #if defined(USE_GLOG)
        FLAGS_v = 0;
    #endif
}

void LoadImage(cv::Mat* image, string image_name)
{
    cv::Mat org_image = cv::imread(image_name, 1);
    cv::Mat channels[3];
    cv::split(org_image, &channels[0]);
    image->create(org_image.rows, org_image.cols, CV_32FC1);
    channels[0].convertTo(*image, CV_32FC1);
    *image = *image * (1.0 / 255);
}


// 获取指定路径下的文件名(主要用于获取图片所在文件夹名字)
string get_file_name(string file_path)
{
    DIR *dp;
    struct dirent *dirp;
    char *file_name_t;
    int n=0;
    const char *filePath = file_path.data();
    if((dp=opendir(filePath))==NULL)
        printf("can't open %s\n", filePath);

    while(((dirp=readdir(dp))!=NULL)){
         if((strcmp(dirp->d_name,".")==0)||(strcmp(dirp->d_name,"..")==0))
            continue;
        file_name_t = dirp->d_name;
        printf("%d: %s\n ",++n, file_name_t);
    }

    if(n == 1){
        string str_file_name(file_name_t);
        return str_file_name;
    }else{
        printf("error: too many files!!! \n");
        return 0;
    }
}

// 读取图片文件夹中所有文件最小和最大的index
bool get_max_min_image_index(int &max_index, int &min_index, string file_path)
{
    DIR *dp;
    struct dirent *dirp;
    char *file_name_t;
    bool is_first_time = 1;
    const char *filePath = file_path.data();
    if((dp=opendir(filePath))==NULL){
        printf("can't open %s", filePath);
        return 0;
    }

    while(((dirp=readdir(dp))!=NULL)){
        if((strcmp(dirp->d_name,".")==0)||(strcmp(dirp->d_name,"..")==0))
           continue;

        file_name_t = dirp->d_name;
        string str_name(file_name_t);
        int number = std::atoi( str_name.c_str());
        if(is_first_time){
            is_first_time = 0;
            max_index = number;
            min_index = number;
        }else{
            if(number > max_index)
                max_index = number;
            if(number < min_index)
                min_index = number;
        }
    }

    printf("max:%d, min:%d\n", max_index, min_index);
    return 1;
}


// 图片IPM
void image_IPM(cv::Mat &ipm_image, cv::Mat org_image, IPMPara ipm_para)
{
    for (int i = 0; i < ipm_para.height; ++i){
        int base = i * ipm_para.width;
        for (int j = 0; j < ipm_para.width; ++j){
            int offset = base + j;
            float ui = ipm_para.uv_grid.at<float>(0, offset);
            float vi = ipm_para.uv_grid.at<float>(1, offset);
            if (ui < ipm_para.u_limits[0] || ui > ipm_para.u_limits[1] || vi < ipm_para.v_limits[0] || vi > ipm_para.v_limits[1])
                continue;
            int x1 = int(ui), x2 = int(ui + 1);
            int y1 = int(vi), y2 = int(vi + 1);
            float x = ui - x1, y = vi - y1;
            float val = org_image.at<float>(y1, x1) * (1 - x) * (1-y) + org_image.at<float>(y1, x2) * x * (1-y) +
                        org_image.at<float>(y2, x1) * (1-x) * y + org_image.at<float>(y2, x2) * x * y;
            ipm_image.at<float>(i, j) = static_cast<float>(val);
        }
    }
}

// 图片IPM
void image_IPM_color(cv::Mat &ipm_image, cv::Mat org_image, IPMPara ipm_para)
{
    for (int i = 0; i < ipm_para.height; ++i){
        int base = i * ipm_para.width;
        for (int j = 0; j < ipm_para.width; ++j){
            int offset = base + j;
            float ui = ipm_para.uv_grid.at<float>(0, offset);
            float vi = ipm_para.uv_grid.at<float>(1, offset);
            if (ui < ipm_para.u_limits[0] || ui > ipm_para.u_limits[1] || vi < ipm_para.v_limits[0] || vi > ipm_para.v_limits[1])
                continue;
            int x1 = int(ui), x2 = int(ui + 1);
            int y1 = int(vi), y2 = int(vi + 1);
            float x = ui - x1, y = vi - y1;
            Vec3b val = org_image.at<Vec3b>(y1, x1) * (1 - x) * (1-y) + org_image.at<Vec3b>(y1, x2) * x * (1-y) +
                        org_image.at<Vec3b>(y2, x1) * (1-x) * y + org_image.at<Vec3b>(y2, x2) * x * y;
            ipm_image.at<Vec3b>(i, j) = static_cast<Vec3b>(val);
        }
    }
}


//// 标记 IPM 转弯半径
void mark_IPM_radius(const IPMPara ipm_para, const double R, const float val,  cv::Mat &ipm_image )
{
    /// 在IPM中标注当前lane
    std::vector<int> x(ipm_para.height+2);
    std::vector<int> y(ipm_para.height+2);
    double y_max_t, x_t;
    int u, v;

    if(R == 0)
        y_max_t = ipm_para.y_limits[1];  // IPM 横向是x
    else
         y_max_t = min(ipm_para.y_limits[1], fabs(R));

    for (double y_t = ipm_para.y_limits[0]; y_t < y_max_t; y_t+=ipm_para.y_scale) {
        if(R > 0)
            x_t = -R + sqrt(R*R - y_t*y_t);
        else if(R < 0)
            x_t = -R - sqrt(R*R - y_t*y_t);
        else
            x_t = 0;

        u = (-y_t + ipm_para.y_limits[1])/ipm_para.y_scale;
        v = (x_t + ipm_para.x_limits[1])/ipm_para.x_scale;


        if (v < 0 || v > ipm_para.width )
           continue;
        else
           ipm_image.at<float>(u, v) = val;
    }
}

// 在彩图上标注R
//// 标记 IPM 转弯半径
void mark_IPM_radius_color(const IPMPara ipm_para, const double R, const Vec3b value_vec,  cv::Mat &ipm_image )
{
    /// 在IPM中标注当前lane
    std::vector<int> x(ipm_para.height+2);
    std::vector<int> y(ipm_para.height+2);
    double y_max_t, x_t;
    int u, v;

    if(R == 0)
        y_max_t = ipm_para.y_limits[1];  // IPM 横向是x
    else
         y_max_t = min(ipm_para.y_limits[1], fabs(R));

    for (double y_t = ipm_para.y_limits[0]; y_t < y_max_t; y_t+=ipm_para.y_scale) {
        if(R > 0)
            x_t = -R + sqrt(R*R - y_t*y_t);
        else if(R < 0)
            x_t = -R - sqrt(R*R - y_t*y_t);
        else
            x_t = 0;

        u = (-y_t + ipm_para.y_limits[1])/ipm_para.y_scale;
        v = (x_t + ipm_para.x_limits[1])/ipm_para.x_scale;

        if (v < 0 || v > ipm_para.width )
           continue;
        else
           ipm_image.at<Vec3b>(u, v) = static_cast<Vec3b>(value_vec);
    }
}


// 获取转弯半径
void do_get_turn_radius()
{
    int r_1 = -1;
    int main_sleep_counter = 0; //  一次外部调用，main sleep的次数
    while(r_1<0){
        r_1 = DataFusion::Instance().GetTurnRadius( g_image_timestamp, &g_R_cur);

        if(main_sleep_counter > 0){
            printf("main timestamp dismatch conunter:%d, match state= %d, so sleep\n", main_sleep_counter, r_1);
            usleep(20000);
        }
        main_sleep_counter++;
        // 计算车轮角度
        double sin_angle, tan_angle;
        if(g_R_cur != 0){
            double tmp1 = vehicle_L/g_R_cur;
            sin_angle = asin(tmp1);
            tan_angle = atan(tmp1);
        }else{
            sin_angle = 0;
            tan_angle = 0;
        }
    }
}
