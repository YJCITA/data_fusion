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
#include "common/math/polyfit.h"
#include "common/base/log_level.h"
#include "common/relative_locate/relative_locate.h"
#include "common/relative_locate/bird_perspective_mapping.h"
#include "common/time/time_utils.h"

#include "data_fusion.h"
#include "datafusion_math.h"
#include "imu_module.h"

using namespace std;
using namespace imu;

DEFINE_string(image_name, "./1.jpg", "image_name");
DEFINE_double(fu, 1506.64297, "fu");
DEFINE_double(fv, 1504.18761, "fv");
DEFINE_double(cu, 664.30351, "cu");
DEFINE_double(cv, 340.94998, "cv");
DEFINE_double(camera_height, 1.2, "camera height mm");    // ??? mm
DEFINE_double(pitch, -0.5, "pitch angle (degree)"); // -1.8
DEFINE_double(yaw, 0.0, "yaw angle (degree)");
DEFINE_int32(image_width, 1280, "image width");
DEFINE_int32(image_height, 720, "image height");
DEFINE_double(x_start_offset, -7.0, "x start offset");
DEFINE_double(x_end_offset, 7.0, "x start offset");
DEFINE_double(y_start_offset, 1.0, "y start offset");
DEFINE_double(y_end_offset, 70.0, "y end offset");
DEFINE_double(x_res, 0.04, "x resolution");
DEFINE_double(y_res, 0.1, "y resolution");

DEFINE_string(log_base_addr, "data/doing/", "log lane address");// 加载文件的基础路径

// 读入log
ifstream infile_log; //("data/doing/log.txt");       // 指定log的路径
string buffer_log;
string data_flag;
stringstream ss_log;
stringstream ss_tmp;

/// lane标注数据
ifstream infile_lane; //("data/doing/lane_data.txt");       // 指定车道线标注结果的路径
int m_order = 2;
int lane_num = 2;
int pts_num = 8;
int lane_index = 0; // 车道线的序号
int uv_feature[2][16]; // 2: XY 16: 左右每条车道8个点
string buffer_lane;
stringstream ss_lane;

cv::Mat uv_feature_pts;
cv::Mat xy_feature;
cv::Mat xy_feature_pre = cv::Mat::zeros(2, pts_num, CV_32FC1);; // 上一帧的中车道线的特征点
cv::Mat lane_coeffs = cv::Mat::zeros(m_order+1, lane_num, CV_32FC1);
cv::Mat lane_coeffs_pre = cv::Mat::zeros(m_order+1, lane_num, CV_32FC1);
cv::Mat lane_coeffs_predict = cv::Mat::zeros(m_order+1, lane_num, CV_32FC1);
double image_timestamp;
double image_timestamp_pre;
bool is_first_lane_predict = 1;

time_t  time_predict1,  time_predict2;

void LoadImage(cv::Mat* image, string image_name);

// 对图片进行IPM变化
void image_IPM(cv::Mat &ipm_image, cv::Mat org_image, IPMPara ipm_para);

// 拟合曲线
int polyfit_vector(std::vector<float>* lane_coeffs, std::vector<cv::Point2f>& vector_feature, int order );

// 拟合曲线
int polyfit1(std::vector<float>* lane_coeffs, const cv::Mat xy_feature, int order );

// 获取指定路径下所有文件
string get_file_name(string file_path);

// 获取图片文件夹中所有的图片的最大最小序号
bool get_max_min_image_index(int &max_index, int &min_index, string file_path);

// 在IPM图上画车道线
void mark_IPM_lane(cv::Mat &ipm_image, const cv::Mat lane_coeffs, const IPMPara ipm_para, const float lane_color_value);

// 车道线预测
void do_predict_feature(DataFusion &data_fusion );


TimeUtils f_time_counter;

// gflog
DEFINE_double(d_test, -1, "imu gyro bias z ");
//DEFINE_string(fileflag, "./imu.flag", "imu gyro bias z ");

int main(int argc, char *argv[])
{
    //解析
    google::ParseCommandLineFlags(&argc, &argv, true);
//    printf("log_data_addr cur: %s\n", FLAGS_log_data_addr.c_str());
//
//    printf("FLAG: gyro_A0= %f, %f, %f\n", FLAGS_gyro_bias_x, FLAGS_gyro_bias_y, FLAGS_gyro_bias_z);
//    printf("FLAG: d_test= %f\n", FLAGS_d_test);

        // 初始化
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = "./log/";

    CameraPara camera_para;
    camera_para.fu = FLAGS_fu;
    camera_para.fv = FLAGS_fv;
    camera_para.cu = FLAGS_cu;
    camera_para.cv = FLAGS_cv;
    camera_para.height = FLAGS_camera_height; // m
    camera_para.pitch = FLAGS_pitch * CV_PI / 180;
    camera_para.yaw = FLAGS_yaw * CV_PI / 180;
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

    // 设置VLOG打印等级
    #if defined(USE_GLOG)
        FLAGS_v = 0; // VLOG_DEBUG;
    #endif

// 初始化融合函数
    DataFusion data_fusion;
    data_fusion.Init();
//    ImuModule::Instance().StartDataFusionTask();

// 本地利用标注的数据测试
    string str_image_frame_add = FLAGS_log_base_addr + "frame/";
    string frame_file_name = get_file_name(str_image_frame_add); // 读取图像所在文件夹名字
    string frame_file_addr = str_image_frame_add + frame_file_name;// 获取图片的max,min index

    // log.txt
    string str_log_add = FLAGS_log_base_addr + "log.txt";
    infile_log.open(str_log_add.c_str());
    if (!infile_log.is_open() || infile_log.fail()){
        infile_log.close();
        printf("Error : failed to open infile_log(%s) file!\n", str_log_add.c_str());
        return -1;
    }

    string str_lane_add = FLAGS_log_base_addr + "lane_data.txt";
    infile_lane.open(str_lane_add.c_str());
    if (!infile_lane.is_open() || infile_lane.fail()){
        infile_lane.close();
        printf("Error : failed to open infile_lane(%s) file!\n", str_lane_add.c_str());
        return -1;
    }

    int max_frame_index, min_frame_index;
    get_max_min_image_index(max_frame_index, min_frame_index, frame_file_addr);

    // 外部lane循环控制
    int image_cal_step = 4;// 每隔多少帧计算一次车道线预测
    bool is_lane_match_image = 0;
    bool is_camera_index_mached = 0; // 是否已经从log中寻找到当前图像的匹配的时间戳
    for(int image_index = min_frame_index+5; image_index <= max_frame_index; image_index += image_cal_step){
        is_camera_index_mached = 0;
        double log_data_t[2];
        while(!is_camera_index_mached){
            getline(infile_log, buffer_log);
            ss_tmp.clear();
            ss_tmp.str(buffer_log);
            ss_tmp>>log_data_t[0]>>log_data_t[1]>>data_flag;
            ss_log.clear();
            ss_log.str(buffer_log);

            if(data_flag == "cam_frame"){
                double camera_raw_timestamp[2];
                string camera_flag, camera_add, image_index_str;
                string image_name;
                int log_image_index;
                ss_log>>camera_raw_timestamp[0]>>camera_raw_timestamp[1]>>camera_flag>>camera_add>>log_image_index;
                image_timestamp = camera_raw_timestamp[0] + camera_raw_timestamp[1]*1e-6;

                // 匹配图片的时间戳
                int pos1 = camera_add.find_last_of('_');
                int pos2 = camera_add.find_last_of('.');
                string log_str_file_name = camera_add.substr(pos1+1, pos2-1-pos1);

                if(log_str_file_name.compare(frame_file_name) == 0 && log_image_index ==  image_index){
                    // 文件名和index已经匹配
                    is_camera_index_mached = 1;

                    VLOG(VLOG_INFO)<<"image_index: "<<image_index;
                    // 查找匹配的车道线标注数据
                    is_lane_match_image = 0; // 进入查找匹配的lane
                    while(!is_lane_match_image){
                        getline(infile_lane, buffer_lane);
                        ss_lane.clear();
                        ss_lane.str(buffer_lane);
                        ss_lane>>lane_index
                                >>uv_feature[0][0]>>uv_feature[1][0]>>uv_feature[0][1]>>uv_feature[1][1]>>uv_feature[0][2]>>uv_feature[1][2]>>uv_feature[0][3]>>uv_feature[1][3]
                                >>uv_feature[0][4]>>uv_feature[1][4]>>uv_feature[0][5]>>uv_feature[1][5]>>uv_feature[0][6]>>uv_feature[1][6]>>uv_feature[0][7]>>uv_feature[1][7]
                                >>uv_feature[0][8]>>uv_feature[1][8]>>uv_feature[0][9]>>uv_feature[1][9]>>uv_feature[0][10]>>uv_feature[1][10]>>uv_feature[0][11]>>uv_feature[1][11]
                                >>uv_feature[0][12]>>uv_feature[1][12]>>uv_feature[0][13]>>uv_feature[1][13]>>uv_feature[0][14]>>uv_feature[1][14]>>uv_feature[0][15]>>uv_feature[1][15];
                        if(lane_index == image_index)
                               is_lane_match_image = 1;
                        else if(lane_index < image_index)
                            continue;
                        else{
                            printf("error: lane index is bigger than image index!!!\n");
                            break;
                        }
                    }

                    // 读取图片
                    char str_iamge_name_index[20]; // 新数据格式，补全8位
                    sprintf(str_iamge_name_index, "%08d", image_index);
                    image_name = frame_file_addr + "/" + str_iamge_name_index + ".jpg";

                    // IPM
                    cv::Mat org_image;
                    LoadImage(&org_image, image_name);
                    cv::Mat ipm_image = cv::Mat::zeros(ipm_para.height+1, ipm_para.width+1, CV_32FC1);
                    image_IPM(ipm_image, org_image, ipm_para);

                    // 执行预测lane
                    do_predict_feature(data_fusion);

                    /// 拟合当前lane
                    xy_feature = cv::Mat::zeros(2, pts_num, CV_32FC1);
                    uv_feature_pts = cv::Mat::zeros(2, pts_num, CV_32FC1);
                    for(int k=0; k<lane_num; k++){
                        for(int i1 = 0; i1<pts_num; i1++){
                            uv_feature_pts.at<float>(0, i1) = uv_feature[0][k*pts_num + i1];
                            uv_feature_pts.at<float>(1, i1) = uv_feature[1][k*pts_num + i1];
                        }
                        // get these points on the ground plane
                        bp_mapping.TransformImage2Ground(uv_feature_pts, &xy_feature);

                        std::vector<float> lane_coeffs_t;
                        polyfit1(&lane_coeffs_t, xy_feature, m_order); // 车道线拟合    Y = AX(X是纵轴);

                        for(int i = 0; i<m_order+1; i++)
                            lane_coeffs.at<float>(i, k) = lane_coeffs_t[i];
                    }

                    // 画车道线
                    mark_IPM_lane(ipm_image, lane_coeffs, ipm_para, 0.9);// 在IPM中标注当前lane 白色
                    mark_IPM_lane(ipm_image, lane_coeffs_pre, ipm_para, 0.45);// 在IPM中标注上一帧lane 灰白
                    mark_IPM_lane(ipm_image, lane_coeffs_predict, ipm_para, 0.15);// 在IPM中标注预测lane 黑色
                    cv::imshow("ipm", ipm_image);

                    //按键事件，空格暂停，其他跳出循环
                    int temp = cvWaitKey(100);
                    if (temp == 32)
                        while (cvWaitKey() == -1);
                    else if (temp >= 0)
                        break;
                }
            }
        }
    }

    return 0;
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

int polyfit_vector(std::vector<float>* lane_coeffs, std::vector<cv::Point2f>& vector_feature, int order )
{
    int feature_points_num = vector_feature.size();
    std::vector<float> x(feature_points_num);
    std::vector<float> y(feature_points_num);
    cv::Mat A = cv::Mat(feature_points_num, order + 1, CV_32FC1);
    cv::Mat b = cv::Mat(feature_points_num+1, 1, CV_32FC1);

    for (int i = 0; i < feature_points_num; i++){
        x[i] = (vector_feature.begin()+i)->y;
        y[i] = (vector_feature.begin()+i)->x;

        for (int j = 0; j <= order; j++)
            A.at<float>(i, j) = pow(y[i], j);

        b.at<float>(i) = x[i];
    }

    cv::Mat coeffs;
    int ret = cv::solve(A, b, coeffs, CV_SVD);
    if(ret<=0){
        printf("cv:solve error!!!\n");
        return -1;
    }

    for(int i=0; i<order+1; i++)
        lane_coeffs->push_back(coeffs.at<float>(i,0));
    return 1;
}


int polyfit1(std::vector<float>* lane_coeffs, const cv::Mat xy_feature, int order )
{
    int feature_points_num = xy_feature.cols;
    std::vector<float> x(feature_points_num);
    std::vector<float> y(feature_points_num);
    cv::Mat A = cv::Mat(feature_points_num, order + 1, CV_32FC1);
    cv::Mat b = cv::Mat(feature_points_num+1, 1, CV_32FC1);

        for (int i = 0; i < feature_points_num; i++) {
            x[i] = xy_feature.at<float>(0, i);
            y[i] = xy_feature.at<float>(1, i);

            for (int j = 0; j <= order; j++)
                A.at<float>(i, j) = pow(y[i], j);
            b.at<float>(i) = x[i];
        }

        cv::Mat coeffs;
        int ret = cv::solve(A, b, coeffs, CV_SVD);
        if(ret<=0){
            printf("cv:solve error!!!\n");
            return -1;
        }

        for(int i=0; i<order+1; i++)
            lane_coeffs->push_back(coeffs.at<float>(i,0));

        return 1;
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
        printf("can't open %s", filePath);

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

//// 标记 IPM lane
void mark_IPM_lane(cv::Mat &ipm_image, const cv::Mat lane_coeffs, const IPMPara ipm_para, const float lane_color_value)
{
    /// 在IPM中标注当前lane
    std::vector<int> x(ipm_para.height+2);
    std::vector<int> y(ipm_para.height+2);
    int i_index = -1;
    for (float i = ipm_para.y_limits[0]; i < ipm_para.y_limits[1]; i+=ipm_para.y_scale) {
       i_index += 1;
       y[i_index] = (-i + ipm_para.y_limits[1])/ipm_para.y_scale;
       float x_t = lane_coeffs.at<float>(0, 1) + lane_coeffs.at<float>(1, 1)*i + lane_coeffs.at<float>(2, 1)*i*i;
       x[i_index] = (x_t + ipm_para.x_limits[1])/ipm_para.x_scale;

       if (x[i_index] <= 0 || x[i_index] >= ipm_para.width )
           continue;
       else
           ipm_image.at<float>(y[i_index], x[i_index]) = lane_color_value;
    }
}


// 预测特征点并画图
void do_predict_feature(DataFusion &data_fusion )
{
    int64 t_1, t_2;

    if(is_first_lane_predict){
        is_first_lane_predict = 0;
        image_timestamp_pre = image_timestamp;
    }
    lane_coeffs.copyTo(lane_coeffs_pre);

    std::vector<cv::Point2f> vector_feature_predict;
    std::vector<cv::Point2f> vector_feature_pre;
    int lane_points_nums = 5; // 每一条车道线取的样本点数量
    double X[5] = {2.0, 5.0, 10.0, 20.0, 35.0};
    cv::Point2f point_xy;
    vector_feature_pre.clear();
    for(int points_index = 0; points_index<lane_points_nums; points_index++){
        point_xy.x = X[points_index];
        point_xy.y = lane_coeffs_pre.at<float>(0, 1) + lane_coeffs_pre.at<float>(1, 1)*X[points_index] + lane_coeffs_pre.at<float>(2, 1)*X[points_index]*X[points_index];
        vector_feature_pre.push_back(point_xy);
    }

    int64 image_timestamp_pre_int = (int64)(image_timestamp_pre*1000);
    int64 image_timestamp_cur_int = (int64)(image_timestamp*1000);

    int r_1 = 0;
    int main_sleep_counter = 0; //  一次外部调用，main sleep的次数
    while(!r_1){
        // 测试运行时间
        t_1 = f_time_counter.Microseconds();
        r_1 = data_fusion.GetPredictFeature( vector_feature_pre, image_timestamp_pre_int, image_timestamp_cur_int, &vector_feature_predict);
//        r_1 = ImuModule::Instance().GetPredictFeature( vector_feature_pre, image_timestamp_pre_int, image_timestamp_cur_int, &vector_feature_predict);
        t_2 = f_time_counter.Microseconds();

        int64 predict_cal_dt = (t_2 - t_1) ;
        VLOG(VLOG_INFO)<<"DF:main- "<<"predict_cal_dt= "<<predict_cal_dt<<endl;

        if(main_sleep_counter > 0)
        {
            printf("main timestamp diamatch conunter:%d, so sleep\n",  main_sleep_counter);
            usleep(100000);
        }
        main_sleep_counter++;
    }

    std::vector<float> lane_coeffs_t;
    polyfit_vector(&lane_coeffs_t, vector_feature_predict, m_order );
    for(int i = 0; i<m_order+1; i++)
        lane_coeffs_predict.at<float>(i, 1) = lane_coeffs_t[i];

    image_timestamp_pre = image_timestamp;

}
