#ifndef IMU_MODULE_H_
#define IMU_MODULE_H_

#include <vector>
#include "common/base/singleton.h"
#include "common/base/stdint.h"

namespace imu {

class DataFusion;

class ImuModule : public SingletonBase<ImuModule> {
public:
    friend class SingletonBase<ImuModule>;

    ~ImuModule();

    void Init(); 

    void Destory( );

    // timestamp_search in microseconds
    // 1: 数据正常
    // 0: no R in the buffer
    // -1:int_timestamp_search < all_data_time 落后
    // -2:int_timestamp_search > all_data_time 超前
    // -3:imu的值异常，默认返回R=0
    // -4:imu初始化失败，默认返回R=0
    int GetTurnRadius(const int64_t &timestamp_search, double *R);

    // 1: 数据正常
    // 0: 没有数据
    // -1:int_timestamp_search < all_data_time 落后
    // -2:int_timestamp_search > all_data_time 超前
    int GetPredictFeature(const std::vector<cv::Point2f>& vector_feature_pre ,
            int64_t image_timestamp_pre,
            int64_t image_timestamp_cur,
            std::vector<cv::Point2f>* vector_feature_predict);
    
//     int GetTimestampData(double timestamp_search, double vehicle_pos[2], double att[3], double *angle_z, double att_gyro[3], double acc[3], double gyro[3] );

private:
    ImuModule();

private:
    DataFusion *m_data_fusion;
};

}

#endif
