#include "data_fusion/data_fusion.h"
#include "imu_module.h"

namespace imu {

ImuModule::ImuModule() {
    m_data_fusion = new DataFusion();
}

ImuModule::~ImuModule() {
    delete m_data_fusion;
    m_data_fusion = NULL;
}

void ImuModule::Init() 
{
    m_data_fusion->Init();
    
}

void ImuModule::Destory( )
{
    m_data_fusion->Destory();
}

int ImuModule::GetTurnRadius(const int64_t &timestamp_search, double *R) {
    return m_data_fusion->GetTurnRadius(timestamp_search / 1000, R);
}

int ImuModule::GetPredictFeature(const std::vector<cv::Point2f>& vector_feature_pre ,
        int64_t image_timestamp_pre,
        int64_t image_timestamp_cur,
        std::vector<cv::Point2f>* vector_feature_predict)
{
    return m_data_fusion->GetPredictFeature(vector_feature_pre,
            image_timestamp_pre,
            image_timestamp_cur,
            vector_feature_predict);
}

// int ImuModule::GetTimestampData(double timestamp_search, double vehicle_pos[2], double att[3], double *angle_z, double att_gyro[3], double acc[3], double gyro[3] )
// {
//     return m_data_fusion->GetTimestampData(timestamp_search, vehicle_pos, att, angle_z, att_gyro, acc, gyro);
// }

}
