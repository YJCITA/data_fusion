#include "imu_attitude_estimate.h"

namespace imu {

ImuAttitudeEstimate::ImuAttitudeEstimate()
{
    Initialize( );
}

void ImuAttitudeEstimate::Initialize( )
{
    m_factor_acc_gyro[0] = 0.001; // 加速度计修正的姿态的系数
    m_factor_acc_gyro[1] = 0.001;
    m_factor_acc_gyro[2] = 0.001;
    memset(&m_att, 0, sizeof(m_att));
    memset(&m_gyro_angle, 0, sizeof(m_gyro_angle));
    m_angle_z = 0.0;
    m_att_init_counter = 30;
    m_accel_range_scale = 8.0/32768;
    m_gyro_range_scale = 2000.0/180*3.141593/32768;
	
	// 摄像头版本
	m_is_imu_mod_set = false;
	m_imu_mode = 0; // 默认是0，1:旧摄像头 2：新摄像头

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

    // 初始化为0,　目前会从配置文件中读取，并每次上电在这个基础上进一步校正
    m_gyro_A0[0] = 0;
    m_gyro_A0[1] = 0;
    m_gyro_A0[2] = 0;
    // murata
    m_accel_range_scale_murata = 1.0/1962;

    m_acc_A0_murata[0] = 0;
    m_acc_A0_murata[1] = 0;
    m_acc_A0_murata[2] = 0;

    m_acc_A1_murata[0][0] = 1;
    m_acc_A1_murata[0][1] = 0;
    m_acc_A1_murata[0][2] = 0;
    m_acc_A1_murata[1][0] = 0;
    m_acc_A1_murata[1][1] = 1;
    m_acc_A1_murata[1][2] = 0;
    m_acc_A1_murata[2][0] = 0;
    m_acc_A1_murata[2][1] = 0;
    m_acc_A1_murata[2][2] = 1;
    
    m_gyro_range_scale_murata = 1.0/50*D2R;
    m_gyro_A0_murata[0] = 0;
    m_gyro_A0_murata[1] = 0;
    m_gyro_A0_murata[2] = 0;

}

void ImuAttitudeEstimate::UpdataAttitude( const double acc_data[3], const double gyro_data[3], double dt)
{
    double acc_angle[3];
    double gyro_rate[3];
    static unsigned char start_flag = 0; // first time run
    // ACC to angle
    acc_angle[X_AXIS] = (atan2f(-acc_data[Y_AXIS], -acc_data[Z_AXIS]));       // Calculating pitch ACC angle
    acc_angle[Y_AXIS] = (atan2f(acc_data[X_AXIS], sqrtf(acc_data[Z_AXIS]*acc_data[Z_AXIS] + acc_data[Y_AXIS]*acc_data[Y_AXIS])));   //Calculating roll ACC angle

    double acc_normal = sqrtf(acc_data[X_AXIS]*acc_data[X_AXIS] + acc_data[Y_AXIS]*acc_data[Y_AXIS] + acc_data[Z_AXIS]*acc_data[Z_AXIS])/ONE_G;
    if(acc_normal > 1.1 || acc_normal < 0.9){
        m_factor_acc_gyro[0] = 0.0002; // 加速度计修正的姿态的系数
        m_factor_acc_gyro[1] = 0.0002;
        m_factor_acc_gyro[2] = 0.0002;
    }else{
        m_factor_acc_gyro[0] = 0.005; // 加速度计修正的姿态的系数
        m_factor_acc_gyro[1] = 0.005;
        m_factor_acc_gyro[2] = 0.005;
    }

    if( start_flag == 0 ){
        m_att_init_counter--;
        if(m_att_init_counter<1){
            start_flag = 1;
            m_att[X_AXIS] = acc_angle[X_AXIS];
            m_att[Y_AXIS] = acc_angle[Y_AXIS];
            m_att[Z_AXIS] = 0.0;
        }else{
            m_att[X_AXIS] = 0.0;
            m_att[Y_AXIS] = 0.0;
            m_att[Z_AXIS] = 0.0;
        }
    }else{
        // X
        gyro_rate[X_AXIS] =  gyro_data[X_AXIS] + sinf(m_att[X_AXIS])*tanf(m_att[Y_AXIS])*gyro_data[Y_AXIS] + cosf(m_att[X_AXIS])*tanf(m_att[Y_AXIS])*gyro_data[Z_AXIS];
        m_gyro_angle[X_AXIS] = m_gyro_angle[X_AXIS]+ gyro_rate[X_AXIS]* dt;
        m_att[X_AXIS] = (m_att[X_AXIS] + gyro_rate[X_AXIS]* dt)+ m_factor_acc_gyro[X_AXIS]*(acc_angle[X_AXIS] - m_att[X_AXIS]);
        // Y
        gyro_rate[Y_AXIS] = cosf(m_att[X_AXIS])*gyro_data[Y_AXIS] - sinf(m_att[X_AXIS])*gyro_data[Z_AXIS] ;
        m_gyro_angle[Y_AXIS] = m_gyro_angle[Y_AXIS]+ gyro_rate[Y_AXIS] * dt;
        m_att[Y_AXIS] = (m_att[Y_AXIS] + gyro_rate[Y_AXIS]*dt)+ m_factor_acc_gyro[Y_AXIS]*(acc_angle[Y_AXIS] - m_att[Y_AXIS]);
        // Z
        gyro_rate[Z_AXIS] = sinf(m_att[X_AXIS])/cosf(m_att[Y_AXIS])*gyro_data[Y_AXIS] + cosf(m_att[X_AXIS])/cosf(m_att[Y_AXIS])*gyro_data[Z_AXIS];
        m_gyro_angle[Z_AXIS]  = m_gyro_angle[Z_AXIS]+ gyro_rate[Z_AXIS] * dt;
        m_att[Z_AXIS] = (m_att[Z_AXIS] + gyro_rate[Z_AXIS]*dt);

        m_angle_z += gyro_data[Z_AXIS] * dt;

    }
}

// euler attitude to Rbn(from n to b)
void ImuAttitudeEstimate::CalculateAtt2Rotation(const double att[3], double R[3][3] )
{
    double roll, pitch, yaw;
    roll = att[0];
    pitch = att[1];
    yaw = att[2];

    R[0][0] = cosf(pitch)*cosf(yaw);
    R[0][1] = cosf(pitch)*sinf(yaw);
    R[0][2] = -sinf(pitch);
    R[1][0] = cosf(yaw)*sinf(pitch)*sinf(roll) - cosf(roll)*sinf(yaw);
    R[1][1] = cosf(roll)*cosf(yaw) + sinf(pitch)*sinf(roll)*sinf(yaw);
    R[1][2] = cosf(pitch)*sinf(roll);
    R[2][0] = sinf(roll)*sinf(yaw) + cosf(roll)*cosf(yaw)*sinf(pitch);
    R[2][1] = cosf(roll)*sinf(pitch)*sinf(yaw) - cosf(yaw)*sinf(roll);
    R[2][2] = cosf(pitch)*cosf(roll);
 }


void ImuAttitudeEstimate::GetAttitude(double att[3])
{
    att[X_AXIS] = m_att[X_AXIS];
    att[Y_AXIS] = m_att[Y_AXIS];
    att[Z_AXIS] = m_att[Z_AXIS];
}

void ImuAttitudeEstimate::GetAttitudeAngleZ(double att[3], double *angle_z)
{
    att[X_AXIS] = m_att[X_AXIS];
    att[Y_AXIS] = m_att[Y_AXIS];
    att[Z_AXIS] = m_att[Z_AXIS];

    *angle_z = m_angle_z;
}

// 获取由gyro积分得到的姿态角度
void ImuAttitudeEstimate::GetAttitudeGyro(double att_gyro[3])
{
    att_gyro[X_AXIS] = m_gyro_angle[X_AXIS];
    att_gyro[Y_AXIS] = m_gyro_angle[Y_AXIS];
    att_gyro[Z_AXIS] = m_gyro_angle[Z_AXIS];
}

void ImuAttitudeEstimate::ResetState()
{
    m_att[Z_AXIS] = 0.0;
}


int ImuAttitudeEstimate::LowpassFilter3f(double y_pre[3], const double x_new[3], double dt, const double filt_hz, double y_new[3] )
{
    double alpha = 0.0f, rc = 0.0f;
    double y_filter[3];

    if(filt_hz <= 0.0f || dt < 0.0f){
        y_new[0] = x_new[0];
        y_new[1] = x_new[1];
        y_new[2] = x_new[2];
        return 0;
    }else{
        rc = 1.0f/(2.0*3.1415*filt_hz);
        alpha = dt/(dt + rc);
    }

    y_filter[0] = y_pre[0] + alpha*(x_new[0] - y_pre[0]);
    y_filter[1] = y_pre[1] + alpha*(x_new[1] - y_pre[1]);
    y_filter[2] = y_pre[2] + alpha*(x_new[2] - y_pre[2]);

    memcpy(y_new, y_filter, sizeof(y_filter));
    return 1;

}

int ImuAttitudeEstimate::SetAccCalibationParam(double A0[3], double A1[3][3])
{
    m_acc_A0[0]  = A0[0];
    m_acc_A0[1]  = A0[1];
    m_acc_A0[2]  = A0[2];

    m_acc_A1[0][0] = A1[0][0];
    m_acc_A1[0][1] = A1[0][1];
    m_acc_A1[0][2] = A1[0][2];
    m_acc_A1[1][0] = A1[1][0];
    m_acc_A1[1][1] = A1[1][1];
    m_acc_A1[1][2] = A1[1][2];
    m_acc_A1[2][0] = A1[2][0];
    m_acc_A1[2][1] = A1[2][1];
    m_acc_A1[2][2] = A1[2][2];

    return 1;
}

// acc_out_calibrated = A1*(acc - A0)  
int ImuAttitudeEstimate::AccDataCalibation(const double acc_data_raw[3], double acc_data_ned[3] )
{
    double acc_data_t[3], acc_data_raw_t[3];
	// 选择摄像头版本
	if(!m_is_imu_mod_set){
		#if defined(IMU_MODE_AUTO_SELECT)// 新旧版本的摄像头模组自动选择
			double acc_z_tmp = acc_data_raw[0]*m_accel_range_scale;
			if( acc_z_tmp < -0.6)
				m_imu_mode = 2; // 新摄像头
			else
				m_imu_mode = 1;
		#else
			// 设定了摄像头是哪个版本
			#if defined(IMU_MODE_SET_2)
				m_imu_mode = 2; // 默认是新版本摄像头
			#else
				m_imu_mode = 1; // 
			#endif
		#endif
		m_is_imu_mod_set = true;
		printf("imu mode: %d\n", m_imu_mode);   
        VLOG(VLOG_DEBUG)<<"DF:ImuAttitudeEstimate::AccDataCalibation--"<<"imu mode: "<<m_imu_mode<<endl;
        VLOG(VLOG_DEBUG)<<"DF:ImuAttitudeEstimate::AccDataCalibation--"<<"acc_scal = "<<m_accel_range_scale<<endl;
        VLOG(VLOG_DEBUG)<<"DF:ImuAttitudeEstimate::AccDataCalibation--"<<"acc_A0 = "<<endl
                        <<m_acc_A0[0]<<" "<<m_acc_A0[1]<<" "<<m_acc_A0[2]<<endl;
        VLOG(VLOG_DEBUG)<<"DF:ImuAttitudeEstimate::AccDataCalibation--"<<"acc_A1 = "<<endl
                    <<m_acc_A1[0][0]<<" "<<m_acc_A1[0][1]<<" "<<m_acc_A1[0][2]<<endl
                    <<m_acc_A1[1][0]<<" "<<m_acc_A1[1][1]<<" "<<m_acc_A1[1][2]<<endl
                    <<m_acc_A1[2][0]<<" "<<m_acc_A1[2][1]<<" "<<m_acc_A1[2][2]<<endl;    
	} 
     
	if(m_imu_mode == 1){
		// 1: 旧摄像头
		// IMU原始坐标系-->大地坐标系(NED)  imu:
		acc_data_raw_t[0] = -acc_data_raw[2]*m_accel_range_scale;
		acc_data_raw_t[1] = -acc_data_raw[1]*m_accel_range_scale;
		acc_data_raw_t[2] = -acc_data_raw[0]*m_accel_range_scale;
	}else if(m_imu_mode == 2){
		// 新摄像头 2017.02.10
		// IMU原始坐标系-->大地坐标系(NED)
		acc_data_raw_t[0] = -acc_data_raw[2]*m_accel_range_scale;
		acc_data_raw_t[1] = acc_data_raw[1]*m_accel_range_scale;
		acc_data_raw_t[2] = acc_data_raw[0]*m_accel_range_scale;
	}else{
        memset(acc_data_raw_t, 0, sizeof(acc_data_raw_t));  
		printf("imu mode set error!!!!\n");
        return 0;
	}

    // 校正
    acc_data_t[0] = acc_data_raw_t[0] - m_acc_A0[0];
    acc_data_t[1] = acc_data_raw_t[1] - m_acc_A0[1];
    acc_data_t[2] = acc_data_raw_t[2] - m_acc_A0[2];

    acc_data_ned[0]= (m_acc_A1[0][0]*acc_data_t[0] + m_acc_A1[0][1]*acc_data_t[1] + m_acc_A1[0][2]*acc_data_t[2])*ONE_G; // 地理坐标系Z
    acc_data_ned[1]= (m_acc_A1[1][0]*acc_data_t[0] + m_acc_A1[1][1]*acc_data_t[1] + m_acc_A1[1][2]*acc_data_t[2])*ONE_G; // 地理坐标系Y
    acc_data_ned[2]= (m_acc_A1[2][0]*acc_data_t[0] + m_acc_A1[2][1]*acc_data_t[1] + m_acc_A1[2][2]*acc_data_t[2])*ONE_G;  // 地理坐标系X
    
//     VLOG(VLOG_DEBUG)<<"DF:ImuAttitudeEstimate::AccDataCalibation--"<<"acc_data_raw_t = "<<acc_data_raw_t[0]<<", "<<acc_data_raw_t[1]<<", "<<acc_data_raw_t[2]<<endl;
//     VLOG(VLOG_DEBUG)<<"DF:ImuAttitudeEstimate::AccDataCalibation--"<<"acc_data_t = "<<acc_data_t[0]<<", "<<acc_data_t[1]<<", "<<acc_data_t[2]<<endl;
//     VLOG(VLOG_DEBUG)<<"DF:ImuAttitudeEstimate::AccDataCalibation--"<<"acc_data_ned = "<<acc_data_ned[0]<<", "<<acc_data_ned[1]<<", "<<acc_data_ned[2]<<endl;
    
    return 1;
}

int ImuAttitudeEstimate::GyrocDataCalibation(const double gyro_data_raw[3], double gyro_data_ned[3] )
{
    double gyro_data_raw_t[3];
   
	if(m_imu_mode == 1){
		// 1: 旧摄像头
        // IMU原始坐标系-->大地坐标系(NED)
        gyro_data_raw_t[0] = -gyro_data_raw[2]*m_gyro_range_scale;
        gyro_data_raw_t[1] = -gyro_data_raw[1]*m_gyro_range_scale;
        gyro_data_raw_t[2] = -gyro_data_raw[0]*m_gyro_range_scale;
	}else if(m_imu_mode == 2){
	    // 新摄像头 2017.02.10
		// IMU原始坐标系-->大地坐标系(NED)
		gyro_data_raw_t[0] = -gyro_data_raw[2]*m_gyro_range_scale;
		gyro_data_raw_t[1] = gyro_data_raw[1]*m_gyro_range_scale;
		gyro_data_raw_t[2] = gyro_data_raw[0]*m_gyro_range_scale;
	}else{
		memset(gyro_data_raw_t, 0, sizeof(gyro_data_raw_t));  
		printf("imu mode set error!!!!\n");
        return 0;
	}
	
    //校正
    gyro_data_ned[0] = gyro_data_raw_t[0] - m_gyro_A0[0];
    gyro_data_ned[1] = gyro_data_raw_t[1] - m_gyro_A0[1];
    gyro_data_ned[2] = gyro_data_raw_t[2] - m_gyro_A0[2];

    return 1;
}

// Murata IMU
int ImuAttitudeEstimate::AccDataCalibationMurata(const double acc_data_raw[3], double acc_data_ned[3] )
{
    double acc_data_t[3];
    double acc_data_imu[3];
    for(int i=0; i<3; i++){
        acc_data_t[i] = acc_data_raw[i]*m_accel_range_scale - m_acc_A0_murata[i];
        acc_data_imu[i]= (m_acc_A1[i][0]*acc_data_t[0] + m_acc_A1[i][1]*acc_data_t[1] + m_acc_A1[i][2]*acc_data_t[2])*ONE_G;
    }
    acc_data_t[0] = acc_data_raw[0]*m_accel_range_scale - m_acc_A0_murata[0];
    acc_data_t[1] = acc_data_raw[1]*m_accel_range_scale - m_acc_A0[1];
    acc_data_t[2] = acc_data_raw[2]*m_accel_range_scale - m_acc_A0[2];

    // IMU原始坐标系-->大地坐标系(NED)
    acc_data_ned[0] = -acc_data_imu[2];
    acc_data_ned[1] = -acc_data_imu[1];
    acc_data_ned[2] = acc_data_imu[0];

    return 1;
}

int ImuAttitudeEstimate::GyrocDataCalibationMurata(const double gyro_data_raw[3], double gyro_data_new[3] )
{
    double gyro_data_imu[3], gyro_data_imu_t[3];
    for(int i=0; i<3; i++)
        gyro_data_imu[i] = gyro_data_raw[i]*m_gyro_range_scale_murata;

    // IMU原始坐标系-->大地坐标系(NED)
    gyro_data_imu_t[0] = -gyro_data_imu[2];
    gyro_data_imu_t[1] = -gyro_data_imu[1];
    gyro_data_imu_t[2] = gyro_data_imu[0];

    // 去掉drift
    for(int i=0; i<3; i++)
         gyro_data_new[i] = -gyro_data_imu_t[i] - m_gyro_A0_murata[i];

    return 1;
}

// 获取当前陀螺仪零偏
void ImuAttitudeEstimate::GetGyroBias( double gyro_A0[3] )
{
    gyro_A0[0] = m_gyro_A0[0];
    gyro_A0[1] = m_gyro_A0[1];
    gyro_A0[2] = m_gyro_A0[2];
}

// 设置陀螺仪新零偏
void ImuAttitudeEstimate::SetGyroBias( const double gyro_bias_new[3] )
{
    m_gyro_A0[0] = gyro_bias_new[0];
    m_gyro_A0[1] = gyro_bias_new[1];
    m_gyro_A0[2] = gyro_bias_new[2];
}

// 陀螺仪新零偏清零
void ImuAttitudeEstimate::ClearGyroBias(  )
{
    m_rw_lock.WriterLock();
    m_gyro_A0[0] = 0;
    m_gyro_A0[1] = 0;
    m_gyro_A0[2] = 0;
    m_rw_lock.WriterUnlock();
}


// 设置imu参数
int ImuAttitudeEstimate::SetImuParameter(const StructImuParameter imu_parameter)
{
    m_rw_lock.WriterLock();
    memcpy(m_gyro_A0, imu_parameter.gyro_A0, sizeof(m_gyro_A0));
    memcpy(m_acc_A0, imu_parameter.acc_A0, sizeof(m_acc_A0));
    memcpy(m_acc_A1, imu_parameter.acc_A1, sizeof(m_acc_A1));
    m_rw_lock.WriterUnlock();

   printf("set acc_A1: %f %f %f\n", imu_parameter.acc_A1[0][0], imu_parameter.acc_A1[1][1], imu_parameter.acc_A1[2][2]);
    return 1;
}

// imu参数清零
int ImuAttitudeEstimate::ResetImuParameter(  )
{
    m_rw_lock.WriterLock();
    memset(m_gyro_A0, 0, sizeof(m_gyro_A0));
    memset(m_acc_A0, 0, sizeof(m_acc_A0));
    memset(m_acc_A1, 0, sizeof(m_acc_A1));
    m_acc_A1[0][0] = 1;
    m_acc_A1[1][1] = 1;
    m_acc_A1[2][2] = 1;
    m_rw_lock.WriterUnlock();
    printf("reset imu parameter!!!!\n");
    return 1;
}




}

