
#include <string.h>
#include "datafusion_math.h"


DEFINE_double(gyro_bias_x, 0, "imu gyro bias x ");
DEFINE_double(gyro_bias_y, 0, "imu gyro bias y ");
DEFINE_double(gyro_bias_z, 0, "imu gyro bias z ");

#if defined DATA_FROM_LOG
    DEFINE_string(imu_init_addr, "./imu.ini", "imu calibrate result base address");
    DEFINE_string(imu_parameter_log_addr, "./imu_paramer_log.txt", "save all the imu calibrate results");
    DEFINE_string(log_data_addr, "./data/doing/log.txt", "the log data address (which save imu, speed and son on)");
    DEFINE_string(jpg_data_addr, "./data/doing/frame/image", "the jpg data address ");
    DEFINE_string(turnlamp_detect_init_addr, "./turnlamp_detect.ini", "turnlamp detect init parameter"); 
#else
    DEFINE_string(imu_init_addr, "/storage/sdcard0/imu.ini", "imu calibrate result base address");
    DEFINE_string(imu_parameter_log_addr, "/storage/sdcard0/imu_paramer_log.txt", "save all the imu calibrate results");
    DEFINE_string(turnlamp_detect_init_addr, "/storage/sdcard0/imu/turnlamp_detect.ini", "turnlamp detect init parameter"); 

#endif

    
int LowpassFilter3f(double y_pre[3], const double x_new[3], double dt, const double filt_hz, double y_new[3] )
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

