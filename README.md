# data_fusion说明
## 文件说明：
data_fusion：预测相关的代码  
tests：本地测试main  

### data_fusion相关文件：
data_fusion.cpp：车道线预测的调用和接口函数  
imu_attitude_estimate.cpp：姿态解算  
can_vehicle_estimate.cpp：车辆运动信息解算  
datafusion_math.cpp：数学库  


### 数据文件
data_fusion：离线跑的时候会调用log数据，会读取build/bin/data/doing/log.txt文件


##备注
lane_tracking_main：进行本地标注数据测试的主函数  
#### 本地测试时的数据
lane_tracking_main：进行本地利用标注数据测试的时候，会调用下面数据文件：  
1). 车道线标注结果：build/bin/data/doing/lane_data.txt  
2). 图像文件：build/bin/data/doing/frame/***  
