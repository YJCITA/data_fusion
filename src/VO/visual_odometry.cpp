#include <stdexcept>
#include <string>
#include <iostream>
#include <fstream>

#include "visual_odometry.h"

using namespace std;
using namespace cv;

const int kMinNumFeature = 100;

VisualOdometry::VisualOdometry(PinholeCamera* cam):
cam_(cam)
{
	focal_ = cam_->fx();
	pp_ = cv::Point2d(cam_->cx(), cam_->cy());
	frame_stage_ = STAGE_FIRST_FRAME;
}

VisualOdometry::~VisualOdometry()
{

}

void VisualOdometry::Init()
{
    memset(m_diff_euler_angle, 0, sizeof(m_diff_euler_angle));
}

/// 提供一个图像
void VisualOdometry::ProcessNewImage(const cv::Mat& img, int frame_id)
{
	//对添加的图像进行判断
	if (img.empty() || img.type() != CV_8UC1 || img.cols != cam_->width() || img.rows != cam_->height())
		throw std::runtime_error("Frame: provided image has not the same size as the camera model or image is not grayscale");
	// 添加新帧
	new_frame_ = img;
	bool res = true;//结果状态
	if(frame_stage_ == STAGE_FIRST_FRAME){
        // 第一次进入系统的时候执行一次
		res = processFirstFrame();
        printf("STAGE_FIRST_FRAME\n");
    }else if(frame_stage_ == STAGE_SECOND_FRAME)
		res = processSecondFrame();
    else if(frame_stage_ == STAGE_DEFAULT_FRAME)
		res = processFrame(frame_id);

    // 处理结束之后将当前帧设为上一处理帧
	last_frame_ = new_frame_;

}

bool VisualOdometry::processFirstFrame()
{
	// 对当前帧进行特征检测
	featureDetection(new_frame_, px_ref_);//第一帧计算出来的特征，为参考特征点
	// 修改状态，表明第一帧已经处理完成
	frame_stage_ = STAGE_SECOND_FRAME;
	return true;
}

bool VisualOdometry::processSecondFrame()
{
	featureTracking(last_frame_, new_frame_, px_ref_, px_cur_, disparities_); //通过光流跟踪确定第二帧中的相关特征
	// 计算初始位置
	cv::Mat E, R, t, mask;
	E = cv::findEssentialMat(px_cur_, px_ref_, focal_, pp_, cv::RANSAC, 0.999, 1.0, mask);
	cv::recoverPose(E, px_cur_, px_ref_, R, t, focal_, pp_, mask);
        
	cur_R_ = R.clone();
	cur_t_ = t.clone();
	frame_stage_ = STAGE_DEFAULT_FRAME;// 设置状态处理默认帧
	px_ref_ = px_cur_;
	return true;
}

bool VisualOdometry::processFrame(int frame_id)
{
	double scale = 1.00;//初始尺度为1
	featureTracking(last_frame_, new_frame_, px_ref_, px_cur_, disparities_); //通过光流跟踪确定第二帧中的相关特征
	
	cv::Mat E, R, t, mask;
	E = cv::findEssentialMat(px_cur_, px_ref_, focal_, pp_, cv::RANSAC, 0.999, 1.0, mask);
	cv::recoverPose(E, px_cur_, px_ref_, R, t, focal_, pp_, mask); // 求取R T
    
    // 计算两帧之间变化的角度
    CalculateEulerAngleFromRotation(R, m_diff_euler_angle); 
	
    // -YJ-
// 	scale = getAbsoluteScale(frame_id);//得到当前帧的实际尺度(目前是利用groud truth 进行计算)
	// 如果尺度小于0.1可能计算出的Rt存在一定的问题,则不做处理，保留上一帧的值
	if (scale > 0.1){
		cur_t_ = cur_t_ + scale*(cur_R_*t);
		cur_R_ = R*cur_R_;
	}
    
    cout<<"NumFeature: "<<px_ref_.size()<<endl;
	// 如果跟踪特征点数小于给定阈值，进行重新全图特征检测
	if (px_ref_.size() < kMinNumFeature){
        // 目前这一块是有问题的，重新全图搜索出来的feature跟pre的如何match
		featureDetection(new_frame_, px_ref_);
		featureTracking(last_frame_, new_frame_, px_ref_, px_cur_, disparities_);
	}
	px_ref_ = px_cur_;
	return true;
}


void VisualOdometry::GetCurrentDiffEuler(double euler_angle[3])
{
    memcpy(euler_angle, m_diff_euler_angle, sizeof(m_diff_euler_angle));
}

// 从旋转矩阵R计算姿态角Euler
void VisualOdometry::CalculateEulerAngleFromRotation(const cv::Mat& R, double euler_angle[3])
{
    // print R
    double r21 = R.at<double>(1, 0);
    double r11 = R.at<double>(0, 0);
    double r31 = R.at<double>(2, 0);
    double r32 = R.at<double>(2, 1);
    double r33 = R.at<double>(2, 2);
    euler_angle[0] = atan(r21/r11) * 180.0 / M_PI;
    euler_angle[1] = atan(r32 / r33) * 180.0 / M_PI;
    euler_angle[2] = atan(-r31 / sqrt(r32 * r32 + r33 * r33)) * 180.0 / M_PI;    
//     std::cout<<"angle: roll = "<<euler_angle[0]<<" pitch = "<<euler_angle[1]<<" yaw = "<<euler_angle[2]<<std::endl;
}

double VisualOdometry::getAbsoluteScale(int frame_id)
{
	std::string line;
	int i = 0;
	std::ifstream ground_truth("./data/poses/01.txt");
	double x = 0, y = 0, z = 0;
	double x_prev, y_prev, z_prev;
	// 获取当前帧真实位置与前一帧的真实位置的距离作为尺度值
	if (ground_truth.is_open()){
		while ((std::getline(ground_truth, line)) && (i <= frame_id)){
			z_prev = z;
			x_prev = x;
			y_prev = y;
			std::istringstream in(line);
			for (int j = 0; j < 12; j++){
				in >> z;
				if (j == 7) y = z;
				if (j == 3)  x = z;
			}
			i++;
		}
		ground_truth.close();
	}else {
		std::cerr<< "Unable to open file";
		return 0;
	}

	return sqrt((x - x_prev)*(x - x_prev) + (y - y_prev)*(y - y_prev) + (z - z_prev)*(z - z_prev));
}

// px_vec = keypoints
void VisualOdometry::featureDetection(cv::Mat image, std::vector<cv::Point2f>& px_vec)	
{
	std::vector<cv::KeyPoint> keypoints;
	int fast_threshold = 20;
	bool non_max_suppression = true;
	cv::FAST(image, keypoints, fast_threshold, non_max_suppression);
	cv::KeyPoint::convert(keypoints, px_vec);
    // -YJ- 画特征点
    cv::drawKeypoints(image, keypoints, image, cv::Scalar::all(255), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);
    cout<<"search for new feature!!"<<endl;
}

//通过光流跟踪(LK)确定第二帧中的相关特征
// 目前直接采用OpenCV中calcOpticalFlowPyrLK方法即金字塔的KLT算法，采用金字塔的KLT算法是为了解决运动范围过大及不连贯的情况，
// 通过在图像金字塔的最高层计算光流，用得到的运动估计结果作为下一次金字塔的起始点，重复这个过程直到金字塔的最底层。
// 具体将图像ItIt中检测到的特征FtFt跟踪到图像It+1It+1中，得到对应的特征Ft+1Ft+1。
void VisualOdometry::featureTracking(cv::Mat image_ref, cv::Mat image_cur,
	std::vector<cv::Point2f>& px_ref, std::vector<cv::Point2f>& px_cur, std::vector<double>& disparities)
{
	const double klt_win_size = 21.0;
	const int klt_max_iter = 30;
	const double klt_eps = 0.001;
	std::vector<uchar> status;
	std::vector<float> error;
	std::vector<float> min_eig_vec;
	cv::TermCriteria termcrit(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, klt_max_iter, klt_eps);
	cv::calcOpticalFlowPyrLK(image_ref, image_cur,
                    		px_ref, px_cur,
                    		status, error,
                    		cv::Size2i(klt_win_size, klt_win_size),
                    		4, termcrit, 0);

	std::vector<cv::Point2f>::iterator px_ref_it = px_ref.begin();
	std::vector<cv::Point2f>::iterator px_cur_it = px_cur.begin();
	disparities.clear(); 
    disparities.reserve(px_cur.size());
    // -YJ- plot
    cv::Mat match_feature_image(image_ref.rows, image_ref.cols + image_cur.cols+1, CV_8UC1);
    image_ref.copyTo(Mat(match_feature_image,Rect(0, 0, image_ref.cols, image_ref.rows)));
    image_cur.copyTo(Mat(match_feature_image,Rect(image_ref.cols,0, image_ref.cols, image_ref.rows)));
//     cv::Mat match_feature_image(image_ref.rows*2, image_ref.cols, CV_8UC1);
//     image_ref.copyTo(Mat(match_feature_image,Rect(0, 0, image_ref.cols, image_ref.rows)));
//     image_cur.copyTo(Mat(match_feature_image,Rect(0, image_ref.rows, image_ref.cols, image_ref.rows)));
        
	for (size_t i = 0; px_ref_it != px_ref.end(); ++i){
		if (!status[i]){
			px_ref_it = px_ref.erase(px_ref_it);
			px_cur_it = px_cur.erase(px_cur_it);
			continue;
		}
		// 这边记录了跟踪之后对应特征之间的像素距离
		// 在我们对特征进行跟踪的时候，跟踪到的特征会越来越少（由于有相同视野的部分越来越小），因此设定一个阈值以保证特征的个数，
		// 小于给定阈值的时候，我们重新进行特征检测。
		disparities.push_back(norm(cv::Point2d(px_ref_it->x - px_cur_it->x, px_ref_it->y - px_cur_it->y)));
        
        // 画两帧之间feature的匹配图
        line(match_feature_image, cv::Point(px_ref_it->x, px_ref_it->y), cv::Point(px_ref_it->x + image_ref.cols, px_cur_it->y), 200, 1.5); 
//         line(match_feature_image, cv::Point(px_ref_it->x, px_ref_it->y), cv::Point(px_ref_it->x, px_cur_it->y + image_ref.rows), 180, 1.5); 
        
		++px_ref_it;
		++px_cur_it;
	}
    cv::imshow("match feature", match_feature_image);
}
