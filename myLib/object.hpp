#ifndef __OBJECT_HPP
#define __OBJECT_HPP

#include <opencv2/opencv.hpp>
#include "KalmanFilter.hpp"

class Object
{
public:
	Object();
	~Object();

    KalmanFilter kalman;	// 创建Kalman滤波器

	int disap_times;

	cv::Rect newRect;  	// 校正后的值

	void ReinitKalmanFilter(cv::Rect newRect);
	cv::Rect kalmanFilter(cv::Rect newRect);

private:

};

#endif // !__OBJECT_HPP