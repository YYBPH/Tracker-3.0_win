#ifndef __OBJECT_HPP
#define __OBJECT_HPP

#include <opencv2/opencv.hpp>
#include "KalmanFilter.hpp"

/*
Object默认构造函数：初始值（x,y,w,h,dx,dy）都为0.
ReinitKalmanFilter：初始值:newRect
*/

class Object
{
public:
	Object();
	~Object();

    MyKalmanFilter kalman;

	int disap_times;

	cv::Rect newRect;  

	void ReinitKalmanFilter(cv::Rect newRect);
	void kalmanFilter(cv::Rect newRect);

private:

};

#endif // !__OBJECT_HPP