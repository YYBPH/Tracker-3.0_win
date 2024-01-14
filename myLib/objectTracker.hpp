#ifndef __OBJECT_TRACKER2_H
#define  __OBJECT_TRACKER2_H

#include <opencv2/opencv.hpp>
#include <vector>

// #include "Object.hpp"
#include "object.hpp"

using namespace std;
using namespace cv;


class ObjectsTracker
{
public:
	ObjectsTracker();
	~ObjectsTracker();

	cv::Mat tracker(Mat newframe);


private:
    Ptr<BackgroundSubtractorMOG2> MOG2;

	Mat original_frame;

	vector<Rect> rects;

	Object object;



	// 标志位
	bool hasObjFlag;


	

private:
	double calculateRectDistance(const cv::Rect& rect1, const cv::Rect& rect2);

};


#endif