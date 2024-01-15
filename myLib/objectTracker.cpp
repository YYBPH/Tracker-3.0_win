#include "objectTracker.hpp"

// 腐蚀内核
Mat kernel_erode = (Mat_<uchar>(3, 3) <<  0, 1, 0,
                                          1, 0, 0,
                                          0, 0, 0);
// 膨胀内核
Mat kernel_dilate = (Mat_<uchar>(5, 5) <<  1, 1, 1, 1, 1,
                                           1, 1, 1, 1, 1,
                                           1, 1, 1, 1, 1,
                                           1, 1, 1, 1, 1,
                                           1, 1, 1, 1, 1);



ObjectsTracker::ObjectsTracker()
{
    this->MOG2 = createBackgroundSubtractorMOG2();
    this->hasObjFlag = false;

    this->maxDist = 100;
}

ObjectsTracker::~ObjectsTracker()
{
    
}

cv::Mat ObjectsTracker::tracker(Mat newframe)
{
    this->original_frame = newframe;

    /**** 0. 1号目标判断  *************************************/
    // 如果无 1号目标，重选 1号为目标
    if(hasObjFlag == false && this->rects.size() > 0)
    {
        object.ReinitKalmanFilter(this->rects[0]);
        printf("new rect:%d %d %d %d\r\n", this->rects[0].x, this->rects[0].y, this->rects[0].width, this->rects[0].height);
        hasObjFlag = true;
    }


    /**** 1. 获取运动候选框  *************************************/
    Mat grayFrame, mog2MaskFrame, erodeFrame, dilateFrame;
    this->MOG2->apply(newframe, mog2MaskFrame);
    erode(mog2MaskFrame, erodeFrame, kernel_erode, Point(-1, -1), 1, cv::BORDER_CONSTANT, cv::Scalar(0));
     
    this->rects.clear();
    vector<vector<Point>> contours;
    findContours(erodeFrame, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    for (size_t i = 0; i < contours.size(); i++) 
    {
        if (contourArea(contours[i]) < 4) continue;     
        Rect rect = boundingRect(contours[i]);
        this->rects.push_back(rect);
    }


    /** 2. 1号目标更新  *************************************/
    // 缺少删除
    cv::Rect rect = findClost(this->object.newRect);
    if(rect.width !=0 && rect.height !=0)
    {
        this->object.disap_times = 0;
        this->object.kalmanFilter(rect);
    }
    else
    {
        this->object.kalmanFilter(this->object.newRect);
        this->object.disap_times++;
        printf("disap_times:%d\r\n", this->object.disap_times);
        if(this->object.disap_times > 5)
        {
            this->hasObjFlag = false;
            printf("hasObjFlag = false\r\n");
        }
    }


    /** 3. drawInfo  *************************************/
    for (size_t i = 0; i < this->rects.size(); i++)
    {
        cv::rectangle(this->original_frame, this->rects[i], cv::Scalar(0, 0, 255), 1);
    }
    if(this->hasObjFlag == true)
        cv::rectangle(this->original_frame, this->object.newRect, cv::Scalar(255, 255, 0), 1);

    return this->original_frame;
}


// 在this->rects中找到最近的方框
cv::Rect ObjectsTracker::findClost(const cv::Rect rect)
{
    double clostDist = DBL_MAX;
    Rect clostRect;

    for(size_t i = 0; i < this->rects.size(); i++)
    {
        double dist = calculateRectDistance(rect, this->rects[i]);
        if(dist < clostDist)
        {
            clostDist = dist;
            clostRect = rects[i];
        }
    }

    if(clostDist < this->maxDist)
        return clostRect;
    else
    {
        clostRect.width = 0;
        clostRect.height = 0;
        return clostRect;
    }
}

// 计算两个矩形的中心点之间的欧氏距离
double ObjectsTracker::calculateRectDistance(const cv::Rect& rect1, const cv::Rect& rect2) {
    cv::Point2f center1(rect1.x + rect1.width / 2.0, rect1.y + rect1.height / 2.0);
    cv::Point2f center2(rect2.x + rect2.width / 2.0, rect2.y + rect2.height / 2.0);

    // 计算欧氏距离
    double distance = cv::norm(center1 - center2);

    return distance;
}


