#ifndef __OTHERS_HPP
#define __OTHERS_HPP

#include <opencv2/opencv.hpp>


class Other
{
public:
    int num;
    int apper_times;
    int disap_times;
    cv::Rect rect;
    
    Other();
    ~Other();

    void update(cv::Rect newRect);
    void disapper();
private:
};



#endif // !__OTHERS_HPP