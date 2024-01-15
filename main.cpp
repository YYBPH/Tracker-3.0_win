#include <opencv2/opencv.hpp>
#include "objectTracker.hpp"

using namespace std;
using namespace cv;


int main()
{
    cv::VideoCapture cap;

    cap.open(0);

    if (cap.isOpened() == false) {
        printf("\r\n\r\n**************************************************************\r\n");
        printf("capture open failed!\r\n");
        printf("*************************************************************\r\n\r\n\r\n");
        return 0;
    }

    ObjectsTracker objectsTracker;

    cv::namedWindow("newFrame", cv::WINDOW_NORMAL);
    cv::resizeWindow("newFrame", cv::Size(800, 600));

    while (1)
    {   
        // 获取新图像
        Mat newFrame;
        cap >> newFrame;

        if (!newFrame.empty()) 
        {
            newFrame = objectsTracker.tracker(newFrame);


            imshow("newFrame", newFrame);

            int key = cv::waitKey(10);
            if (key == 27)
                break;

        } else 
        {
            cout << "Empty frame!" << endl;
            return 0;
        }
    }
}