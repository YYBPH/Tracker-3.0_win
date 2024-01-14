#include <opencv2/opencv.hpp>
#include "objectTracker.hpp"

using namespace std;
using namespace cv;


int main()
{
    cv::VideoCapture cap;

    cap.open("./output.avi");

    if (cap.isOpened() == false) {
        printf("\r\n\r\n**************************************************************\r\n");
        printf("capture open failed!\r\n");
        printf("*************************************************************\r\n\r\n\r\n");
        return 0;
    }

    ObjectsTracker objectsTracker;


    while (1)
    {   
        // 获取新图像
        Mat newFrame;
        cap >> newFrame;
        // resize(newFrame, newFrame, Size(newFrame.cols/3, newFrame.rows/3), 0, 0);
        // resize(newFrame, newFrame, Size(800, 640), 0, 0);

        if (!newFrame.empty()) 
        {
            newFrame = objectsTracker.tracker(newFrame);


            imshow("newFrame", newFrame);

            int key = cv::waitKey(100);
            if (key == 27)
                break;

        } else 
        {
            cout << "Empty frame!" << endl;
            return 0;
        }
    }
}