/*
 * @Description: 
 * @Version: 1.0
 * @Autor: chen zhan
 * @Date: 2023-02-21 21:43:24
 * @LastEditors: chen zhan
 * @LastEditTime: 2023-02-24 22:08:05
 */
#ifndef __FAN_TRACKING__
#define __FAN_TRACKING__
#include "opencv2/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>

/*显示的宏定义，定义之后打开画面显示*/
#define _SHOW_

#define Match_Error 0
#define Match_Success 1
using namespace std;

typedef struct armor_point{
    cv::Point point;
    double dis;
}armor_point;

class Fan_tracking
{
    private:
        cv::Mat image_;
        armor_point armor_point_; //装甲信息
        vector<vector<cv::Point>> contours_;
        cv::Point r_center; // R的中心点，非常重要，根据R中心来确定能量机关在画面中的位置
        vector<cv::RotatedRect> suspect_armor_mrect; //可疑的最小轮廓旋转矩形
        vector<cv::RotatedRect> target_min_rects;//筛选出的最小轮廓旋转矩形,正确识别时应该为2
        bool find_ok=false;
    public:
        explicit Fan_tracking(cv::Mat image);
        void drawRotatedRect(cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness=1);
        double distance(cv::Point a,cv::Point b);
        void image_process();
        void suspect_profile_screen();
        int matching_profile();
};

#endif // !__FAN_TRACKING__
