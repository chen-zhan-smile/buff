/*
 * @Description: 
 * @Version: 1.0
 * @Autor: chen zhan
 * @Date: 2023-02-15 20:41:13
 * @LastEditors: chen zhan
 * @LastEditTime: 2023-02-15 20:45:40
 */
#ifndef __BUFF__
#define __BUFF__
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
//引入rclcpp，构建ros2节点

typedef struct armor_point{
    cv::Point point;
    double dis;
    double timestemp;
}armor_point;

class Buff
{
    private:
    /* data */
    public:
        Buff(/* args */);
        ~Buff();
};

Buff::Buff(/* args */)
{
}

Buff::~Buff()
{
}

#endif // __BUFF__
