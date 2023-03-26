
#ifndef __GRNERAL_HPP__
#define __GRNERAL_HPP__
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <fmt/color.h>
#include <fmt/format.h>
#include <future>
#include <opencv2/core.hpp>
#include <vector>

using namespace std;
using namespace cv;

namespace BUFF {

struct TaskData {
    int                mode;
    double             bullet_speed;
    cv::Mat            img;
    Eigen::Quaterniond quat;
    int                timestamp; //单位：ms
};

struct BuffObject {
    Point2f                  apex[ 5 ];
    cv::Rect_<float>         rect;
    int                      cls;
    int                      color;
    float                    prob;
    std::vector<cv::Point2f> pts;
};

/**
 * @description: 联合体让一个浮点数为四个字节
 * @description:
 * 可以通过float2uchar.c来操作一个uint32类型的对象，当需要将uint32变量的低端字节看做一个字符的时候，只需要访问float2uchar.c就可以了。
 * @return {*}
 * @author: chen-zhan-smile
 */
typedef union {
    float         f;
    unsigned char c[ 4 ];
} float2uchar;

/**
 * @description: 串口来的数据，定义一个结构体接收，串口作为一个主题
 * 不停发布当前车的状态数据
 * @return {*}
 * @author: chen-zhan-smile
 */
typedef struct {
    float2uchar pitch_angle;  //偏航角
    float2uchar yaw_angle;    //俯仰角
    float2uchar dis;          //目标距离
    int         isSwitched;   //目标是否发生切换
    int         isFindTarget; //当识别的图片范围内有目标且电控发来的信号不为0x00（关闭视觉）置为1，否则置0
    int         isSpinning;   //目标是否处于陀螺状态
    int         ismiddle; //设置1表示目标进入了可以开火的范围，设置0则表示目标尚未进入可开火的范围，默认置0
} VisionData;



//初始化矩阵
template <typename T>
bool initMatrix( Eigen::MatrixXd &matrix, std::vector<T> &vector ) {
    int cnt = 0;
    for ( int row = 0; row < matrix.rows(); row++ ) {
        for ( int col = 0; col < matrix.cols(); col++ ) {
            matrix( row, col ) = vector[ cnt ];
            cnt++;
        }
    }
    return true;
}

} // namespace BUFF
#endif