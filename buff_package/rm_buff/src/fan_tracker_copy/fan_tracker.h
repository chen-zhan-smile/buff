#include <cstddef>
#include <iostream>

#include <future>
#include <vector>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

struct Fan {
    int     id;  //id
    int     color; //扇叶的颜色
    double  conf; //扇叶的置信度
    string  key; 
    Point2f apex2d[ 5 ];
   
    /**
     * @description: 对应的相机坐标以及世界坐标
     * @return {*}
     * @author: chen-zhan-smile
     */
    Eigen::Vector3d centerR3d_cam; //
    Eigen::Vector3d centerR3d_world;
    Eigen::Vector3d armor3d_cam;
    Eigen::Vector3d armor3d_world;
    Eigen::Vector3d euler;
    Eigen::Matrix3d rmat;
};

class FanTracker {
  public:
    Fan    prev_fan;            //上一次装甲板
    Fan    last_fan;            //本次装甲板
    bool   is_last_fan_exists;  //是否存在上一次扇叶
    bool   is_initialized;      //是否完成初始化
    double rotate_speed;        //角速度
    size_t max_history_len = 2; //队列长度
    int    prev_timestamp;      //上次装甲板时间戳
    int    last_timestamp;      //本次装甲板时间戳

    std::deque<Fan> history_info; //目标队列

    FanTracker( Fan src, int src_timestamp );
    bool update( Fan new_fan, int new_timestamp );
};