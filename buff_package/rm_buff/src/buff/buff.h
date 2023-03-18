#include <future>
#include <vector>

#include "../../include/ceresSolver.hpp"
#include "../fan_tracker_copy/fan_tracker.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fmt/color.h>
#include <fmt/format.h>
#include <opencv2/core.hpp>

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
    float2uchar pitch_angle; //偏航角
    float2uchar yaw_angle;   //俯仰角
    // float2uchar yaw_angle;//偏航角
    // float2uchar pitch_angle;//俯仰角
    float2uchar dis;          //目标距离
    int         isSwitched;   //目标是否发生切换
    int         isFindTarget; //当识别的图片范围内有目标且电控发来的信号不为0x00（关闭视觉）置为1，否则置0
    int         isSpinning;   //目标是否处于陀螺状态
    int         ismiddle; //设置1表示目标进入了可以开火的范围，设置0则表示目标尚未进入可开火的范围，默认置0
} VisionData;

class Buff {
  public:
    Buff();
    ~Buff();

    bool run( TaskData &src, VisionData &data ); // 自瞄主函数
    bool chooseFromTargets(  vector<Fan> &fans, Fan &target );

  private:
    const std::string network_path      = "../model/buff.xml";
    const std::string camera_param_path = "../params/coord_param.yaml";

    bool                    is_last_target_exists;
    int                     lost_cnt;
    int                     last_timestamp;
    double                  last_target_area;
    double                  last_bullet_speed;
    cv::Point2i             last_roi_center;
    cv::Point2i             roi_offset;
    cv::Size2d              input_size;
    std::vector<FanTracker> trackers;         // tracker
    const int               max_lost_cnt = 4; //最大丢失目标帧数
    const int               max_v        = 4; //最大旋转速度(rad/s)
    const int max_delta_t = 100; //使用同一预测器的最大时间间隔(ms)
    const double fan_length = 0.7; //大符臂长(R字中心至装甲板中心)
    const double no_crop_thres =
        2e-3; //禁用ROI裁剪的装甲板占图像面积最大面积比值

    Fan last_fan;

   
    BUFF::BuffCeresSolver predictor;//解算拟合预测器
    //CoordSolver   coordsolver;//坐标变换器

    Point2i cropImageByROI( Mat &img );
};
