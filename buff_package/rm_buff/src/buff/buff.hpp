

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <functional>
#include <memory>

#include <opencv2/opencv.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <string>

#include "../ceres-solver/ceresSolver.hpp"
#include "../coordsolver/coordsolver.hpp"
#include "../fan_tracker/fan_tracker.hpp"

#include "std_msgs/msg/string.hpp"

#define msg_predict_pub std_msgs::msg::String
#define msg_serial_sub std_msgs::msg::String
namespace BUFF {
const int RED  = 1;
const int BLUE = 0;
class Buff : public rclcpp::Node {
  private:
    sensor_msgs::msg::Image::ConstSharedPtr      img_msg;
    std::shared_ptr<image_transport::Subscriber> image_sub;
    //图像发布器，可以将图像发送到对应的主题
    image_transport::Publisher binary_img_pub_;
    image_transport::Publisher predict_img_pub_;
    // 将预测后的结果发布到串口主题，从而通过串口转发给下位机
    rclcpp::Publisher<msg_predict_pub>::SharedPtr predict_pub_;
    // 定义一个接收端接收串口主题发布的信息
    rclcpp::Subscription<msg_serial_sub>::SharedPtr serial_sub_;
    // clang-format off
    //定义节点的一些参数
    string camera_name     = this->declare_parameter( "camera_name", "infantry_3" );
    string transport_      = this->declare_parameter( "subscribe_compressed", false ) ? "compressed" : "raw";
    bool USING_ROI         = this->declare_parameter( "using_roi", false );
    bool USING_IMU         = this->declare_parameter( "using_imu", false );
    int  color_mode        = this->declare_parameter( "isColorMode_Red", true ) ? RED : BLUE;
    bool SHOW_ALL_FANS     = this->declare_parameter( "show_all_fans", false );
    bool SHOW_PREDICT      = this->declare_parameter( "show_predict", true );
    bool SHOW_FPS          = this->declare_parameter( "show_fps", true );
    bool PRINT_LATENCY     = this->declare_parameter( "print_latency", false );
    bool PRINT_TARGET_INFO = this->declare_parameter( "print_target_info", false );
    // clang-format on
  public:
    Buff();
    void image_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr &msg ); //图像帧触发器
    void serial_topic_callbak( const msg_serial_sub &msg_serial );
    bool run( TaskData &src, VisionData &data );              // 自瞄主函数
    bool chooseFromTargets( vector<Fan> &fans, Fan &target ); //选择装甲板函数

  private:
    const std::string camera_param_path =
        "../params/coord_param.yaml"; //相机参数

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
    //使用同一预测器的最大时间间隔(ms)
    const int max_delta_t = 100;
    //大符臂长(R字中心至装甲板中心)
    const double fan_length = 0.7;
    //禁用ROI裁剪的装甲板占图像面积最大面积比值
    const double no_crop_thres = 2e-3;
    //定义一个扇，储存最后一个扇的数据
    Fan last_fan;
    //定义一个串口的数据结构
    VisionData visionData_;
    //定义一个任务的数据结构
    TaskData taskData_;

    std::shared_ptr<BUFF::BuffCeresSolver> predictor_; //解算拟合预测器
    std::shared_ptr<BUFF::CoordSolver>     coordSolver_;
    // CoordSolver   coordsolver;//坐标变换器

    Point2i cropImageByROI( Mat &img );
};

} // namespace BUFF