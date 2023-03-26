#ifndef __BUFF_NODE__
#define __BUFF_NODE__
#include "ceres-solver/ceresSolver.hpp"
#include "cv_bridge/cv_bridge.h"
#include "fan_track/fan_tracking.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include <functional>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
namespace BUFF {

class RM_buff : public rclcpp::Node {

  private:
    std::shared_ptr<image_transport::Subscriber> image_sub;
    std::shared_ptr<BUFF::BuffCeresSolver>       solver;
    
  public:
    //buff节点的构造函数，当节点构造时进行初始化
    RM_buff()
        : Node( "rm_buff" ) {
        image_sub = std::make_shared<image_transport::Subscriber>(
            image_transport::create_subscription(
                this, "camera/image",
                std::bind( &RM_buff::image_callback, this,
                           std::placeholders::_1 ),
                "raw", rmw_qos_profile_sensor_data ) );
    };
    //定义图像帧触发器
    void image_callback( const sensor_msgs::msg::Image::ConstSharedPtr &msg );
};

void RM_buff::image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr &msg ) {
    try {
        
        //接收图像帧处理函数
        auto cv_ptr =
            cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 );
        cv::imshow( "main", cv_ptr->image );
        cv::namedWindow( "main" );
        cv::waitKey( 10 );
    } catch ( const cv_bridge::Exception &e ) {
        auto logger = rclcpp::get_logger( "my_subscriber" );
        RCLCPP_ERROR( logger, "Could not convert from '%s' to 'bgr8'.",
                      msg->encoding.c_str() );
    }
    RCLCPP_INFO( this->get_logger(), "OK----------------!!!!!" );
}

} // namespace BUFF
#endif