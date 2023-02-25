/*
 * @Description: buff_image_sub_node
 * @Version: 1.0
 * @Autor: chen zhan
 * @Date: 2023-02-20 21:35:39
 * @LastEditors: chen zhan
 * @LastEditTime: 2023-02-25 22:08:54
 */

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include "opencv.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "fan_track/fan_tracking.hpp"
class My_Sub 
{
    rclcpp::NodeOptions options;
    /*创建image_listener节点指针*/
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_listener", options);
    /*image_transport定义*/
    image_transport::ImageTransport it;
    Fan_tracking tracker;
    bool is_fan_tracker_create=false;

public:
    int count_;
    My_Sub(/* args */):it(node)
    { 
      count_=0;
      image_transport::Subscriber sub = it.subscribe("camera/image",1,&My_Sub::imageCallback,this);
      rclcpp::spin(node);
    };
private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
    {
      try {
          cv::Mat image=cv_bridge::toCvShare(msg, "bgr8")->image;
          if (!is_fan_tracker_create)
          {
            tracker(image);
            is_fan_tracker_create=true;
          }
          cv::imshow("view", image);
          cv::waitKey(10);
      } 
      catch (const cv_bridge::Exception & e) {
          auto logger = rclcpp::get_logger("my_subscriber");
          RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
      }
      RCLCPP_INFO(node->get_logger(),"OK----------------!!!!!%d",count_); 
      count_++;
    };
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  My_Sub sub;
  cv::namedWindow("view");
  cv::startWindowThread();
  cv::destroyWindow("view");
  rclcpp::shutdown();
  return 0;
}