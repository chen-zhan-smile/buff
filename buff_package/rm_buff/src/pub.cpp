/*
 * @Description: buff_image_pub
 * @Version: 1.0
 * @Autor: chen zhan
 * @Date: 2023-02-20 21:36:03
 * @LastEditors: chen zhan
 * @LastEditTime: 2023-02-21 21:39:25
 */

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_publisher", options);
  image_transport::ImageTransport it(node);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  std::string image_name_="/home/chen/Robomaster/Buff/buff_package/rm_buff/image/0.jpg";
  cv::Mat image = cv::imread(image_name_, cv::IMREAD_COLOR);
  std_msgs::msg::Header hdr;
  sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(hdr, "bgr8", image).toImageMsg();
  rclcpp::WallRate loop_rate(5);
  while (rclcpp::ok()) {
    pub.publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
}