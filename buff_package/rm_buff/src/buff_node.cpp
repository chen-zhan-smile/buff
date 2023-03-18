/*
 * @Description: buff_image_sub_node
 * @Version: 1.0
 * @Autor: chen zhan
 * @Date: 2023-02-20 21:35:39
 * @LastEditors: chen-zhan-smile
 * @LastEditTime: 2023-03-14 21:42:32
 */

#include "buff_node.hpp"


int main( int argc, char **argv ) {
    rclcpp::init( argc, argv );
    auto node = make_shared<BUFF::RM_buff>();
    rclcpp::spin( node );
    cv::namedWindow( "main" );
    cv::startWindowThread();
    cv::destroyWindow( "main" );
    rclcpp::shutdown();
    return 0;
}