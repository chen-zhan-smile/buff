// Copyright 2021, Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <sstream>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

int main( int argc, char **argv ) {
    // Check if video source has been passed as a parameter
    rclcpp::init( argc, argv );
    rclcpp::NodeOptions     options;
    rclcpp::Node::SharedPtr node =
        rclcpp::Node::make_shared( "image_publisher", options );
    image_transport::ImageTransport it( node );
    image_transport::Publisher      pub = it.advertise( "camera/image", 1 );

    YAML::Node config = YAML::LoadFile( "../params/filter/filter.yaml" );
    // Convert the command line parameter index for the video device to an
    // integer
    std::string      video_path = "/home/chen/buff_test/buff_blue.mp4";
    cv::VideoCapture cap;
    cap.open( video_path );
    // Check if video device can be opened with the given index
    if ( !cap.isOpened() ) {
        return 1;
    }
    cv::Mat frame;

    sensor_msgs::msg::Image::ConstSharedPtr msg;

    rclcpp::WallRate loop_rate( 5 );
    while ( rclcpp::ok() ) {
        cap >> frame;
        // Check if grabbed frame is actually full with some content
        if ( !frame.empty() ) {
            msg = cv_bridge::CvImage( msg->header, "bgr8", frame ).toImageMsg();
            pub.publish( msg );
            cv::waitKey( 1 );
        }

        rclcpp::spin_some( node );
        loop_rate.sleep();
    }

    return 0;
}