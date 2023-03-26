#ifndef __DERECTOR__
#define __DERECTOR__

#include "../../general.hpp"

#include <rclcpp/logger.hpp>

namespace BUFF {

class DetectorBuff {
  public:
    DetectorBuff();
    ~DetectorBuff();

  public:
    bool detect( cv::Mat &src, std::vector<BUFF::BuffObject> &objects);
};

} // namespace BUFF

#endif