cmake_minimum_required(VERSION 3.8)
project(rm_buff)
set(DCMAKE_EXPORT_COMPILE_COMMANDS True)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(fmt_DIR /path)
# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
# set()
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
)

ament_auto_find_build_dependencies()
# ament_auto_add_executable(buff_sub
#   src/fan_track/fan_tracking.hpp
#   src/fan_track/fan_tracking.cpp
#   src/buff_node.hpp
#   src/buff_node.cpp

# )

# ament_auto_add_executable(buff_node
#   src/buff/buff.hpp
#   src/buff/buff.cpp
#   src/ceres-solver/ceresSolver.hpp
#   src/ceres-solver/ceresSolver.cpp
#   src/coordsolver/coordsolver.hpp
#   src/coordsolver/coordsolver.cpp
#   src/fan_tracker/fan_tracker.hpp
#   src/fan_tracker/fan_tracker.cpp
#   src/filter/filter.hpp
#   src/filter/filter.cpp
#   ./general.hpp
# )

ament_auto_add_executable(buff_image_pub
  src/pub.cpp

)

ament_target_dependencies(buff_image_pub Eigen3 fmt yaml-cpp Ceres)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
