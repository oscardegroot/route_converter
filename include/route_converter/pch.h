// pch.hpp
#ifndef PCH_HPP
#define PCH_HPP

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_extension/utility/query.hpp>              // See route-handler
#include <lanelet2_extension/utility/message_conversion.hpp> // See route-handler

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
// #include <autoware_planning_msgs/msg/path_with_lane_id.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/empty.hpp>

#include <autoware/route_handler/route_handler.hpp>

#include <ros_tools/logging.h>
#include <ros_tools/visuals.h>
#include <ros_tools/convertions.h>
#include <ros_tools/math.h>
#include <ros_tools/ros2_wrappers.h>

#endif // PCH_HPP