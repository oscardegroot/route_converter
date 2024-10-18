#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <route_converter/pch.h>



#define LOG_MARK(msg)         \
  if (config_->debug_output_) \
  {                           \
    LOG_INFO(msg);            \
  }

class RoadmapConfig
{

  /**
   * @brief Class for retrieving configuration parameters
   *
   */

public:
  RoadmapConfig();
  ~RoadmapConfig();
  bool declared_ = false;

  /**
   * @brief initialize:  check parameters on parameter server and read from there
   * @param node_handle_name: node handler initialize from name, as parameter set inside that name
   * @return true all parameter initialize successfully else false
   */
  bool initialize(rclcpp::Node::SharedPtr node);

  // High-level Parameters
  bool debug_output_;

  std::string map_package_name_;
  std::string map_file_name_;

  double update_frequency_;

  // Spline settings
  bool fit_clothoid_;

  double spline_sample_distance_;
  double minimum_waypoint_distance_;
  double clothoid_point_per_xm_;

  double max_velocity_;

  // Visual settings
  double scale_;

  // Topics
  std::string external_waypoint_topic_;

  bool success_;

  double autoware_update_interval_;
  double autoware_forward_distance_, autoware_backward_distance_;
  bool autoware_include_other_lane_;

  /* Retrieve paramater, if it doesn't exist return false */
  template <class T>
  bool retrieveParameter(rclcpp::Node::SharedPtr node, const std::string &name, T &value)
  {
    if (!declared_)
      node->declare_parameter<T>(name);

    return node->get_parameter(name, value);
  }

  template <class T>
  bool retrieveParameter(rclcpp::Node *node, const std::string &name, T &value)
  {
    if (!declared_)
      node->declare_parameter<T>(name);

    return node->get_parameter(name, value);
  }

  /* Retrieve parameter, if it doesn't exist use the default */
  template <class T>
  void retrieveParameter(rclcpp::Node::SharedPtr node, const std::string &name, T &value, const T &default_value)
  {
    if (!declared_)
      node->declare_parameter<T>(name, default_value);

    node->get_parameter(name, value);
  }

  /* Retrieve parameter, if it doesn't exist use the default */
  template <class T>
  void retrieveParameter(rclcpp::Node *node, const std::string &name, T &value, const T &default_value)
  {
    if (!declared_)
      node->declare_parameter<T>(name, default_value);

    node->get_parameter(name, value);
  }

  template <class L>
  void retrieveParameter(rclcpp::Node::SharedPtr node, const std::string &name, std::vector<L> &value, const std::vector<L> &default_value)
  {
    if (!declared_)
      node->declare_parameter<std::vector<L>>(name, default_value);

    node->get_parameter(name, value);
  }

  template <class L>
  void retrieveParameter(rclcpp::Node *node, const std::string &name, std::vector<L> &value, const std::vector<L> &default_value)
  {
    if (!declared_)
      node->declare_parameter<std::vector<L>>(name, default_value);

    node->get_parameter(name, value);
  }
};

#endif
