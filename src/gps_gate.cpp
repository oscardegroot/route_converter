#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <ros_tools/logging.h>
#include <ros_tools/math.h>
#include <ros_tools/visuals.h>
#include <ros_tools/profiling.h>

#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

class GateChecker : public rclcpp::Node
{
public:
        GateChecker() : Node("gate_checker")
        {
                STATIC_NODE_POINTER.init(this);
                VISUALS.init(this);

                prev_pos_ = Eigen::Vector2d(0., 0.);

                double delay;
                this->declare_parameter("gps_gate.delay", 1.75);
                this->get_parameter("gps_gate.delay", delay); // Delay (in s)

                // This delays the pedestrian scenario (in s)
                delay_ = std::make_unique<RosTools::Timer>(delay);

                std::filesystem::path exePath = std::filesystem::path(__FILE__).parent_path();

                std::string filename = (exePath / "saved_poses.csv").string(); // See scripts/gps_gate_recorder.py
                if (!load_poses(filename))
                {
                        LOG_ERROR("Failed to load GPS gate poses from file. See scripts/gps_gate_recorder.py");
                }
                else
                {
                        LOG_INFO("GPS Gate Initialized Succesfully");
                }

                subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
                    "/localization/kinematic_state", 10,
                    std::bind(&GateChecker::odometry_callback, this, std::placeholders::_1));

                publisher_ = this->create_publisher<std_msgs::msg::Empty>("/pedestrian_simulator/reset", 10);
        }

private:
        bool load_poses(const std::string &filename)
        {
                std::ifstream file(filename);
                if (!file.is_open())
                {
                        return false;
                }

                std::string line;
                int i = 0;
                while (std::getline(file, line))
                {
                        std::stringstream ss(line);
                        std::string cell;
                        std::vector<std::string> row;

                        while (std::getline(ss, cell, ','))
                        {
                                row.push_back(cell);
                        }

                        if (row.size() == 2)
                        {
                                double x = std::stod(row[0]);
                                double y = std::stod(row[1]);
                                if (i == 0)
                                {
                                        gate_p1_ = Eigen::Vector2d(x, y);
                                }
                                else if (i == 1)
                                {
                                        gate_p2_ = Eigen::Vector2d(x, y);
                                }
                        }
                        i++;
                }
                file.close();
                return true;
        }

        void odometry_callback(nav_msgs::msg::Odometry::SharedPtr msg)
        {
                Eigen::Vector2d pos(msg->pose.pose.position.x, msg->pose.pose.position.y);

                if (RosTools::distance(pos, prev_pos_) > 5.)
                {
                        LOG_INFO("GPS Gate: Detected jump in position");
                        prev_sign_ = -2; // Reset the gate sign
                }

                prev_pos_ = pos;

                // Check if the vehicle has passed through the gate
                if (is_through_gate(pos))
                {

                        visualize(true);
                        delay_->start();
                        delay_has_started_ = true;
                        return;
                }

                if (delay_has_started_ && delay_->hasFinished())
                {
                        std_msgs::msg::Empty reset_msg;
                        publisher_->publish(reset_msg);
                        delay_has_started_ = false;
                }

                visualize(false);
        }

        bool is_through_gate(Eigen::Vector2d pos)
        {

                // Construct a line through the two points that make up the gate
                Eigen::Vector2d a = gate_p2_ - gate_p1_;

                // Orthogonal
                Eigen::Vector2d orth = a.unitOrthogonal();
                double b = orth.transpose() * gate_p2_; // p2 is on the line

                // Compute the projection of the vehicle onto the orthogonal
                double line_dist = orth.transpose() * pos - b;

                double overall_dist = std::min(
                    (pos - gate_p1_).norm(),
                    (pos - gate_p2_).norm());

                if (prev_sign_ == -2) // First run
                {
                        prev_sign_ = RosTools::sgn(line_dist);
                        return false;
                }

                if (overall_dist < 5.0 && RosTools::sgn(line_dist) != prev_sign_) // Check if the sign changed
                {
                        prev_sign_ = RosTools::sgn(line_dist);
                        LOG_INFO("GPS Gate Triggered - Resetting Pedestrians");
                        return true;
                }

                prev_sign_ = RosTools::sgn(line_dist);
                return false;
        }

        void visualize(bool triggered)
        {
                auto &publisher = VISUALS.getPublisher("gps_gate");

                auto &gate = publisher.getNewPointMarker("CUBE");
                gate.setScale(RosTools::distance(gate_p1_, gate_p2_), 0.25, 2.0);
                gate.setOrientation(std::atan2(gate_p2_.y() - gate_p1_.y(), gate_p2_.x() - gate_p1_.x()));

                if (prev_sign_ < 0)
                {
                        gate.setColorInt(9, 0.3);
                }
                else
                {
                        gate.setColorInt(9, 0.8);
                }
                gate.addPointMarker((gate_p1_ + gate_p2_) / 2., 0);

                publisher.publish();
        }

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_;
        Eigen::Vector2d gate_p1_, gate_p2_;

        std::unique_ptr<RosTools::Timer> delay_;
        bool delay_has_started_{false};

        int prev_sign_ = -2;
        Eigen::Vector2d prev_pos_;
};

int main(int argc, char *argv[])
{
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<GateChecker>());
        rclcpp::shutdown();
        return 0;
}
