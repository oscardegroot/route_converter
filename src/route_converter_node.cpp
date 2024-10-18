#include "route_converter/pch.h"
#include <route_converter/route_converter.h>

int main(int argc, char *argv[])
{
        // Initialize ROS 2
        rclcpp::init(argc, argv);

        // Create a shared pointer to the RouteConverter node
        auto node = std::make_shared<RouteConverter>();
        VISUALS.init(node.get());
        node->initialize();

        // Spin to keep the node alive and handle communication
        rclcpp::spin(node);

        // Shutdown ROS 2
        rclcpp::shutdown();
        return 0;
}