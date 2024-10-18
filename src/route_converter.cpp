#include <route_converter/route_converter.h>

#include <route_converter/configuration.h>

namespace
{
        rclcpp::SubscriptionOptions createSubscriptionOptions(rclcpp::Node *node_ptr)
        {
                rclcpp::CallbackGroup::SharedPtr callback_group =
                    node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

                auto sub_opt = rclcpp::SubscriptionOptions();
                sub_opt.callback_group = callback_group;

                return sub_opt;
        }
} // namespace

RouteConverter::RouteConverter()
    : rclcpp::Node("route_converter")
{
        route_handler_ = std::make_shared<route_handler::RouteHandler>();
        STATIC_NODE_POINTER.init(this);
}

void RouteConverter::initialize()
{
        LOG_INFO("Initializing");

        // Initialize the configuration
        config_ = std::make_shared<RoadmapConfig>();
        config_->initialize(shared_from_this());

        VISUALS.init(this);

        auto qos_transient_local = rclcpp::QoS{1}.transient_local();

        // The map of the world
        vector_map_subscriber_ = this->create_subscription<HADMapBin>(
            "~/input/vector_map", qos_transient_local,
            std::bind(&RouteConverter::onMap, this, _1),
            createSubscriptionOptions(this));

        // The route to be followed on the map
        route_subscriber_ = this->create_subscription<LaneletRoute>(
            "~/input/route", 1,
            std::bind(&RouteConverter::onRoute, this, _1),
            createSubscriptionOptions(this));

        // The vehicle position (to figure out from where we should look up the route)
        odometry_subscriber_ = this->create_subscription<Odometry>(
            "~/input/odometry", 1,
            std::bind(&RouteConverter::onOdometry, this, _1),
            createSubscriptionOptions(this));

        initial_pose_subscriber_ = this->create_subscription<PoseWithCovarianceStamped>(
            "/initialpose", 1,
            std::bind(&RouteConverter::onInitialPose, this, _1),
            createSubscriptionOptions(this));

        goal_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "~/input/goal", 1,
            std::bind(&RouteConverter::onGoal, this, _1),
            createSubscriptionOptions(this));

        autoware_path_publisher_ = this->create_publisher<PathWithLaneId>(
            "~/output/path_with_lane_id", 1);

        goal_reached_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
            "~/output/goal_reached", 1);

        // reference_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        //     "roadmap/reference", 1);

        LOG_SUCCESS("Ready");
}

void RouteConverter::onInitialPose(PoseWithCovarianceStamped::ConstSharedPtr msg)
{
        (void)msg;
        if (route_ptr_)
        {
                LOG_INFO("Vehicle respawned. Recomputing the route...");

                onRoute(route_ptr_); // Computing a new part of the map
        }
}

void RouteConverter::onGoal(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
        goal_ = Eigen::Vector2d(msg->pose.position.x, msg->pose.position.y);
        goal_angle_ = RosTools::quaternionToAngle(msg->pose.orientation);

        odometry_ptr_ = nullptr; // When a new goal is received, we need to read the state
}

void RouteConverter::onOdometry(Odometry::ConstSharedPtr msg)
{
        if (!odometry_ptr_)
        {
                if (config_->debug_output_)
                {
                        LOG_INFO("First odometry received");
                }

                odometry_ptr_ = msg; // Read odometry if it was not received yet
        }
        else
        {

                // Check if the vehicle reached the goal
                if (RosTools::distance(Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y),
                                       goal_) < 5.0)
                {
                        LOG_SUCCESS("Goal Reached");
                        goal_reached_publisher_->publish(std_msgs::msg::Empty());
                        goal_ = Eigen::Vector2d(); // To prevent multiple  resets
                }

                rclcpp::Time previous_msg_time = odometry_ptr_->header.stamp;
                rclcpp::Time cur_msg_time = msg->header.stamp;
                if ((cur_msg_time - previous_msg_time).seconds() > config_->autoware_update_interval_)
                { // Otherwise, check how old the data is
                        if (config_->debug_output_)
                        {
                                LOG_INFO("Updating odometry and route");
                        }

                        odometry_ptr_ = msg; // And update if necessary
                        if (route_ptr_ != nullptr)
                        {
                                onRoute(route_ptr_); // Computing a new part of the map
                        }
                }
        }
}

void RouteConverter::onMap(HADMapBin::ConstSharedPtr msg)
{
        if (config_->debug_output_)
        {
                LOG_INFO("Map Received");
        }
        map_ptr_ = msg;
}

void RouteConverter::onRoute(LaneletRoute::ConstSharedPtr msg)
{
        if (config_->debug_output_)
        {
                LOG_INFO("Route Received");
        }

        // We need the map to understand the route
        if (!map_ptr_)
        {
                LOG_INFO_THROTTLE(5000, "waiting for lanelet_map msg...");
                return;
        }
        if (!odometry_ptr_)
        {
                LOG_INFO_THROTTLE(5000, "waiting for odometry msg...");
                return;
        }

        // Error handling
        if (msg->segments.size() == 1)
        {
                LOG_WARN("Route has only one segment, not updating the route.");
                return;
        }

        if (config_->debug_output_)
                LOG_INFO("Converting " << msg->segments.size() << " lanelets to the goal into a reference path for planning");

        route_ptr_ = msg;

        route_handler_->setMap(*map_ptr_);
        route_handler_->setRoute(*route_ptr_);

        PathWithLaneId reference_path{};
        lanelet::ConstLanelets current_lanes;
        getReferencePath(reference_path, current_lanes);

        std::vector<Eigen::Vector2d> centerline;
        std::vector<double> lengths, left_width, right_width;
        convertRoute(reference_path, current_lanes,
                     lengths, centerline, left_width, right_width);

        visualize(reference_path);
        autoware_path_publisher_->publish(reference_path);
}

void RouteConverter::getReferencePath(PathWithLaneId &reference_path, lanelet::ConstLanelets &current_lanes)
{
        /** @see behavior_path_planner/planner manager */

        geometry_msgs::msg::Pose cur_pose = odometry_ptr_->pose.pose;
        // PathWithLaneId reference_path{}, extended_reference_path{}; // The final reference path

        const auto backward_length = config_->autoware_backward_distance_; // How far back should the path go
        const auto forward_length = config_->autoware_forward_distance_;   // How far forward should the path go

        reference_path.header = route_handler_->getRouteHeader();

        // Find the closest lanelet to our lanelet
        lanelet::ConstLanelet closest_lane{};
        route_handler_->getClosestLaneletWithinRoute(cur_pose, &closest_lane); // Get the closest lanelet to the pose

        current_lanes = route_handler_->getLaneletSequence(
            closest_lane, cur_pose, -backward_length, forward_length);
        reference_path = route_handler_->getCenterLinePath(
            current_lanes, -backward_length,
            forward_length, true);
        AddRoadBoundaries(reference_path, current_lanes);
}

void RouteConverter::convertRoute(PathWithLaneId &reference_path, const lanelet::ConstLanelets &current_lanes,
                                  std::vector<double> &lengths,
                                  std::vector<Eigen::Vector2d> &centerline,
                                  std::vector<double> &width_left, std::vector<double> &width_right)
{

        /** @brief points */
        auto &publisher = VISUALS.getPublisher("debug");
        auto &cube = publisher.getNewPointMarker("CUBE");
        cube.setScale(0.2, 0.2, 0.2);
        std::vector<Eigen::Vector2d> center, left, right;

        double length = 0.;
        const auto &first_lane = current_lanes[0];
        center.push_back(Eigen::Vector2d(first_lane.centerline2d().front().x(), first_lane.centerline2d().front().y()));
        left.push_back(Eigen::Vector2d(first_lane.leftBound2d().front().x(), first_lane.leftBound2d().front().y()));
        right.push_back(Eigen::Vector2d(first_lane.rightBound2d().front().x(), first_lane.rightBound2d().front().y()));
        lengths.push_back(length);

        if (config_->debug_output_)
                LOG_INFO("Splitting boundaries and centerline into segments");
        // Split in segments
        int num_segments = 12;
        for (size_t l = 0; l < current_lanes.size(); l++)
        {
                const auto &lane = current_lanes[l];
                double lane_length = lanelet::geometry::length2d(lane);
                double left_length = getLineLength(lane.leftBound2d());
                double right_length = getLineLength(lane.rightBound2d());

                if (l == current_lanes.size() - 1)
                {
                        // Project the goal to the centerline
                        auto goal_on_centerline = lanelet::geometry::project(lane.centerline2d(), goal_);
                        auto arc_goal = lanelet::geometry::toArcCoordinates(lane.centerline2d(), goal_on_centerline);
                        double ratio = arc_goal.length / lane_length;
                        lane_length = arc_goal.length;
                        left_length *= ratio;
                        right_length *= ratio;
                }

                int n_segments = std::ceil(lane_length / config_->spline_sample_distance_);
                n_segments = std::min(num_segments, n_segments);

                double left_step = left_length / (double)n_segments;
                for (int s = 1; s < n_segments; s++)
                {
                        auto left_point = lanelet::geometry::interpolatedPointAtDistance(
                            lane.leftBound2d(), (double)s * left_step);
                        left.push_back(Eigen::Vector2d(left_point.x(), left_point.y()));
                }

                double right_step = right_length / (double)n_segments;
                for (int s = 1; s < n_segments; s++)
                {
                        auto right_point = lanelet::geometry::interpolatedPointAtDistance(
                            lane.rightBound2d(), (double)s * right_step);
                        right.push_back(Eigen::Vector2d(right_point.x(), right_point.y()));

                        length += lane_length / (double)n_segments;
                        lengths.push_back(length);
                }

                double center_step = lane_length / (double)n_segments;
                for (int s = 1; s < n_segments; s++)
                {
                        auto center_point = lanelet::geometry::interpolatedPointAtDistance(
                            lane.centerline2d(), (double)s * center_step);
                        center.push_back(Eigen::Vector2d(center_point.x(), center_point.y()));
                }

                // auto middle_left = lanelet::geometry::interpolatedPointAtDistance(
                //     lane.leftBound2d(), left_length / 2.);
                // auto middle_right = lanelet::geometry::interpolatedPointAtDistance(
                //     lane.rightBound2d(), right_length / 2.);
                // // lanelet::ArcCoordinates arc_half;
                // // arc_half.length = lane_length / 2.;
                // // auto middle_left = lanelet::geometry::fromArcCoordinates(lane.leftBound2d(), half_arc);
                // left.push_back(Eigen::Vector2d(middle_left.x(), middle_left.y()));
                // // auto middle_right = lanelet::geometry::fromArcCoordinates(lane.rightBound2d(), half_arc);
                // right.push_back(Eigen::Vector2d(middle_right.x(), middle_right.y()));

                if (l != current_lanes.size() - 1)
                {
                        left.push_back(Eigen::Vector2d(lane.leftBound2d().back().x(), lane.leftBound2d().back().y()));
                        right.push_back(Eigen::Vector2d(lane.rightBound2d().back().x(), lane.rightBound2d().back().y()));
                        center.push_back(Eigen::Vector2d(lane.centerline2d().back().x(), lane.centerline2d().back().y()));
                }
                else
                {
                        auto left_point = lanelet::geometry::interpolatedPointAtDistance(
                            lane.leftBound2d(), left_length);
                        left.push_back(Eigen::Vector2d(left_point.x(), left_point.y()));

                        auto right_point = lanelet::geometry::interpolatedPointAtDistance(
                            lane.rightBound2d(), right_length);
                        right.push_back(Eigen::Vector2d(right_point.x(), right_point.y()));

                        auto center_point = lanelet::geometry::interpolatedPointAtDistance(
                            lane.centerline2d(), lane_length);
                        center.push_back(Eigen::Vector2d(center_point.x(), center_point.y()));
                }

                length += lane_length / (double)n_segments;
                lengths.push_back(length);
        }

        cube.setColor(0., 0., 0.);
        for (auto &point : center)
                cube.addPointMarker(point);

        cube.setColor(1., 0., 0.);

        for (auto &point : left)
                cube.addPointMarker(point);

        for (auto &point : right)
                cube.addPointMarker(point);

        publisher.publish();

        // Compute all lengths
        std::vector<double> all_center_lengths;
        for (const auto &point : reference_path.points)
        {
                double arclength = 0.;
                Eigen::Vector2d p(point.point.pose.position.x, point.point.pose.position.y);
                for (const auto &lane : current_lanes)
                {
                        if (lanelet::geometry::inside(lane, p))
                        {
                                all_center_lengths.push_back(arclength +
                                                             lanelet::geometry::toArcCoordinates(lane.centerline2d(), p).length);
                                break;
                        }
                        arclength += lanelet::geometry::length2d(lane);
                }
        }

        /** @brief Widths
        // std::vector<double> width_left, width_right, lengths,
        std::vector<double> velocities;
        // std::vector<Eigen::Vector2d> centerline_points;
        double length = 0.;
        lengths.push_back(length);
        const auto &first_lane = current_lanes[0];
        centerline.push_back(Eigen::Vector2d(first_lane.centerline2d()[0].x(), first_lane.centerline2d()[0].y()));

        width_left.push_back(RosTools::distance(first_lane.leftBound2d()[0].basicPoint2d(), first_lane.centerline2d()[0].basicPoint2d()));
        width_right.push_back(RosTools::distance(first_lane.rightBound2d()[0].basicPoint2d(), first_lane.centerline2d()[0].basicPoint2d()));

        for (auto &lane : current_lanes)
        {
                length += lanelet::geometry::length2d(lane);
                lengths.push_back(length);

                centerline.push_back(Eigen::Vector2d(lane.centerline2d().back().x(), lane.centerline2d().back().y()));

                width_left.push_back(RosTools::distance(lane.leftBound2d().back().basicPoint2d(), lane.centerline2d().back().basicPoint2d()));
                width_right.push_back(RosTools::distance(lane.rightBound2d().back().basicPoint2d(), lane.centerline2d().back().basicPoint2d()));
        }

        centerline.push_back(Eigen::Vector2d(current_lanes.back().centerline2d().back().x(), current_lanes.back().centerline2d().back().y()));

        width_left.push_back(RosTools::distance(current_lanes.back().leftBound2d().back().basicPoint2d(), current_lanes.back().centerline2d().back().basicPoint2d()));
        width_right.push_back(RosTools::distance(current_lanes.back().rightBound2d().back().basicPoint2d(), current_lanes.back().centerline2d().back().basicPoint2d()));
*/

        // Overwrite boundary points here
        reference_path.left_bound.clear();
        reference_path.right_bound.clear();
        reference_path.points.clear();

        geometry_msgs::msg::Point point;

        for (size_t i = 0; i < left.size(); i++)
        {
                // point.x = lengths[i]; // Should be the "s" distance along the centerpath spline
                // point.y = width_left[i];
                point.x = left[i](0);
                point.y = left[i](1);
                point.z = lengths[i];

                reference_path.left_bound.push_back(point);
        }

        for (size_t i = 0; i < right.size(); i++)
        {
                // point.x = lengths[i];
                // point.y = width_right[i];
                point.x = right[i](0);
                point.y = right[i](1);
                point.z = lengths[i];

                reference_path.right_bound.push_back(point);
        }

        for (size_t i = 0; i < center.size(); i++)
        {
                reference_path.points.emplace_back();
                reference_path.points[i].point.pose.position.x = center[i](0);
                reference_path.points[i].point.pose.position.y = center[i](1);
                reference_path.points[i].point.heading_rate_rps = lengths[i];

                // reference_path.points[i].point.heading_rate_rps = all_center_lengths[i];
        }

        // }

        // Fit a velocity reference based on the curvature of the path

        // Convert to nav path and forward to the rest of the roadmap
        // WaypointCallback(ConvertAutowarePathWithLaneIdToNavPath(reference_path));

        if (config_->debug_output_)
                LOG_SUCCESS("Published reference path!");
}

double RouteConverter::getLineLength(lanelet::ConstLineString2d line)
{
        double length = 0.;
        for (size_t i = 0; i < line.size() - 1; i++)
        {
                length += lanelet::geometry::distance2d(
                    line[i].basicPoint2d(),
                    line[i + 1].basicPoint2d());
        }
        return length;
}

void RouteConverter::AddRoadBoundaries(
    PathWithLaneId &path,
    const std::vector<lanelet::ConstLanelet> &lanelet_sequence)
{

        // Go throught the left boundary of the lanelet and add its points
        std::vector<geometry_msgs::msg::Point> left_points;
        for (auto &lanelet : lanelet_sequence)
        {
                // First get the left most lanelet
                // auto left_most_lanelet = route_handler_->getAllSharedLineStringLanelets(lanelet, false, true, true);

                lanelet::ConstLineString2d bound = lanelet.leftBound2d(); // I will assume left most is 0 index?
                for (auto &point : bound)
                {
                        left_points.push_back(lanelet::utils::conversion::toGeomMsgPt(point));
                }
        }
        path.left_bound = left_points;

        // Same for the right boundary
        std::vector<geometry_msgs::msg::Point> right_points;
        for (auto &lanelet : lanelet_sequence)
        {
                // Get opposite lanes if they exist
                auto right_most_lanelet = route_handler_->getAllRightSharedLinestringLanelets(
                    lanelet, true,
                    true);

                // For the opposite lanes, invert it then take the left bound? Or should it be the right bound
                lanelet::ConstLineString2d bound = right_most_lanelet.size() > 0 ? right_most_lanelet[0].rightBound2d() : // (is already inverted!)
                                                       lanelet.rightBound2d();

                for (auto &point : bound)
                {
                        right_points.push_back(lanelet::utils::conversion::toGeomMsgPt(point));
                }
        }
        path.right_bound = right_points;
}

// void RouteConverter::WaypointCallback(const nav_msgs::msg::Path &msg)
// {
//         LOG_MARK("Received waypoints");
//         Way new_way; // Create a way object
//         for (size_t i = 0; i < msg.poses.size(); i++)
//         {
//                 new_way.AddNode(RoadNode(
//                     msg.poses[i].pose.position.x,
//                     msg.poses[i].pose.position.y,
//                     RosTools::quaternionToAngle(msg.poses[i].pose)));
//         }
//         int id = 0;
//         new_way.AddLane("road", 4.0, true, id); // Add a lane
//         auto &lane = new_way.lanes.back();

//         // SplineFitter spline_fitter;
//         // spline_fitter.Initialize(config_.get());
//         // spline_fitter.FitSplineOnLane(lane); // Fit a spline on the received waypoints

//         auto ref_msg = std::make_shared<nav_msgs::msg::Path>();
//         ref_msg->poses.reserve(lane.nodes.size());

//         geometry_msgs::msg::PoseStamped pose_msg;
//         pose_msg.header.stamp = rclcpp::Clock().now();
//         pose_msg.header.frame_id = "map";

//         for (RoadNode &node : lane.nodes)
//         {
//                 pose_msg.pose.position.x = node.x;
//                 pose_msg.pose.position.y = node.y;
//                 pose_msg.pose.orientation = RosTools::angleToQuaternion(node.theta);
//                 ref_msg->poses.push_back(pose_msg);
//         }

//         ref_msg->header.stamp = rclcpp::Clock().now();
//         ref_msg->header.frame_id = "map";

//         reference_pub_->publish(*ref_msg); // publish
// }

void RouteConverter::visualize(PathWithLaneId &reference_path)
{
        auto &publisher = VISUALS.getPublisher("points");
        auto &draw_point = publisher.getNewPointMarker("CUBE");
        draw_point.setScale(0.25, 0.25, 0.25);
        draw_point.setColor(1., 0., 0.);

        for (size_t i = 0; i < reference_path.points.size(); i++)
        {
                draw_point.setColor(0., 0., 0., 1.);
                draw_point.addPointMarker(reference_path.points[i].point.pose);

                // Boundaries cannot be drawn like this (the x is the spline length)
                // draw_point.setColor(1., 0., 0., 1.);
                // draw_point.addPointMarker(Eigen::Vector2d(reference_path.left_bound[i].x,
                //                                           reference_path.left_bound[i].y));
                // draw_point.addPointMarker(Eigen::Vector2d(reference_path.right_bound[i].x,
                //                                           reference_path.right_bound[i].y));
        }
        publisher.publish();
}