/**
 * @file student_cmd.cpp
 * @brief Mimics a student laptop publishing waypoints through the fencer.
 *
 * Cycles through a list of waypoints, publishing PositionTarget to
 *   /student/droneXX/setpoint_raw/local
 * at a configurable rate.  One waypoint is intentionally outside the
 * fence so the safety_fencer should block it and log a warning.
 *
 * Parameters (via sim_params.yaml):
 *   drone_id      – which drone to command
 *   rate_hz       – publish rate
 *   hold_time_sec – seconds to hold each waypoint before advancing
 *   waypoints     – list of {x, y, z}
 */

#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/position_target.hpp>

#include <string>
#include <vector>

struct Waypoint { double x, y, z; };

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("student_cmd");

    node->declare_parameter("drone_id", 10);
    node->declare_parameter("rate_hz", 20.0);
    node->declare_parameter("hold_time_sec", 4.0);
    node->declare_parameter("waypoints", std::vector<double>{});

    const int drone_id       = node->get_parameter("drone_id").as_int();
    const double rate_hz     = node->get_parameter("rate_hz").as_double();
    const double hold_sec    = node->get_parameter("hold_time_sec").as_double();
    const auto wp_flat       = node->get_parameter("waypoints").as_double_array();

    // Parse flat array [x1,y1,z1, x2,y2,z2, ...] into Waypoint vector
    std::vector<Waypoint> waypoints;
    for (size_t i = 0; i + 2 < wp_flat.size(); i += 3) {
        waypoints.push_back({wp_flat[i], wp_flat[i + 1], wp_flat[i + 2]});
    }
    if (waypoints.empty()) {
        RCLCPP_ERROR(node->get_logger(), "No waypoints loaded — exiting.");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(),
        "[StudentCmd] drone%d  %zu waypoints  rate=%.0f Hz  hold=%.1f s",
        drone_id, waypoints.size(), rate_hz, hold_sec);

    const std::string topic =
        "/student/drone" + std::to_string(drone_id) + "/setpoint_raw/local";
    auto pub = node->create_publisher<mavros_msgs::msg::PositionTarget>(topic, 10);

    size_t wp_idx = 0;
    auto wp_start = node->now();

    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / rate_hz)),
        [&]() {
            // Advance waypoint after hold_sec
            if ((node->now() - wp_start).seconds() >= hold_sec) {
                wp_idx = (wp_idx + 1) % waypoints.size();
                wp_start = node->now();
                const auto& wp = waypoints[wp_idx];
                RCLCPP_INFO(node->get_logger(),
                    "[StudentCmd] -> waypoint %zu  (%.1f, %.1f, %.1f)",
                    wp_idx, wp.x, wp.y, wp.z);
            }

            const auto& wp = waypoints[wp_idx];

            mavros_msgs::msg::PositionTarget msg;
            msg.header.stamp = node->now();
            msg.coordinate_frame =
                mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
            msg.type_mask =
                mavros_msgs::msg::PositionTarget::IGNORE_VX |
                mavros_msgs::msg::PositionTarget::IGNORE_VY |
                mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
            msg.position.x = wp.x;
            msg.position.y = wp.y;
            msg.position.z = wp.z;
            msg.yaw = 0.0f;

            pub->publish(msg);
        });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
