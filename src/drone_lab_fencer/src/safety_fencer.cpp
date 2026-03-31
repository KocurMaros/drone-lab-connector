/**
 * @file safety_fencer.cpp
 * @brief ROS 2 Humble geofencing middleman node for a multi-drone indoor lab.
 *
 * Sits between student command publishers and the real MAVROS setpoint topics.
 * Every incoming PositionTarget is checked against a configurable 3-D bounding
 * box loaded from YAML parameters.  Messages outside the fence are dropped and
 * the drone is commanded to hold its current position.
 *
 * Subscriptions (per drone):
 *   /student/droneXX/setpoint_raw/local  (mavros_msgs/msg/PositionTarget)
 *   /droneXX/mavros/local_position/pose  (geometry_msgs/msg/PoseStamped)
 *
 * Publications (per drone):
 *   /droneXX/mavros/setpoint_raw/local   (mavros_msgs/msg/PositionTarget)
 */

#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <string>
#include <vector>

// -------------------------------------------------------------------
struct FenceBounds {
    double x_min, x_max;
    double y_min, y_max;
    double z_min, z_max;
};

// -------------------------------------------------------------------
class DroneProxy {
public:
    DroneProxy(rclcpp::Node::SharedPtr node, int drone_id,
               const FenceBounds& fence)
        : node_(node),
          drone_id_(drone_id),
          ns_("drone" + std::to_string(drone_id)),
          fence_(fence),
          has_pose_(false)
    {
        // Publisher -> real MAVROS setpoint
        mavros_pub_ = node_->create_publisher<mavros_msgs::msg::PositionTarget>(
            "/" + ns_ + "/mavros/setpoint_raw/local", 1);

        // Subscriber <- drone's local position (OptiTrack via MAVROS)
        pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/" + ns_ + "/mavros/local_position/pose", 1,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                current_pose_ = *msg;
                has_pose_ = true;
            });

        // Subscriber <- student command
        student_sub_ = node_->create_subscription<mavros_msgs::msg::PositionTarget>(
            "/student/" + ns_ + "/setpoint_raw/local", 1,
            [this](mavros_msgs::msg::PositionTarget::SharedPtr msg) {
                studentCmdCb(msg);
            });

        RCLCPP_INFO(node_->get_logger(),
            "[Fencer] Proxy ready for %s  "
            "(fence X[%.1f,%.1f] Y[%.1f,%.1f] Z[%.1f,%.1f])",
            ns_.c_str(),
            fence_.x_min, fence_.x_max,
            fence_.y_min, fence_.y_max,
            fence_.z_min, fence_.z_max);
    }

private:
    void studentCmdCb(const mavros_msgs::msg::PositionTarget::SharedPtr& msg) {
        const double x = msg->position.x;
        const double y = msg->position.y;
        const double z = msg->position.z;

        if (x >= fence_.x_min && x <= fence_.x_max &&
            y >= fence_.y_min && y <= fence_.y_max &&
            z >= fence_.z_min && z <= fence_.z_max)
        {
            mavros_pub_->publish(*msg);
        } else {
            RCLCPP_WARN(node_->get_logger(),
                "[Fencer] %s BLOCKED setpoint (%.2f, %.2f, %.2f) "
                "-- outside fence!", ns_.c_str(), x, y, z);
            publishHold();
        }
    }

    void publishHold() {
        if (!has_pose_) {
            RCLCPP_WARN(node_->get_logger(),
                "[Fencer] %s has no pose yet -- cannot publish hold.",
                ns_.c_str());
            return;
        }

        mavros_msgs::msg::PositionTarget hold;
        hold.header.stamp = node_->now();
        hold.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        hold.type_mask =
            mavros_msgs::msg::PositionTarget::IGNORE_VX  |
            mavros_msgs::msg::PositionTarget::IGNORE_VY  |
            mavros_msgs::msg::PositionTarget::IGNORE_VZ  |
            mavros_msgs::msg::PositionTarget::IGNORE_AFX |
            mavros_msgs::msg::PositionTarget::IGNORE_AFY |
            mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
            mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

        hold.position.x = current_pose_.pose.position.x;
        hold.position.y = current_pose_.pose.position.y;
        hold.position.z = current_pose_.pose.position.z;

        mavros_pub_->publish(hold);
    }

    // -- members -------------------------------------------------------
    rclcpp::Node::SharedPtr node_;
    int         drone_id_;
    std::string ns_;
    FenceBounds fence_;
    bool        has_pose_;

    geometry_msgs::msg::PoseStamped current_pose_;

    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr  mavros_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<mavros_msgs::msg::PositionTarget>::SharedPtr student_sub_;
};

// ===================================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("safety_fencer");

    // -- Declare and load fence parameters from YAML ------------------
    node->declare_parameter("fence.x_min", -2.0);
    node->declare_parameter("fence.x_max",  2.0);
    node->declare_parameter("fence.y_min", -2.0);
    node->declare_parameter("fence.y_max",  2.0);
    node->declare_parameter("fence.z_min",  0.1);
    node->declare_parameter("fence.z_max",  2.5);
    node->declare_parameter("drone_id_min", 10);
    node->declare_parameter("drone_id_max", 19);

    FenceBounds fence{};
    fence.x_min = node->get_parameter("fence.x_min").as_double();
    fence.x_max = node->get_parameter("fence.x_max").as_double();
    fence.y_min = node->get_parameter("fence.y_min").as_double();
    fence.y_max = node->get_parameter("fence.y_max").as_double();
    fence.z_min = node->get_parameter("fence.z_min").as_double();
    fence.z_max = node->get_parameter("fence.z_max").as_double();

    const int drone_id_min = node->get_parameter("drone_id_min").as_int();
    const int drone_id_max = node->get_parameter("drone_id_max").as_int();

    RCLCPP_INFO(node->get_logger(),
        "[Fencer] Bounding box: X[%.1f,%.1f] Y[%.1f,%.1f] Z[%.1f,%.1f]",
        fence.x_min, fence.x_max,
        fence.y_min, fence.y_max,
        fence.z_min, fence.z_max);
    RCLCPP_INFO(node->get_logger(),
        "[Fencer] Monitoring drones %d -- %d", drone_id_min, drone_id_max);

    // -- Create one proxy per drone ------------------------------------
    std::vector<std::unique_ptr<DroneProxy>> proxies;
    for (int id = drone_id_min; id <= drone_id_max; ++id) {
        proxies.push_back(std::make_unique<DroneProxy>(node, id, fence));
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
