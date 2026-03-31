/**
 * @file fake_drone.cpp
 * @brief Lightweight MAVROS-stub simulator for testing drone_lab_fencer.
 *
 * For each configured drone ID this node:
 *   - Publishes  /droneXX/mavros/local_position/pose   at ~30 Hz (fake OptiTrack)
 *   - Publishes  /droneXX/mavros/state                 at ~1 Hz
 *   - Subscribes /droneXX/mavros/setpoint_raw/local    and slowly moves the
 *     simulated pose toward the setpoint
 *   - Advertises /droneXX/mavros/cmd/arming             (always succeeds)
 *   - Advertises /droneXX/mavros/set_mode               (always succeeds)
 *
 * Parameters (via sim_params.yaml):
 *   drone_id_min, drone_id_max   – range of SysIDs to simulate
 *   pose_rate                    – Hz for pose publishing
 *   drift_speed                  – m per tick the sim pose moves toward setpoint
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// -------------------------------------------------------------------
class FakeDrone {
public:
    FakeDrone(rclcpp::Node::SharedPtr node, int drone_id, double drift_speed)
        : node_(node),
          ns_("drone" + std::to_string(drone_id)),
          drift_(drift_speed),
          armed_(false),
          mode_("STABILIZE")
    {
        // Start at origin + slight offset per drone so they don't overlap
        pose_.pose.position.x = 0.0;
        pose_.pose.position.y = static_cast<double>(drone_id - 10) * 0.5;
        pose_.pose.position.z = 0.0;
        pose_.pose.orientation.w = 1.0;

        target_x_ = pose_.pose.position.x;
        target_y_ = pose_.pose.position.y;
        target_z_ = pose_.pose.position.z;

        // Publishers
        pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/" + ns_ + "/mavros/local_position/pose", 10);
        state_pub_ = node_->create_publisher<mavros_msgs::msg::State>(
            "/" + ns_ + "/mavros/state", 10);

        // Subscriber (setpoint from fencer or takeoff_service)
        setpoint_sub_ = node_->create_subscription<mavros_msgs::msg::PositionTarget>(
            "/" + ns_ + "/mavros/setpoint_raw/local", 10,
            [this](mavros_msgs::msg::PositionTarget::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(mu_);
                target_x_ = msg->position.x;
                target_y_ = msg->position.y;
                target_z_ = msg->position.z;
            });

        // Services
        arm_srv_ = node_->create_service<mavros_msgs::srv::CommandBool>(
            "/" + ns_ + "/mavros/cmd/arming",
            [this](const std::shared_ptr<mavros_msgs::srv::CommandBool::Request> req,
                   std::shared_ptr<mavros_msgs::srv::CommandBool::Response> res) {
                std::lock_guard<std::mutex> lk(mu_);
                armed_ = req->value;
                res->success = true;
                res->result = 0;
                RCLCPP_INFO(node_->get_logger(), "[Sim %s] %s",
                            ns_.c_str(), armed_ ? "ARMED" : "DISARMED");
            });

        mode_srv_ = node_->create_service<mavros_msgs::srv::SetMode>(
            "/" + ns_ + "/mavros/set_mode",
            [this](const std::shared_ptr<mavros_msgs::srv::SetMode::Request> req,
                   std::shared_ptr<mavros_msgs::srv::SetMode::Response> res) {
                std::lock_guard<std::mutex> lk(mu_);
                mode_ = req->custom_mode;
                res->mode_sent = true;
                RCLCPP_INFO(node_->get_logger(), "[Sim %s] Mode -> %s",
                            ns_.c_str(), mode_.c_str());
            });

        RCLCPP_INFO(node_->get_logger(),
            "[Sim] Fake drone %s ready at (%.1f, %.1f, %.1f)",
            ns_.c_str(),
            pose_.pose.position.x,
            pose_.pose.position.y,
            pose_.pose.position.z);
    }

    /** Called from the timer callback to advance the simulation one tick. */
    void tick() {
        std::lock_guard<std::mutex> lk(mu_);

        // Move toward setpoint only if armed
        if (armed_) {
            auto& p = pose_.pose.position;
            const double dx = target_x_ - p.x;
            const double dy = target_y_ - p.y;
            const double dz = target_z_ - p.z;
            const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (dist > 0.01) {
                const double step = std::min(drift_, dist);
                p.x += dx / dist * step;
                p.y += dy / dist * step;
                p.z += dz / dist * step;
            }
        }

        // Publish pose
        pose_.header.stamp = node_->now();
        pose_.header.frame_id = "map";
        pose_pub_->publish(pose_);
    }

    /** Called at ~1 Hz to publish state. */
    void publishState() {
        std::lock_guard<std::mutex> lk(mu_);
        mavros_msgs::msg::State st;
        st.header.stamp = node_->now();
        st.connected = true;
        st.armed     = armed_;
        st.guided    = (mode_ == "GUIDED" || mode_ == "OFFBOARD");
        st.mode      = mode_;
        state_pub_->publish(st);
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::string ns_;
    double drift_;
    bool armed_;
    std::string mode_;
    std::mutex mu_;

    geometry_msgs::msg::PoseStamped pose_;
    double target_x_, target_y_, target_z_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    pose_pub_;
    rclcpp::Publisher<mavros_msgs::msg::State>::SharedPtr            state_pub_;
    rclcpp::Subscription<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_sub_;
    rclcpp::Service<mavros_msgs::srv::CommandBool>::SharedPtr        arm_srv_;
    rclcpp::Service<mavros_msgs::srv::SetMode>::SharedPtr            mode_srv_;
};

// ===================================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("fake_drone");

    node->declare_parameter("drone_id_min", 10);
    node->declare_parameter("drone_id_max", 11);
    node->declare_parameter("pose_rate", 30.0);
    node->declare_parameter("drift_speed", 0.02);

    const int id_min     = node->get_parameter("drone_id_min").as_int();
    const int id_max     = node->get_parameter("drone_id_max").as_int();
    const double rate_hz = node->get_parameter("pose_rate").as_double();
    const double drift   = node->get_parameter("drift_speed").as_double();

    RCLCPP_INFO(node->get_logger(),
        "[Sim] Spawning fake drones %d -- %d  (rate=%.0f Hz, drift=%.3f m/tick)",
        id_min, id_max, rate_hz, drift);

    std::vector<std::shared_ptr<FakeDrone>> drones;
    for (int id = id_min; id <= id_max; ++id) {
        drones.push_back(std::make_shared<FakeDrone>(node, id, drift));
    }

    // Pose timer
    auto pose_timer = node->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / rate_hz)),
        [&drones]() {
            for (auto& d : drones) d->tick();
        });

    // State timer (1 Hz)
    auto state_timer = node->create_wall_timer(
        std::chrono::seconds(1),
        [&drones]() {
            for (auto& d : drones) d->publishState();
        });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
