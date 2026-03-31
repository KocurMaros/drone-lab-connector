/**
 * @file takeoff_service.cpp
 * @brief ROS 2 Humble per-drone safe arm-and-takeoff service for an indoor lab.
 *
 * Advertises  /student/droneXX/takeoff  (std_srvs/srv/Trigger) for each drone.
 * Uses the "setpoint-before-arm" pattern:
 *   1. Verify a recent local_position/pose exists (OptiTrack alive).
 *   2. Stream a target setpoint at current XY + desired Z altitude.
 *   3. Switch to GUIDED (ArduPilot) or OFFBOARD (PX4) mode.
 *   4. Arm the vehicle.
 *   5. Continue streaming until the service returns.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

// -------------------------------------------------------------------
class DroneTakeoffHandler {
public:
    static constexpr double POSE_TIMEOUT_SEC     = 2.0;
    static constexpr double SETPOINT_STREAM_HZ   = 20.0;
    static constexpr double SETPOINT_PREFILL_SEC = 2.0;
    static constexpr double POST_ARM_STREAM_SEC  = 3.0;

    DroneTakeoffHandler(rclcpp::Node::SharedPtr node, int drone_id,
                        double takeoff_alt, double fence_z_min)
        : node_(node),
          drone_id_(drone_id),
          ns_("drone" + std::to_string(drone_id)),
          takeoff_alt_(std::max(takeoff_alt, fence_z_min)),
          has_pose_(false)
    {
        // Subscribers
        pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/" + ns_ + "/mavros/local_position/pose", 1,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mu_);
                current_pose_ = *msg;
                pose_stamp_   = node_->now();
                has_pose_     = true;
            });

        state_sub_ = node_->create_subscription<mavros_msgs::msg::State>(
            "/" + ns_ + "/mavros/state", 1,
            [this](mavros_msgs::msg::State::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mu_);
                current_state_ = *msg;
            });

        // Publisher
        setpoint_pub_ = node_->create_publisher<mavros_msgs::msg::PositionTarget>(
            "/" + ns_ + "/mavros/setpoint_raw/local", 1);

        // Service clients (MAVROS)
        arm_client_ = node_->create_client<mavros_msgs::srv::CommandBool>(
            "/" + ns_ + "/mavros/cmd/arming");
        mode_client_ = node_->create_client<mavros_msgs::srv::SetMode>(
            "/" + ns_ + "/mavros/set_mode");

        // Advertise student-facing service
        service_ = node_->create_service<std_srvs::srv::Trigger>(
            "/student/" + ns_ + "/takeoff",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                handleTakeoff(req, res);
            });

        RCLCPP_INFO(node_->get_logger(),
            "[Takeoff] Service ready: /student/%s/takeoff  (alt=%.2f m)",
            ns_.c_str(), takeoff_alt_);
    }

private:
    // -- Helpers -------------------------------------------------------
    mavros_msgs::msg::PositionTarget makeTarget(double x, double y, double z) {
        mavros_msgs::msg::PositionTarget t;
        t.header.stamp       = node_->now();
        t.coordinate_frame   = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        t.type_mask =
            mavros_msgs::msg::PositionTarget::IGNORE_VX  |
            mavros_msgs::msg::PositionTarget::IGNORE_VY  |
            mavros_msgs::msg::PositionTarget::IGNORE_VZ  |
            mavros_msgs::msg::PositionTarget::IGNORE_AFX |
            mavros_msgs::msg::PositionTarget::IGNORE_AFY |
            mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
            mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
        t.position.x = x;
        t.position.y = y;
        t.position.z = z;
        return t;
    }

    void streamSetpoints(double x, double y, double z, double duration_sec) {
        const auto period = std::chrono::milliseconds(
            static_cast<int>(1000.0 / SETPOINT_STREAM_HZ));
        const auto end_time = node_->now() + rclcpp::Duration::from_seconds(duration_sec);

        while (rclcpp::ok() && node_->now() < end_time) {
            setpoint_pub_->publish(makeTarget(x, y, z));
            std::this_thread::sleep_for(period);
        }
    }

    // -- Service handler ------------------------------------------------
    void handleTakeoff(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        geometry_msgs::msg::PoseStamped pose;
        rclcpp::Time stamp(0, 0, RCL_ROS_TIME);

        {
            std::lock_guard<std::mutex> lock(mu_);
            if (!has_pose_) {
                res->success = false;
                res->message = ns_ + ": No local position received -- is OptiTrack running?";
                RCLCPP_WARN(node_->get_logger(), "[Takeoff] %s", res->message.c_str());
                return;
            }
            pose  = current_pose_;
            stamp = pose_stamp_;
        }

        const double age = (node_->now() - stamp).seconds();
        if (age > POSE_TIMEOUT_SEC) {
            res->success = false;
            res->message = ns_ + ": Pose is stale (" +
                           std::to_string(age) + "s old). Check OptiTrack.";
            RCLCPP_WARN(node_->get_logger(), "[Takeoff] %s", res->message.c_str());
            return;
        }

        const double px = pose.pose.position.x;
        const double py = pose.pose.position.y;

        // -- 1. Pre-fill the setpoint buffer ----------------------------
        RCLCPP_INFO(node_->get_logger(),
            "[Takeoff] %s streaming prefill setpoints ...", ns_.c_str());
        streamSetpoints(px, py, takeoff_alt_, SETPOINT_PREFILL_SEC);

        // -- 2. Switch to GUIDED (ArduPilot) / OFFBOARD (PX4) ----------
        {
            auto mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            mode_req->custom_mode = "GUIDED";

            auto future = mode_client_->async_send_request(mode_req);
            if (future.wait_for(3s) != std::future_status::ready ||
                !future.get()->mode_sent)
            {
                RCLCPP_WARN(node_->get_logger(),
                    "[Takeoff] %s GUIDED not acknowledged, trying OFFBOARD ...",
                    ns_.c_str());
                mode_req->custom_mode = "OFFBOARD";
                auto future2 = mode_client_->async_send_request(mode_req);
                if (future2.wait_for(3s) != std::future_status::ready ||
                    !future2.get()->mode_sent)
                {
                    res->success = false;
                    res->message = ns_ + ": set_mode service call failed.";
                    RCLCPP_ERROR(node_->get_logger(), "[Takeoff] %s", res->message.c_str());
                    return;
                }
            }
        }

        // -- 3. Arm -----------------------------------------------------
        {
            auto arm_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            arm_req->value = true;

            auto future = arm_client_->async_send_request(arm_req);
            if (future.wait_for(3s) != std::future_status::ready ||
                !future.get()->success)
            {
                res->success = false;
                res->message = ns_ + ": Arming rejected by FCU.";
                RCLCPP_WARN(node_->get_logger(), "[Takeoff] %s", res->message.c_str());
                return;
            }
        }

        // -- 4. Continue streaming the takeoff setpoint -----------------
        streamSetpoints(px, py, takeoff_alt_, POST_ARM_STREAM_SEC);

        res->success = true;
        res->message = ns_ + ": Armed and climbing to " +
                       std::to_string(takeoff_alt_) + " m.";
        RCLCPP_INFO(node_->get_logger(), "[Takeoff] %s", res->message.c_str());
    }

    // -- members -------------------------------------------------------
    rclcpp::Node::SharedPtr node_;
    int         drone_id_;
    std::string ns_;
    double      takeoff_alt_;
    bool        has_pose_;
    std::mutex  mu_;

    geometry_msgs::msg::PoseStamped current_pose_;
    rclcpp::Time                    pose_stamp_{0, 0, RCL_ROS_TIME};
    mavros_msgs::msg::State         current_state_;

    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr   setpoint_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr         state_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr         arm_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr             mode_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr               service_;
};

// ===================================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("takeoff_service");

    node->declare_parameter("drone_id_min", 10);
    node->declare_parameter("drone_id_max", 19);
    node->declare_parameter("fence_z_min",  0.1);
    node->declare_parameter("takeoff_altitude", 1.0);

    const int drone_id_min = node->get_parameter("drone_id_min").as_int();
    const int drone_id_max = node->get_parameter("drone_id_max").as_int();
    const double fence_z_min = node->get_parameter("fence_z_min").as_double();
    const double takeoff_alt = node->get_parameter("takeoff_altitude").as_double();

    RCLCPP_INFO(node->get_logger(),
        "[Takeoff] Registering services for drones %d -- %d (alt=%.2f m)",
        drone_id_min, drone_id_max, takeoff_alt);

    std::vector<std::unique_ptr<DroneTakeoffHandler>> handlers;
    for (int id = drone_id_min; id <= drone_id_max; ++id) {
        handlers.push_back(
            std::make_unique<DroneTakeoffHandler>(node, id, takeoff_alt, fence_z_min));
    }

    // Multi-threaded executor so service callbacks don't block subscriptions
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
