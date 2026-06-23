#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

class DiffDriveNode : public rclcpp::Node
{
public:
    DiffDriveNode() : Node("diff_drive")
    {
        const std::array<std::string, 4> positions = {
            "front_left", "front_right", "rear_left", "rear_right"
        };

        const std::string drive_type = declare_parameter<std::string>("drive_type", "pwm");
        velocity_mode_ = (drive_type == "velocity");
        max_rpm_ = declare_parameter("max_rpm", 80.0);

        for (size_t i = 0; i < 4; ++i) {
            const auto & pos   = positions[i];
            std::string titan  = declare_parameter<std::string>(pos + ".titan", "drive");
            int         port   = declare_parameter<int>(pos + ".port", static_cast<int>(i));

            const std::string suffix = velocity_mode_ ? "/rpm_cmd" : "/cmd";
            std::string topic = "/" + titan + "/m_" + std::to_string(port) + suffix;
            motor_pubs_[i] = create_publisher<std_msgs::msg::Float64>(topic, 1);
            RCLCPP_INFO(get_logger(), "%s -> %s", pos.c_str(), topic.c_str());
        }

        if (velocity_mode_) {
            RCLCPP_INFO(get_logger(),
                "drive_type=velocity max_rpm=%.1f - set studica_params default_pid_type to 2 (mcv2) "
                "or 1 (legacy)",
                max_rpm_);
        } else {
            RCLCPP_INFO(get_logger(), "drive_type=pwm (open-loop duty)");
        }

        max_linear_  = declare_parameter("max_linear",  1.0);
        max_angular_ = declare_parameter("max_angular", 2.0);

        int publish_rate = declare_parameter("publish_rate", 50); // 50 Hz

        // Jerk limiting — max duty cycle change per timer tick.
        // At 50 Hz, accel_rate=0.05 → full speed reached in 20 ticks = 400 ms.
        // Set to 1.0 to disable ramping (instant response).
        accel_rate_ = declare_parameter("accel_rate", 0.05);

        std::string cmd_topic = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");

        // cmd_vel callback only stores the latest values — does NOT publish immediately.
        // This prevents flooding the Titan's CAN buffer when cmd_vel arrives quickly.
        sub_ = create_subscription<geometry_msgs::msg::Twist>(
            cmd_topic, 1,
            std::bind(&DiffDriveNode::on_cmd_vel, this, std::placeholders::_1));

        // Timer drains the latest stored command to the Titan at a fixed rate.
        // Always sends the most recent value — stale commands never accumulate.
        auto period = std::chrono::duration<double>(1.0 / publish_rate);
        timer_ = create_wall_timer(period,
            std::bind(&DiffDriveNode::on_timer, this));

        RCLCPP_INFO(get_logger(), "publish_rate: %d Hz", publish_rate);
    }

private:
    void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Store latest velocities — the timer will send them at the fixed rate.
        latest_linear_  = msg->linear.x;
        latest_angular_ = msg->angular.z;
    }

    void on_timer()
    {
        // Ramp current duty toward target — limits jerk, reduces mecanum roller slip.
        double target_left  = (latest_linear_ / max_linear_) - (latest_angular_ / max_angular_);
        double target_right = (latest_linear_ / max_linear_) + (latest_angular_ / max_angular_);

        // Ratio-preserving clamp on target before ramping.
        double peak = std::max({std::abs(target_left), std::abs(target_right), 1.0});
        target_left  /= peak;
        target_right /= peak;

        // Step current duty toward target by at most accel_rate_ per tick.
        auto ramp = [this](double current, double target) {
            double diff = target - current;
            double step = std::clamp(diff, -accel_rate_, accel_rate_);
            return current + step;
        };
        current_left_  = ramp(current_left_,  target_left);
        current_right_ = ramp(current_right_, target_right);

        if (velocity_mode_) {
            publish_rpm(0, current_left_  * max_rpm_); // front_left
            publish_rpm(1, current_right_ * max_rpm_); // front_right
            publish_rpm(2, current_left_  * max_rpm_); // rear_left
            publish_rpm(3, current_right_ * max_rpm_); // rear_right
        } else {
            publish_duty(0, current_left_);  // front_left
            publish_duty(1, current_right_); // front_right
            publish_duty(2, current_left_);  // rear_left
            publish_duty(3, current_right_); // rear_right
        }
    }

    void publish_duty(size_t idx, double duty)
    {
        std_msgs::msg::Float64 msg;
        msg.data = std::clamp(duty, -1.0, 1.0);
        motor_pubs_[idx]->publish(msg);
    }

    void publish_rpm(size_t idx, double rpm)
    {
        std_msgs::msg::Float64 msg;
        msg.data = rpm;
        motor_pubs_[idx]->publish(msg);
    }

    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 4> motor_pubs_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool velocity_mode_ = false;
    double max_rpm_;
    double max_linear_;
    double max_angular_;
    double accel_rate_     = 0.05;
    double latest_linear_  = 0.0;
    double latest_angular_ = 0.0;
    double current_left_   = 0.0;  // ramped duty cycle currently being sent
    double current_right_  = 0.0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffDriveNode>());
    rclcpp::shutdown();
    return 0;
}
