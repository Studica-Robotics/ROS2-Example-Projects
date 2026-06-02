#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

class MecanumDriveNode : public rclcpp::Node
{
public:
    MecanumDriveNode() : Node("mecanum_drive")
    {
        const std::array<std::string, 4> positions = {
            "front_left", "front_right", "rear_left", "rear_right"
        };

        for (size_t i = 0; i < 4; ++i) {
            const auto & pos   = positions[i];
            std::string titan  = declare_parameter<std::string>(pos + ".titan", "drive");
            int         port   = declare_parameter<int>(pos + ".port", static_cast<int>(i));
            std::string topic  = "/" + titan + "/m_" + std::to_string(port) + "/cmd";

            motor_pubs_[i] = create_publisher<std_msgs::msg::Float64>(topic, 1);
            RCLCPP_INFO(get_logger(), "%s -> %s", pos.c_str(), topic.c_str());
        }

        max_linear_  = declare_parameter("max_linear",  1.0);
        max_angular_ = declare_parameter("max_angular", 2.0);

        int publish_rate = declare_parameter("publish_rate", 50); // 50 hz

        // Jerk limiting — ramp vx/vy/wz inputs at most accel_rate_ per tick.
        // Ramping the inputs (not individual motors) preserves holonomic kinematics
        // throughout the ramp: the robot always moves in the correct direction.
        // At 50 Hz: 0.05 → full speed in 400 ms. Set to 1.0 to disable.
        accel_rate_ = declare_parameter("accel_rate", 0.05);

        std::string cmd_topic = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");

        // cmd_vel only stores the latest values — does NOT publish immediately.
        sub_ = create_subscription<geometry_msgs::msg::Twist>(
            cmd_topic, 1,
            std::bind(&MecanumDriveNode::on_cmd_vel, this, std::placeholders::_1));

        // Timer drains the latest stored command to the Titan at a fixed rate.
        auto period = std::chrono::duration<double>(1.0 / publish_rate);
        timer_ = create_wall_timer(period,
            std::bind(&MecanumDriveNode::on_timer, this));

        RCLCPP_INFO(get_logger(), "mecanum_drive ready  publish_rate=%d Hz  accel_rate=%.2f",
            publish_rate, accel_rate_);
    }

private:
    void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        latest_vx_ = msg->linear.x;
        latest_vy_ = msg->linear.y;
        latest_wz_ = msg->angular.z;
    }

    void on_timer()
    {
        // Ramp inputs toward target — preserves holonomic kinematics during acceleration.
        auto ramp = [this](double current, double target) {
            double diff = target - current;
            double step = std::clamp(diff, -accel_rate_, accel_rate_);
            return current + step;
        };
        current_vx_ = ramp(current_vx_, latest_vx_ / max_linear_);
        current_vy_ = ramp(current_vy_, latest_vy_ / max_linear_);
        current_wz_ = ramp(current_wz_, latest_wz_ / max_angular_);

        // Mecanum holonomic kinematics
        double fl = current_vx_ - current_vy_ - current_wz_;
        double fr = current_vx_ + current_vy_ + current_wz_;
        double rl = current_vx_ + current_vy_ - current_wz_;
        double rr = current_vx_ - current_vy_ + current_wz_;

        // Ratio-preserving clamp: scale all four together so no axis is lost.
        double peak = std::max({std::abs(fl), std::abs(fr), std::abs(rl), std::abs(rr), 1.0});
        fl /= peak;
        fr /= peak;
        rl /= peak;
        rr /= peak;

        publish(0, fl);  // front_left
        publish(1, fr);  // front_right
        publish(2, rl);  // rear_left
        publish(3, rr);  // rear_right
    }

    void publish(size_t idx, double duty)
    {
        std_msgs::msg::Float64 msg;
        msg.data = std::clamp(duty, -1.0, 1.0);
        motor_pubs_[idx]->publish(msg);
    }

    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 4> motor_pubs_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double max_linear_;
    double max_angular_;
    double accel_rate_   = 0.05;

    // Latest cmd_vel input (normalized in on_timer)
    double latest_vx_    = 0.0;
    double latest_vy_    = 0.0;
    double latest_wz_    = 0.0;

    // Ramped normalized values currently being sent to motors
    double current_vx_   = 0.0;
    double current_vy_   = 0.0;
    double current_wz_   = 0.0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MecanumDriveNode>());
    rclcpp::shutdown();
    return 0;
}
