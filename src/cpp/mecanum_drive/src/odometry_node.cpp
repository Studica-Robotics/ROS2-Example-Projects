#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <array>
#include <cmath>
#include <string>

// Mecanum inverse kinematics (all encoder values positive = wheel moves forward):
//   Position:  dx_body = (fl+fr+rl+rr)/4    dy_body = (-fl+fr+rl-rr)/4
//   Velocity:  vx_body = (fl+fr+rl+rr)/4    vy_body = (-fl+fr+rl-rr)/4   (from RPM)
//   Angular:   ω       = (-fl+fr-rl+rr) / (2*(track_width+wheel_base))   (from RPM)
//
// ω is encoder-derived and independent of the IMU — the EKF fuses both sources
// and can detect wheel slip (encoders ≠ gyro) or IMU drift (gyro ≠ encoders).
//
// Requires invert_encoder: true for front_right and rear_right in studica_control.

class OdometryNode : public rclcpp::Node
{
public:
    OdometryNode() : Node("odometry")
    {
        const std::array<std::string, 4> positions = {
            "front_left", "front_right", "rear_left", "rear_right"
        };

        for (size_t i = 0; i < 4; ++i) {
            const auto & pos  = positions[i];
            std::string titan = declare_parameter<std::string>(pos + ".titan", "drive");
            int         port  = declare_parameter<int>(pos + ".port", static_cast<int>(i));

            std::string enc_topic = "/" + titan + "/m_" + std::to_string(port) + "/encoder";
            std::string rpm_topic = "/" + titan + "/m_" + std::to_string(port) + "/rpm";

            enc_subs_[i] = create_subscription<std_msgs::msg::Float64>(
                enc_topic, 10,
                [this, i](std_msgs::msg::Float64::SharedPtr msg) {
                    if (!enc_valid_[i]) {
                        enc_prev_[i] = msg->data;
                    }
                    enc_vals_[i]  = msg->data;
                    enc_valid_[i] = true;
                });

            rpm_subs_[i] = create_subscription<std_msgs::msg::Float64>(
                rpm_topic, 10,
                [this, i](std_msgs::msg::Float64::SharedPtr msg) {
                    rpm_vals_[i]  = msg->data;
                    rpm_valid_[i] = true;
                });
        }

        // RPM (rev/min) → m/s: multiply by (2π × r) / 60
        double wheel_radius = declare_parameter("wheel_radius", 0.076);
        double gear_ratio   = declare_parameter("gear_ratio",   1.0);
        double track_width  = declare_parameter("track_width",  0.3);
        double wheel_base   = declare_parameter("wheel_base",   0.3);

        rpm_to_ms_  = (2.0 * M_PI * wheel_radius) / (60.0 * gear_ratio);
        ang_denom_  = 2.0 * (track_width + wheel_base);

        imu_topic_  = declare_parameter<std::string>("imu_topic",   "imu");
        odom_topic_ = declare_parameter<std::string>("odom_topic",  "odom");
        odom_frame_ = declare_parameter<std::string>("odom_frame",  "odom");
        base_frame_ = declare_parameter<std::string>("base_frame",  "base_link");
        publish_tf_ = declare_parameter("publish_tf", false);

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, 10,
            std::bind(&OdometryNode::on_imu, this, std::placeholders::_1));

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

        if (publish_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&OdometryNode::update, this));

        RCLCPP_INFO(get_logger(),
            "odometry ready  wheel_radius=%.4fm  track_width=%.3fm  wheel_base=%.3fm",
            wheel_radius, track_width, wheel_base);
    }

private:
    void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        const auto & q = msg->orientation;
        double siny = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        double yaw  = std::atan2(siny, cosy);

        if (!imu_ready_) {
            imu_offset_ = yaw;
            imu_ready_  = true;
        }

        current_yaw_ = yaw - imu_offset_;
    }

    void update()
    {
        if (!imu_ready_) return;

        for (int i = 0; i < 4; ++i) {
            if (!enc_valid_[i]) return;
        }

        // Position: mecanum inverse kinematics on encoder distance deltas.
        double fl = enc_vals_[0] - enc_prev_[0];
        double fr = enc_vals_[1] - enc_prev_[1];
        double rl = enc_vals_[2] - enc_prev_[2];
        double rr = enc_vals_[3] - enc_prev_[3];

        for (int i = 0; i < 4; ++i) {
            enc_prev_[i] = enc_vals_[i];
        }

        double dx_body = (fl + fr + rl + rr) / 4.0;
        double dy_body = (-fl + fr + rl - rr) / 4.0;

        double cos_y = std::cos(current_yaw_);
        double sin_y = std::sin(current_yaw_);
        x_ += dx_body * cos_y - dy_body * sin_y;
        y_ += dx_body * sin_y + dy_body * cos_y;

        // Velocity: mecanum inverse kinematics on RPM — no dt division.
        double vx = 0.0, vy = 0.0, vz = 0.0;
        if (rpm_valid_[0] && rpm_valid_[1] && rpm_valid_[2] && rpm_valid_[3]) {
            double vfl = rpm_vals_[0] * rpm_to_ms_;
            double vfr = rpm_vals_[1] * rpm_to_ms_;
            double vrl = rpm_vals_[2] * rpm_to_ms_;
            double vrr = rpm_vals_[3] * rpm_to_ms_;
            vx = (vfl + vfr + vrl + vrr) / 4.0;
            vy = (-vfl + vfr + vrl - vrr) / 4.0;
            // Encoder-derived angular velocity — independent of IMU for EKF cross-check.
            vz = (-vfl + vfr - vrl + vrr) / ang_denom_;
        }

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, current_yaw_);

        auto now = get_clock()->now();

        nav_msgs::msg::Odometry odom;
        odom.header.stamp            = now;
        odom.header.frame_id         = odom_frame_;
        odom.child_frame_id          = base_frame_;
        odom.pose.pose.position.x    = x_;
        odom.pose.pose.position.y    = y_;
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        odom.twist.twist.linear.x    = vx;
        odom.twist.twist.linear.y    = vy;
        odom.twist.twist.angular.z   = vz;

        // Pose covariance — diagonal: x, y, z, roll, pitch, yaw
        odom.pose.covariance[0]  = 0.01;   // x  (σ = 10 cm)
        odom.pose.covariance[7]  = 0.01;   // y
        odom.pose.covariance[35] = 0.05;   // yaw (σ ≈ 13°)

        // Twist covariance — diagonal: vx, vy, vz, vroll, vpitch, vyaw
        odom.twist.covariance[0]  = 0.001;  // vx
        odom.twist.covariance[7]  = 0.001;  // vy (lateral — meaningful on mecanum)
        odom.twist.covariance[35] = 0.001;  // vyaw (wheel differential)

        odom_pub_->publish(odom);

        if (publish_tf_ && tf_broadcaster_) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header              = odom.header;
            tf.child_frame_id      = base_frame_;
            tf.transform.translation.x = x_;
            tf.transform.translation.y = y_;
            tf.transform.rotation      = odom.pose.pose.orientation;
            tf_broadcaster_->sendTransform(tf);
        }
    }

    // Encoder distance — index: 0=FL, 1=FR, 2=RL, 3=RR
    std::array<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr, 4> enc_subs_;
    std::array<double, 4> enc_vals_  = {};
    std::array<double, 4> enc_prev_  = {};
    std::array<bool,   4> enc_valid_ = {};

    // RPM — same index order
    std::array<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr, 4> rpm_subs_;
    std::array<double, 4> rpm_vals_  = {};
    std::array<bool,   4> rpm_valid_ = {};

    // IMU state
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    bool   imu_ready_   = false;
    double imu_offset_  = 0.0;
    double current_yaw_ = 0.0;

    // Odometry state
    double x_ = 0.0, y_ = 0.0;

    // Config
    double      rpm_to_ms_;
    double      ang_denom_;
    std::string imu_topic_, odom_topic_, odom_frame_, base_frame_;
    bool        publish_tf_;

    // ROS
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>        tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr                          timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryNode>());
    rclcpp::shutdown();
    return 0;
}
