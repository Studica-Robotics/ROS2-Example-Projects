#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

class ConfidenceScanGate : public rclcpp::Node
{
public:
    ConfidenceScanGate() : Node("confidence_scan_gate_node")
    {
        in_topic_   = declare_parameter<std::string>("input_scan_topic",  "/scan");
        out_topic_  = declare_parameter<std::string>("output_scan_topic", "/scan_filtered");
        map_topic_  = declare_parameter<std::string>("map_topic",         "/map");
        odom_topic_ = declare_parameter<std::string>("odom_topic",        "/odometry/filtered");

        bootstrap_scans_  = declare_parameter("bootstrap_scan_count",             40);
        min_map_cells_    = declare_parameter("min_occupied_cells_before_gating", 200);

        motion_gate_    = declare_parameter("enable_motion_gate",   true);
        max_lin_speed_  = declare_parameter("max_linear_speed",     0.35);
        max_ang_speed_  = declare_parameter("max_angular_speed",    0.75);
        max_lin_accel_  = declare_parameter("max_linear_accel",     0.75);
        max_ang_accel_  = declare_parameter("max_angular_accel",    1.50);

        overlap_gate_   = declare_parameter("enable_overlap_gate",    true);
        stride_         = std::max(1, static_cast<int>(declare_parameter("sample_stride", 3)));
        min_validation_ = declare_parameter("min_validation_points",  30);
        min_overlap_    = declare_parameter("min_overlap_ratio",       0.58);
        occ_radius_     = declare_parameter("occupied_near_radius",    0.12);

        ghost_gate_     = declare_parameter("enable_ghost_wall_gate", true);
        ghost_min_      = declare_parameter("ghost_min_offset",       0.08);
        ghost_max_      = declare_parameter("ghost_max_offset",       0.35);
        ghost_gap_      = declare_parameter("ghost_max_point_gap",    0.18);
        ghost_run_len_  = declare_parameter("ghost_min_run_length",   0.75);

        starv_guard_    = declare_parameter("enable_starvation_guard", true);
        starv_window_   = declare_parameter("starvation_window_size",  50);
        starv_thresh_   = declare_parameter("starvation_threshold",    0.85);

        pass_on_tf_fail_ = declare_parameter("pass_on_tf_failure",     true);
        log_interval_    = declare_parameter("debug_log_every_n_scans", 20);

        tf_buf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_lis_ = std::make_shared<tf2_ros::TransformListener>(*tf_buf_, this);

        // /map is published TRANSIENT_LOCAL by slam_toolbox
        rclcpp::QoS map_qos(1);
        map_qos.transient_local().reliable();

        map_sub_  = create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_, map_qos,
            std::bind(&ConfidenceScanGate::on_map, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 20,
            std::bind(&ConfidenceScanGate::on_odom, this, std::placeholders::_1));

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            in_topic_, rclcpp::SensorDataQoS(),
            std::bind(&ConfidenceScanGate::on_scan, this, std::placeholders::_1));

        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(out_topic_, 10);

        RCLCPP_INFO(get_logger(), "Confidence scan gate: %s -> %s",
            in_topic_.c_str(), out_topic_.c_str());
    }

private:

    void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_ = msg;
        occupied_count_ = 0;
        for (auto v : msg->data) if (v >= 50) ++occupied_count_;
    }

    void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double t   = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        double lin = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
        double ang = std::abs(msg->twist.twist.angular.z);

        if (last_odom_t_ > 0.0) {
            double dt   = std::max(1e-3, t - last_odom_t_);
            cur_lin_acc_ = std::abs(lin - last_lin_spd_) / dt;
            cur_ang_acc_ = std::abs(ang - last_ang_spd_) / dt;
        }

        cur_lin_spd_  = lin;
        cur_ang_spd_  = ang;
        last_lin_spd_ = lin;
        last_ang_spd_ = ang;
        last_odom_t_  = t;
    }

    void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        ++total_;

        if (bootstrapping())             { pass(*scan, "bootstrap");             return; }
        if (starv_guard_ && starving())  { pass(*scan, "starvation_guard", false); return; }
        if (motion_gate_ && !motion_ok()){ reject("motion");                    return; }

        auto pose = lookup_pose(*scan);
        if (!pose) {
            pass_on_tf_fail_ ? pass(*scan, "tf_unavailable", false) : reject("tf");
            return;
        }

        auto points = scan_to_map(*scan, *pose);
        if (points.empty()) { reject("no_points"); return; }

        if (overlap_gate_) {
            auto [ok, ratio, n] = overlap_ok(points);
            if (!ok) {
                reject("overlap ratio=" + to_str(ratio, 2) + " n=" + std::to_string(n));
                return;
            }
        }

        if (ghost_gate_) {
            auto [triggered, run_m] = ghost_wall_run(points);
            if (triggered) { reject("ghost_wall run=" + to_str(run_m, 2) + "m"); return; }
        }

        pass(*scan, "accepted");
    }

    bool bootstrapping() const
    {
        return total_ <= bootstrap_scans_ || !map_ || occupied_count_ < min_map_cells_;
    }

    bool motion_ok() const
    {
        return cur_lin_spd_ <= max_lin_speed_
            && cur_ang_spd_ <= max_ang_speed_
            && cur_lin_acc_ <= max_lin_accel_
            && cur_ang_acc_ <= max_ang_accel_;
    }

    bool starving()
    {
        if (static_cast<int>(decisions_.size()) < starv_window_ / 2) return false;
        int rejects = static_cast<int>(std::count(decisions_.begin(), decisions_.end(), false));
        double rate = static_cast<double>(rejects) / static_cast<double>(decisions_.size());
        if (rate > starv_thresh_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                "Starvation guard: rejection_rate=%.0f%% — forcing pass", rate * 100.0);
            return true;
        }
        return false;
    }

    struct OverlapResult { bool ok; double ratio; int n; };

    OverlapResult overlap_ok(const std::vector<std::pair<double, double>> & pts)
    {
        int validation = 0, matched = 0;
        for (auto [x, y] : pts) {
            if (!cell_known(x, y)) continue;
            ++validation;
            if (near_occupied(x, y, occ_radius_)) ++matched;
        }
        // Too few known cells — robot is in unexplored area, pass through.
        if (validation < min_validation_) return {true, 0.0, validation};
        double ratio = static_cast<double>(matched) / validation;
        return {ratio >= min_overlap_, ratio, validation};
    }

    struct GhostResult { bool triggered; double run_m; };

    GhostResult ghost_wall_run(const std::vector<std::pair<double, double>> & pts)
    {
        double current_run = 0.0, max_run = 0.0;
        std::optional<std::pair<double, double>> last_suspicious;

        for (auto [x, y] : pts) {
            bool on_wall   = near_occupied(x, y, ghost_min_);
            bool near_wall = near_occupied(x, y, ghost_max_);
            bool suspicious = near_wall && !on_wall;

            if (suspicious) {
                if (last_suspicious) {
                    double gap = std::hypot(x - last_suspicious->first,
                                           y - last_suspicious->second);
                    current_run = (gap <= ghost_gap_) ? current_run + gap : 0.0;
                }
                max_run        = std::max(max_run, current_run);
                last_suspicious = {x, y};
            }
            // Non-suspicious: do not reset last_suspicious — run expires via gap threshold.
        }
        return {max_run >= ghost_run_len_, max_run};
    }

    using Pose2D = std::tuple<double, double, double>;

    std::optional<Pose2D> lookup_pose(const sensor_msgs::msg::LaserScan & scan)
    {
        try {
            auto tf = tf_buf_->lookupTransform(
                "map", scan.header.frame_id, scan.header.stamp,
                rclcpp::Duration::from_seconds(0.1));

            const auto & r = tf.transform.rotation;
            double siny = 2.0 * (r.w * r.z + r.x * r.y);
            double cosy = 1.0 - 2.0 * (r.y * r.y + r.z * r.z);
            return Pose2D{tf.transform.translation.x,
                          tf.transform.translation.y,
                          std::atan2(siny, cosy)};
        } catch (const tf2::TransformException & ex) {
            if (total_ % std::max(1, log_interval_) == 0)
                RCLCPP_WARN(get_logger(), "TF lookup failed: %s", ex.what());
            return std::nullopt;
        }
    }

    std::vector<std::pair<double, double>> scan_to_map(
        const sensor_msgs::msg::LaserScan & scan, const Pose2D & pose)
    {
        auto [tx, ty, yaw] = pose;
        double c = std::cos(yaw), s = std::sin(yaw);
        std::vector<std::pair<double, double>> pts;

        double angle = scan.angle_min;
        for (int i = 0; i < static_cast<int>(scan.ranges.size()); ++i, angle += scan.angle_increment) {
            if (i % stride_ != 0) continue;
            double r = scan.ranges[i];
            if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) continue;
            double lx = r * std::cos(angle), ly = r * std::sin(angle);
            pts.emplace_back(tx + c * lx - s * ly, ty + s * lx + c * ly);
        }
        return pts;
    }

    std::optional<std::pair<int, int>> world_to_cell(double x, double y) const
    {
        if (!map_) return std::nullopt;
        const auto & info = map_->info;
        int cx = static_cast<int>((x - info.origin.position.x) / info.resolution);
        int cy = static_cast<int>((y - info.origin.position.y) / info.resolution);
        if (cx < 0 || cy < 0 ||
            cx >= static_cast<int>(info.width) || cy >= static_cast<int>(info.height))
            return std::nullopt;
        return std::pair{cx, cy};
    }

    int8_t cell_value(int cx, int cy) const
    {
        return map_->data[static_cast<size_t>(cy) * map_->info.width + cx];
    }

    bool cell_known(double x, double y) const
    {
        auto c = world_to_cell(x, y);
        return c && cell_value(c->first, c->second) >= 0;
    }

    bool near_occupied(double x, double y, double radius_m) const
    {
        auto c = world_to_cell(x, y);
        if (!c || !map_) return false;

        int r = std::max(1, static_cast<int>(std::ceil(radius_m / map_->info.resolution)));
        int w = static_cast<int>(map_->info.width);
        int h = static_cast<int>(map_->info.height);

        for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
                if (dx * dx + dy * dy > r * r) continue;
                int nx = c->first + dx, ny = c->second + dy;
                if (nx >= 0 && ny >= 0 && nx < w && ny < h && cell_value(nx, ny) >= 50)
                    return true;
            }
        }
        return false;
    }

    void push_decision(bool passed)
    {
        decisions_.push_back(passed);
        if (static_cast<int>(decisions_.size()) > starv_window_)
            decisions_.pop_front();
    }

    void pass(const sensor_msgs::msg::LaserScan & scan,
              const std::string & reason, bool record = true)
    {
        ++passed_;
        if (record) push_decision(true);
        scan_pub_->publish(scan);

        if (total_ % std::max(1, log_interval_) == 0)
            RCLCPP_INFO(get_logger(),
                "PASS  [%s] total=%d pass=%d reject=%d cells=%d",
                reason.c_str(), total_, passed_, rejected_, occupied_count_);
    }

    void reject(const std::string & reason)
    {
        ++rejected_;
        push_decision(false);

        if (total_ % std::max(1, log_interval_) == 0)
            RCLCPP_WARN(get_logger(),
                "REJECT [%s] total=%d pass=%d reject=%d",
                reason.c_str(), total_, passed_, rejected_);
    }

    static std::string to_str(double v, int decimals)
    {
        std::string s = std::to_string(v);
        auto dot = s.find('.');
        if (dot != std::string::npos && static_cast<int>(s.size()) > static_cast<int>(dot) + decimals + 1)
            s.resize(dot + decimals + 1);
        return s;
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    int occupied_count_ = 0;

    double last_odom_t_  = -1.0;
    double last_lin_spd_ = 0.0, last_ang_spd_ = 0.0;
    double cur_lin_spd_  = 0.0, cur_ang_spd_  = 0.0;
    double cur_lin_acc_  = 0.0, cur_ang_acc_  = 0.0;

    int total_ = 0, passed_ = 0, rejected_ = 0;
    std::deque<bool> decisions_;

    std::shared_ptr<tf2_ros::Buffer>           tf_buf_;
    std::shared_ptr<tf2_ros::TransformListener> tf_lis_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr   scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr      scan_pub_;

    std::string in_topic_, out_topic_, map_topic_, odom_topic_;
    int    bootstrap_scans_, min_map_cells_;
    bool   motion_gate_;
    double max_lin_speed_, max_ang_speed_, max_lin_accel_, max_ang_accel_;
    bool   overlap_gate_;
    int    stride_, min_validation_;
    double min_overlap_, occ_radius_;
    bool   ghost_gate_;
    double ghost_min_, ghost_max_, ghost_gap_, ghost_run_len_;
    bool   starv_guard_;
    int    starv_window_;
    double starv_thresh_;
    bool   pass_on_tf_fail_;
    int    log_interval_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConfidenceScanGate>());
    rclcpp::shutdown();
    return 0;
}
