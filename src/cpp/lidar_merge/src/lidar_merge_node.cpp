#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

class LidarMergeNode : public rclcpp::Node
{
public:
    LidarMergeNode() : Node("lidar_merge")
    {
        front_x_   = declare_parameter("front.x",    0.0);
        front_y_   = declare_parameter("front.y",    0.0);
        front_yaw_ = declare_parameter("front.yaw",  0.0);
        back_x_    = declare_parameter("back.x",     0.0);
        back_y_    = declare_parameter("back.y",     0.0);
        back_yaw_  = declare_parameter("back.yaw",   M_PI);

        double half_deg = declare_parameter("valid_half_angle_deg", 103.0);
        valid_half_ = half_deg * M_PI / 180.0;
        // Scan angle 0° points outward (away from robot body) for both lidars
        // (effect of inverted:true + reversion:true on the YDLidar driver).
        // Valid arc: ±valid_half_ of 0°.
        // DISCARD rays where: norm > valid_half_ && norm < (2π - valid_half_)

        double res_deg = declare_parameter("output_resolution_deg", 0.5);
        res_  = res_deg * M_PI / 180.0;
        bins_ = static_cast<int>(std::round(2.0 * M_PI / res_));

        range_min_    = declare_parameter("range_min",    0.05);
        range_max_    = declare_parameter("range_max",    12.0);
        output_frame_ = declare_parameter("output_frame", std::string("base_scan"));

        // noise filters
        // Intensity threshold: skip rays whose intensity is below this value.
        // 0.0 = disabled.  Start around 100–200 and tune up if noise persists.
        min_intensity_ = declare_parameter("min_intensity", 0.0);

        // Neighbor filter: after merging, discard any bin that has no occupied
        // neighbour within ±neighbor_window_bins at a similar range.
        // min_neighbors:       how many neighbours required (1 is usually enough)
        // neighbor_window_deg: angular search window each side (degrees)
        // neighbor_dist_tol:   max range difference to count as a neighbour (m)
        // Set min_neighbors to 0 to disable.
        min_neighbors_      = declare_parameter("min_neighbors",       1);
        double win_deg      = declare_parameter("neighbor_window_deg", 2.0);
        neighbor_dist_tol_  = declare_parameter("neighbor_dist_tol",   0.3);
        neighbor_window_bins_ = static_cast<int>(std::ceil(win_deg / res_deg));

        auto qos = rclcpp::SensorDataQoS();

        front_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/front/scan", qos,
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                front_scan_ = msg;
                try_merge();
            });

        back_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/back/scan", qos,
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                back_scan_ = msg;
                try_merge();
            });

        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

        RCLCPP_INFO(get_logger(),
                    "Lidar merge ready — frame=%s bins=%d res=%.2f° valid=±%.1f° "
                    "min_intensity=%.0f neighbors≥%d win=±%d bins tol=%.2fm",
                    output_frame_.c_str(), bins_, res_deg, half_deg,
                    min_intensity_, min_neighbors_,
                    neighbor_window_bins_, neighbor_dist_tol_);
    }

private:
    void try_merge()
    {
        if (!front_scan_ || !back_scan_) return;

        std::vector<float> out_ranges(bins_, std::numeric_limits<float>::infinity());
        std::vector<float> out_intensities(bins_, 0.0f);

        project(*front_scan_, front_x_, front_y_, front_yaw_, out_ranges, out_intensities);
        project(*back_scan_,  back_x_,  back_y_,  back_yaw_,  out_ranges, out_intensities);

        if (min_neighbors_ > 0) {
            neighbor_filter(out_ranges, out_intensities);
        }

        sensor_msgs::msg::LaserScan merged;
        merged.header.frame_id = output_frame_;
        merged.header.stamp    =
            rclcpp::Time(front_scan_->header.stamp) > rclcpp::Time(back_scan_->header.stamp)
            ? front_scan_->header.stamp
            : back_scan_->header.stamp;

        merged.angle_min       = -M_PI;
        merged.angle_max       =  M_PI;
        merged.angle_increment = res_;
        merged.time_increment  = 0.0f;
        merged.scan_time       = front_scan_->scan_time;
        merged.range_min       = static_cast<float>(range_min_);
        merged.range_max       = static_cast<float>(range_max_);
        merged.ranges          = std::move(out_ranges);
        merged.intensities     = std::move(out_intensities);

        scan_pub_->publish(merged);

        front_scan_.reset();
        back_scan_.reset();
    }

    void project(
        const sensor_msgs::msg::LaserScan & scan,
        double cx, double cy, double cyaw,
        std::vector<float> & out_ranges,
        std::vector<float> & out_intensities)
    {
        bool has_intensity = (scan.intensities.size() == scan.ranges.size());
        double angle = scan.angle_min;

        for (size_t i = 0; i < scan.ranges.size(); ++i, angle += scan.angle_increment) {
            // Intensity filter — skip unreliable low-intensity returns
            if (has_intensity && min_intensity_ > 0.0 &&
                scan.intensities[i] < static_cast<float>(min_intensity_)) {
                continue;
            }

            // Valid-arc filter: 0° is outward, discard rays pointing back into robot
            double norm = std::fmod(angle, 2.0 * M_PI);
            if (norm < 0.0) norm += 2.0 * M_PI;
            if (norm > valid_half_ && norm < (2.0 * M_PI - valid_half_)) continue;

            float r = scan.ranges[i];
            if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) continue;

            double a = angle + cyaw;
            double x = cx + r * std::cos(a);
            double y = cy + r * std::sin(a);

            double out_r = std::hypot(x, y);
            double out_a = std::atan2(y, x);

            int idx = static_cast<int>((out_a + M_PI) / res_) % bins_;
            if (idx < 0) idx += bins_;

            if (static_cast<float>(out_r) < out_ranges[idx]) {
                out_ranges[idx] = static_cast<float>(out_r);
                if (has_intensity) {
                    out_intensities[idx] = scan.intensities[i];
                }
            }
        }
    }

    // Post-merge neighbor filter: remove any bin whose nearest occupied neighbour
    // within ±neighbor_window_bins_ is more than neighbor_dist_tol_ away in range,
    // or has no occupied neighbour at all.
    void neighbor_filter(std::vector<float> & ranges, std::vector<float> & intensities)
    {
        std::vector<float> filtered(bins_, std::numeric_limits<float>::infinity());
        std::vector<float> filt_int(bins_, 0.0f);

        for (int i = 0; i < bins_; ++i) {
            if (!std::isfinite(ranges[i])) continue;

            int neighbors_found = 0;
            for (int d = 1; d <= neighbor_window_bins_ && neighbors_found < min_neighbors_; ++d) {
                int left  = (i - d + bins_) % bins_;
                int right = (i + d) % bins_;

                if (std::isfinite(ranges[left]) &&
                    std::abs(ranges[left] - ranges[i]) <= neighbor_dist_tol_) {
                    ++neighbors_found;
                }
                if (neighbors_found < min_neighbors_ &&
                    std::isfinite(ranges[right]) &&
                    std::abs(ranges[right] - ranges[i]) <= neighbor_dist_tol_) {
                    ++neighbors_found;
                }
            }

            if (neighbors_found >= min_neighbors_) {
                filtered[i]  = ranges[i];
                filt_int[i]  = intensities[i];
            }
        }

        ranges      = std::move(filtered);
        intensities = std::move(filt_int);
    }

    double front_x_, front_y_, front_yaw_;
    double back_x_,  back_y_,  back_yaw_;
    double valid_half_;
    double res_, range_min_, range_max_;
    int    bins_;
    std::string output_frame_;

    double min_intensity_;
    int    min_neighbors_;
    int    neighbor_window_bins_;
    double neighbor_dist_tol_;

    sensor_msgs::msg::LaserScan::SharedPtr front_scan_;
    sensor_msgs::msg::LaserScan::SharedPtr back_scan_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr front_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr back_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr    scan_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarMergeNode>());
    rclcpp::shutdown();
    return 0;
}
