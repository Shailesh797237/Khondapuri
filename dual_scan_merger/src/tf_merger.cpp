#include <memory>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

// TF2 Headers
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class DualScanMerger : public rclcpp::Node
{
public:
    DualScanMerger() : Node("dual_scan_merger")
    {
        // Parameters fetching
        front_topic_ = this->declare_parameter<std::string>("front_topic", "/front/scan");
        back_topic_  = this->declare_parameter<std::string>("back_topic", "/back/scan");
        output_topic_ = this->declare_parameter<std::string>("output_topic", "/scan");
        output_frame_ = this->declare_parameter<std::string>("output_frame", "base_link");
        max_allowed_range_ = this->declare_parameter<double>("max_allowed_range", 5.0);

        // NEW PARAMETER (detect only after this distance)
        detection_start_range_ = this->declare_parameter<double>("detection_start_range", 2.0);

        // Directional ignore distances (robot body filtering)
        front_ignore_ = this->declare_parameter<double>("front_ignore", 1.0);
        back_ignore_  = this->declare_parameter<double>("back_ignore", 1.0);
        left_ignore_  = this->declare_parameter<double>("left_ignore", 1.0);
        right_ignore_ = this->declare_parameter<double>("right_ignore", 1.0);

        // TF2 Setup
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publisher
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, 10);

        // Message Filters Setup
        front_sub_.subscribe(this, front_topic_);
        back_sub_.subscribe(this, back_topic_);

        sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), front_sub_, back_sub_);
        sync_->registerCallback(std::bind(&DualScanMerger::scanCallback, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "C++ TF2 Scan Merger Started (270° Support)");
    }

private:
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

    void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr front,
                      const sensor_msgs::msg::LaserScan::ConstSharedPtr back)
    {
        if (front->ranges.empty() || back->ranges.empty()) return;

        auto merged = std::make_unique<sensor_msgs::msg::LaserScan>();
        merged->header.stamp = this->now();
        merged->header.frame_id = output_frame_;

        merged->angle_min = -M_PI;
        merged->angle_max = M_PI;
        merged->angle_increment = front->angle_increment;
        merged->time_increment = front->time_increment;
        merged->scan_time = front->scan_time;
        merged->range_min = std::min(front->range_min, back->range_min);
        merged->range_max = static_cast<float>(max_allowed_range_);

        int total_points = static_cast<int>(std::ceil((merged->angle_max - merged->angle_min) / merged->angle_increment));
        merged->ranges.assign(total_points, std::numeric_limits<float>::infinity());

        // Process both sensors
        processAndInsert(front, *merged);
        processAndInsert(back, *merged);

        scan_pub_->publish(std::move(merged));
    }

    void processAndInsert(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan,
                          sensor_msgs::msg::LaserScan &merged)
    {
        geometry_msgs::msg::TransformStamped tf_stamped;
        try {
            tf_stamped = tf_buffer_->lookupTransform(output_frame_, scan->header.frame_id, tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s: %s", scan->header.frame_id.c_str(), ex.what());
            return;
        }

        double tx = tf_stamped.transform.translation.x;
        double ty = tf_stamped.transform.translation.y;
        
        auto q = tf_stamped.transform.rotation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        for (size_t i = 0; i < scan->ranges.size(); ++i) {

            float r = scan->ranges[i];
            if (!std::isfinite(r) || r < scan->range_min) continue;

            // NEW FILTER (ignore objects closer than detection_start_range)
            if (r < detection_start_range_) continue;

            double angle_lidar = scan->angle_min + (i * scan->angle_increment);
            double lx = r * std::cos(angle_lidar);
            double ly = r * std::sin(angle_lidar);

            double bx = lx * std::cos(yaw) - ly * std::sin(yaw) + tx;
            double by = lx * std::sin(yaw) + ly * std::cos(yaw) + ty;

            double new_angle = std::atan2(by, bx);
            float new_range = static_cast<float>(std::sqrt(bx * bx + by * by));

            // NEW FILTER after transform
            if (new_range < detection_start_range_) continue;

            double ignore_dist = 0.0;
            double abs_a = std::abs(new_angle);

            if (abs_a < M_PI / 4.0) ignore_dist = front_ignore_;
            else if (abs_a > 3.0 * M_PI / 4.0) ignore_dist = back_ignore_;
            else if (new_angle > 0) ignore_dist = left_ignore_;
            else ignore_dist = right_ignore_;

            if (new_range < ignore_dist) continue;
            if (new_range > max_allowed_range_) new_range = static_cast<float>(max_allowed_range_);

            int index = static_cast<int>(std::round((new_angle - merged.angle_min) / merged.angle_increment));

            if (index >= 0 && index < static_cast<int>(merged.ranges.size())) {
                merged.ranges[index] = std::min(merged.ranges[index], new_range);
            }
        }
    }

    std::string front_topic_, back_topic_, output_topic_, output_frame_;
    double max_allowed_range_;
    double detection_start_range_;  // NEW VARIABLE
    double front_ignore_, back_ignore_, left_ignore_, right_ignore_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    message_filters::Subscriber<sensor_msgs::msg::LaserScan> front_sub_, back_sub_;
    std::shared_ptr<Synchronizer> sync_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DualScanMerger>());
    rclcpp::shutdown();
    return 0;
}
