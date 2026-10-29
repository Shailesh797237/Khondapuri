#include <memory>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

// TF2 Headers
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class TripleScanMerger : public rclcpp::Node
{
public:
  TripleScanMerger() : Node("triple_scan_merger")
  {
    // Parameters fetching
    topic_livox_  = this->declare_parameter<std::string>("topic_livox", "/livox/scan");
    topic_front_  = this->declare_parameter<std::string>("topic_front", "/front/scan");
    topic_back_   = this->declare_parameter<std::string>("topic_back", "/back/scan");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/scan");
    output_frame_ = this->declare_parameter<std::string>("output_frame", "base_link");

    max_range_         = this->declare_parameter<double>("max_range", 30.0);
    min_range_         = this->declare_parameter<double>("min_range", 0.15);
    robot_body_radius_ = this->declare_parameter<double>("robot_body_radius", 0.45);
    angle_increment_   = this->declare_parameter<double>("angle_increment", 0.00436332); // ~0.25 deg

    // TF Setup
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Publisher - RELIABLE depth=1 to satisfy both RELIABLE and BEST_EFFORT subscribers
    // depth=1 prevents storing stale scan messages for late subscribers
    rclcpp::QoS pub_qos(1);
    pub_qos.reliable();
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, pub_qos);

    // Subscribers with BestEffort / SensorData QoS to match sensor drivers
    rclcpp::QoS sub_qos(10);
    sub_qos.best_effort();

    sub_livox_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      topic_livox_, sub_qos,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        onScanReceived(0, msg);
      });

    sub_front_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      topic_front_, sub_qos,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        onScanReceived(1, msg);
      });

    sub_back_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      topic_back_, sub_qos,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        onScanReceived(2, msg);
      });

    // Publish merged scan at 20 Hz (every 50ms)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&TripleScanMerger::publishMergedScan, this));

    RCLCPP_INFO(this->get_logger(),
      "Triple Scan Merger Node Started!\n"
      "  Input Livox Scan: %s\n"
      "  Input Front SICK Scan: %s\n"
      "  Input Back SICK Scan: %s\n"
      "  Output Topic: %s (Frame: %s)",
      topic_livox_.c_str(), topic_front_.c_str(), topic_back_.c_str(),
      output_topic_.c_str(), output_frame_.c_str());
  }

private:
  void onScanReceived(size_t sensor_idx, const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_scans_[sensor_idx] = msg;
  }

  void publishMergedScan()
  {
    std::vector<sensor_msgs::msg::LaserScan::SharedPtr> scans_to_process;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      scans_to_process = latest_scans_;
    }

    bool has_any_scan = false;
    for (const auto & s : scans_to_process) {
      if (s) {
        has_any_scan = true;
        break;
      }
    }
    if (!has_any_scan) return;

    auto merged = std::make_unique<sensor_msgs::msg::LaserScan>();
    merged->header.stamp = this->now();
    merged->header.frame_id = output_frame_;

    merged->angle_min = -M_PI;
    merged->angle_max =  M_PI;
    merged->angle_increment = static_cast<float>(angle_increment_);
    merged->time_increment  = 0.0f;
    merged->scan_time       = 0.05f; // 20 Hz
    merged->range_min       = static_cast<float>(min_range_);
    merged->range_max       = static_cast<float>(max_range_);

    int total_points = static_cast<int>(
      std::ceil((merged->angle_max - merged->angle_min) / merged->angle_increment));
    merged->ranges.assign(total_points, std::numeric_limits<float>::infinity());

    for (const auto & scan : scans_to_process) {
      if (!scan) continue;
      processScan(scan, *merged);
    }

    scan_pub_->publish(std::move(merged));
  }

  void processScan(const sensor_msgs::msg::LaserScan::SharedPtr & scan,
                   sensor_msgs::msg::LaserScan & merged)
  {
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buffer_->lookupTransform(
        output_frame_, scan->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Could not transform %s to %s: %s",
        scan->header.frame_id.c_str(), output_frame_.c_str(), ex.what());
      return;
    }

    double tx = tf_stamped.transform.translation.x;
    double ty = tf_stamped.transform.translation.y;

    auto q = tf_stamped.transform.rotation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
      float r = scan->ranges[i];
      if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max) continue;

      double angle_sensor = scan->angle_min + (i * scan->angle_increment);
      double lx = r * std::cos(angle_sensor);
      double ly = r * std::sin(angle_sensor);

      // Transform to output_frame (base_link)
      double bx = lx * cos_yaw - ly * sin_yaw + tx;
      double by = lx * sin_yaw + ly * cos_yaw + ty;

      double new_range = std::sqrt(bx * bx + by * by);
      if (new_range < robot_body_radius_) continue;
      if (new_range < min_range_) continue;
      if (new_range > max_range_) new_range = max_range_;

      double new_angle = std::atan2(by, bx);

      int index = static_cast<int>(
        std::round((new_angle - merged.angle_min) / merged.angle_increment));

      if (index >= 0 && index < static_cast<int>(merged.ranges.size())) {
        merged.ranges[index] = std::min(merged.ranges[index], static_cast<float>(new_range));
      }
    }
  }

  std::string topic_livox_, topic_front_, topic_back_;
  std::string output_topic_, output_frame_;
  double max_range_, min_range_, robot_body_radius_, angle_increment_;

  std::mutex mutex_;
  std::vector<sensor_msgs::msg::LaserScan::SharedPtr> latest_scans_{3, nullptr};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_livox_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_front_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_back_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TripleScanMerger>());
  rclcpp::shutdown();
  return 0;
}
