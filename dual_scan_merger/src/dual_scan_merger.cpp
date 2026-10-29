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

using std::placeholders::_1;
using std::placeholders::_2;

class DualScanMerger : public rclcpp::Node
{
public:
  DualScanMerger()
  : Node("dual_scan_merger")
  {
    // Topics
    front_topic_  = this->declare_parameter<std::string>("front_topic", "/front/scan");
    back_topic_   = this->declare_parameter<std::string>("back_topic", "/back/scan");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/scan");
    output_frame_ = this->declare_parameter<std::string>("output_frame", "base_link");

    // Sensor yaw offsets (from URDF joint rpy Z values)
    front_yaw_offset_ = this->declare_parameter<double>("front_yaw_offset", 0.7854);    // +45°
    back_yaw_offset_  = this->declare_parameter<double>("back_yaw_offset", -2.3562);     // -135°

    // Sensor XY positions relative to base_link (from URDF joint xyz)
    front_sensor_x_ = this->declare_parameter<double>("front_sensor_x",  0.690);
    front_sensor_y_ = this->declare_parameter<double>("front_sensor_y",  0.395);
    back_sensor_x_  = this->declare_parameter<double>("back_sensor_x",  -0.690);
    back_sensor_y_  = this->declare_parameter<double>("back_sensor_y",  -0.395);

    // Max scan distance
    max_allowed_range_ = this->declare_parameter<double>("max_allowed_range", 20.0);

    // Minimum range to filter robot body hits (distance from base_link origin)
    robot_body_radius_ = this->declare_parameter<double>("robot_body_radius", 0.55);

    rclcpp::QoS pub_qos(1);
    pub_qos.reliable();
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, pub_qos);

    front_sub_.subscribe(this, front_topic_);
    back_sub_.subscribe(this, back_topic_);

    sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), front_sub_, back_sub_);
    sync_->registerCallback(std::bind(&DualScanMerger::scanCallback, this, _1, _2));

    RCLCPP_INFO(this->get_logger(),
      "Dual Scan Merger Started (front: yaw=%.4f pos=(%.3f,%.3f), back: yaw=%.4f pos=(%.3f,%.3f))",
      front_yaw_offset_, front_sensor_x_, front_sensor_y_,
      back_yaw_offset_, back_sensor_x_, back_sensor_y_);
  }

private:

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::LaserScan,
      sensor_msgs::msg::LaserScan> SyncPolicy;

  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

  void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr front,
                    const sensor_msgs::msg::LaserScan::ConstSharedPtr back)
  {
    if(front->ranges.empty() || back->ranges.empty())
      return;

    sensor_msgs::msg::LaserScan merged;

    merged.header.stamp = this->now();
    merged.header.frame_id = output_frame_;

    merged.angle_min = -M_PI;
    merged.angle_max =  M_PI;

    merged.angle_increment = front->angle_increment;
    merged.time_increment  = front->time_increment;
    merged.scan_time       = front->scan_time;

    merged.range_min = std::min(front->range_min, back->range_min);
    merged.range_max = max_allowed_range_;

    int total_points = static_cast<int>(
        std::ceil((merged.angle_max - merged.angle_min) / merged.angle_increment));

    // Initialize with infinity (no obstacle)
    merged.ranges.assign(total_points, std::numeric_limits<float>::infinity());

    // Transform each scan with BOTH rotation AND translation
    insertScan(front, merged, front_yaw_offset_, front_sensor_x_, front_sensor_y_);
    insertScan(back,  merged, back_yaw_offset_,  back_sensor_x_,  back_sensor_y_);

    scan_pub_->publish(merged);
  }

  /**
   * Transform scan rays from sensor frame to base_link frame.
   *
   * For each ray:
   *   1. Convert polar (angle, range) → Cartesian (lx, ly) in sensor-local frame
   *   2. Rotate by sensor yaw offset
   *   3. Translate by sensor XY position on robot
   *   4. Convert back to polar (new_angle, new_range) relative to base_link
   */
  void insertScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan,
                  sensor_msgs::msg::LaserScan &merged,
                  double yaw_offset,
                  double sensor_x, double sensor_y)
  {
    const double cos_yaw = std::cos(yaw_offset);
    const double sin_yaw = std::sin(yaw_offset);

    for(size_t i = 0; i < scan->ranges.size(); ++i)
    {
      float range = scan->ranges[i];
      if(!std::isfinite(range) || range < scan->range_min)
        continue;

      // Step 1: Ray angle and Cartesian point in sensor-local frame
      double angle_in_sensor = scan->angle_min + i * scan->angle_increment;
      double lx = range * std::cos(angle_in_sensor);
      double ly = range * std::sin(angle_in_sensor);

      // Step 2: Rotate by sensor yaw offset
      double rx = cos_yaw * lx - sin_yaw * ly;
      double ry = sin_yaw * lx + cos_yaw * ly;

      // Step 3: Translate by sensor position on robot
      double bx = rx + sensor_x;
      double by = ry + sensor_y;

      // Step 4: Convert back to polar relative to base_link
      double new_range = std::sqrt(bx * bx + by * by);
      double new_angle = std::atan2(by, bx);

      // Filter: skip points inside the robot body
      if(new_range < robot_body_radius_)
        continue;

      // Clamp to max range
      if(new_range > max_allowed_range_)
        new_range = max_allowed_range_;

      // Map to merged scan index
      int index = static_cast<int>(
          std::round((new_angle - merged.angle_min) / merged.angle_increment));

      if(index >= 0 && index < static_cast<int>(merged.ranges.size()))
      {
        merged.ranges[index] =
            std::min(merged.ranges[index], static_cast<float>(new_range));
      }
    }
  }

  std::string front_topic_;
  std::string back_topic_;
  std::string output_topic_;
  std::string output_frame_;

  double front_yaw_offset_;
  double back_yaw_offset_;

  double front_sensor_x_;
  double front_sensor_y_;
  double back_sensor_x_;
  double back_sensor_y_;

  double max_allowed_range_;
  double robot_body_radius_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

  message_filters::Subscriber<sensor_msgs::msg::LaserScan> front_sub_;
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> back_sub_;
  std::shared_ptr<Synchronizer> sync_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DualScanMerger>());
  rclcpp::shutdown();
  return 0;
}


// #include <memory>
// #include <vector>
// #include <algorithm>
// #include <cmath>
// #include <limits>

// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/laser_scan.hpp"
// #include "message_filters/subscriber.h"
// #include "message_filters/synchronizer.h"
// #include "message_filters/sync_policies/approximate_time.h"

// using std::placeholders::_1;
// using std::placeholders::_2;

// class DualScanMerger : public rclcpp::Node
// {
// public:
//   DualScanMerger()
//   : Node("dual_scan_merger")
//   {
//     front_topic_  = this->declare_parameter<std::string>("front_topic", "/front/scan");
//     back_topic_   = this->declare_parameter<std::string>("back_topic", "/back/scan");
//     output_topic_ = this->declare_parameter<std::string>("output_topic", "/scan");
//     output_frame_ = this->declare_parameter<std::string>("output_frame", "base_link");

//     front_yaw_offset_ = 0.7854;   // +45°
//     back_yaw_offset_  = -2.3562;  // -135°

//     max_allowed_range_ = this->declare_parameter<double>("max_allowed_range", 5.0);

//     // 🔥 Robot body ignore radius (meters)
//     robot_ignore_radius_ = this->declare_parameter<double>("robot_ignore_radius", 0.60);

//     scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, 10);

//     front_sub_.subscribe(this, front_topic_);
//     back_sub_.subscribe(this, back_topic_);

//     sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), front_sub_, back_sub_);
//     sync_->registerCallback(std::bind(&DualScanMerger::scanCallback, this, _1, _2));

//     RCLCPP_INFO(this->get_logger(), "Dual Scan Merger Started with Robot Body Filtering");
//   }

// private:

//   typedef message_filters::sync_policies::ApproximateTime<
//       sensor_msgs::msg::LaserScan,
//       sensor_msgs::msg::LaserScan> SyncPolicy;

//   typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

//   void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr front,
//                     const sensor_msgs::msg::LaserScan::ConstSharedPtr back)
//   {
//     if(front->ranges.empty() || back->ranges.empty())
//       return;

//     sensor_msgs::msg::LaserScan merged;

//     merged.header.stamp = this->now();
//     merged.header.frame_id = output_frame_;

//     merged.angle_min = -M_PI;
//     merged.angle_max =  M_PI;

//     merged.angle_increment = front->angle_increment;
//     merged.time_increment  = front->time_increment;
//     merged.scan_time       = front->scan_time;

//     merged.range_min = std::min(front->range_min, back->range_min);
//     merged.range_max = max_allowed_range_;

//     int total_points = static_cast<int>(
//         std::ceil((merged.angle_max - merged.angle_min) / merged.angle_increment));

//     // Initialize as infinity (no obstacle)
//     merged.ranges.assign(total_points, std::numeric_limits<float>::infinity());

//     insertScan(front, merged, front_yaw_offset_);
//     insertScan(back,  merged, back_yaw_offset_);

//     scan_pub_->publish(merged);
//   }

//   void insertScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan,
//                   sensor_msgs::msg::LaserScan &merged,
//                   double yaw_offset)
//   {
//     for(size_t i = 0; i < scan->ranges.size(); ++i)
//     {
//       double angle = scan->angle_min + i * scan->angle_increment;
//       angle += yaw_offset;

//       while(angle > M_PI)  angle -= 2.0 * M_PI;
//       while(angle < -M_PI) angle += 2.0 * M_PI;

//       int index = static_cast<int>(
//           std::round((angle - merged.angle_min) / merged.angle_increment));

//       if(index >= 0 && index < static_cast<int>(merged.ranges.size()))
//       {
//         float range = scan->ranges[i];

//         if(std::isfinite(range) && range >= scan->range_min)
//         {
//           // 🔥 IGNORE ROBOT BODY AREA
//           if(range < robot_ignore_radius_)
//             continue;

//           if(range > max_allowed_range_)
//             range = max_allowed_range_;

//           merged.ranges[index] =
//               std::min(merged.ranges[index], range);
//         }
//       }
//     }
//   }

//   std::string front_topic_;
//   std::string back_topic_;
//   std::string output_topic_;
//   std::string output_frame_;

//   double front_yaw_offset_;
//   double back_yaw_offset_;
//   double max_allowed_range_;
//   double robot_ignore_radius_;

//   rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
//   message_filters::Subscriber<sensor_msgs::msg::LaserScan> front_sub_;
//   message_filters::Subscriber<sensor_msgs::msg::LaserScan> back_sub_;
//   std::shared_ptr<Synchronizer> sync_;
// };

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<DualScanMerger>());
//   rclcpp::shutdown();
//   return 0;
// }













// #include <memory>
// #include <vector>
// #include <algorithm>
// #include <cmath>

// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/laser_scan.hpp"
// #include "message_filters/subscriber.h"
// #include "message_filters/synchronizer.h"
// #include "message_filters/sync_policies/approximate_time.h"

// using std::placeholders::_1;
// using std::placeholders::_2;

// class DualScanMerger : public rclcpp::Node
// {
// public:
//   DualScanMerger()
//   : Node("dual_scan_merger")
//   {
//     // Topics
//     front_topic_  = this->declare_parameter<std::string>("front_topic", "/front/scan");
//     back_topic_   = this->declare_parameter<std::string>("back_topic", "/back/scan");
//     output_topic_ = this->declare_parameter<std::string>("output_topic", "/scan");
//     output_frame_ = this->declare_parameter<std::string>("output_frame", "base_link");

//     // 🔥 URDF yaw offsets (from your robot)
//     front_yaw_offset_ = 0.7854;   // +45°
//     back_yaw_offset_  = -2.3562;  // -135°

//     // 🔥 Max allowed range (default 30m)
//     max_allowed_range_ = this->declare_parameter<double>("max_allowed_range", 30.0);

//     scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, 10);

//     front_sub_.subscribe(this, front_topic_);
//     back_sub_.subscribe(this, back_topic_);

//     sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), front_sub_, back_sub_);
//     sync_->registerCallback(std::bind(&DualScanMerger::scanCallback, this, _1, _2));

//     RCLCPP_INFO(this->get_logger(), "Dual Scan Merger Started");
//   }

// private:

//   typedef message_filters::sync_policies::ApproximateTime<
//       sensor_msgs::msg::LaserScan,
//       sensor_msgs::msg::LaserScan> SyncPolicy;

//   typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

//   void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr front,
//                     const sensor_msgs::msg::LaserScan::ConstSharedPtr back)
//   {
//     if(front->ranges.empty() || back->ranges.empty())
//       return;

//     sensor_msgs::msg::LaserScan merged;

//     merged.header.stamp = this->now();
//     merged.header.frame_id = output_frame_;

//     // Full 360°
//     merged.angle_min = -M_PI;
//     merged.angle_max =  M_PI;

//     merged.angle_increment = front->angle_increment;
//     merged.time_increment  = front->time_increment;
//     merged.scan_time       = front->scan_time;

//     merged.range_min = std::min(front->range_min, back->range_min);

//     // 🔥 Force range max to 30m (or parameter)
//     merged.range_max = max_allowed_range_;

//     int total_points = static_cast<int>(
//         std::ceil((merged.angle_max - merged.angle_min) / merged.angle_increment));

//     merged.ranges.assign(total_points, merged.range_max);

//     insertScan(front, merged, front_yaw_offset_);
//     insertScan(back,  merged, back_yaw_offset_);

//     scan_pub_->publish(merged);
//   }

//   void insertScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan,
//                   sensor_msgs::msg::LaserScan &merged,
//                   double yaw_offset)
//   {
//     for(size_t i = 0; i < scan->ranges.size(); ++i)
//     {
//       double angle = scan->angle_min + i * scan->angle_increment;

//       // Apply URDF yaw correction
//       angle += yaw_offset;

//       // Normalize angle to [-pi, pi]
//       while(angle > M_PI)  angle -= 2.0 * M_PI;
//       while(angle < -M_PI) angle += 2.0 * M_PI;

//       int index = static_cast<int>(
//           std::round((angle - merged.angle_min) / merged.angle_increment));

//       if(index >= 0 && index < static_cast<int>(merged.ranges.size()))
//       {
//         float range = scan->ranges[i];

//         if(std::isfinite(range) && range >= scan->range_min)
//         {
//           // 🔥 Clamp to max allowed range (30m)
//           if(range > max_allowed_range_)
//             range = max_allowed_range_;

//           merged.ranges[index] =
//               std::min(merged.ranges[index], range);
//         }
//       }
//     }
//   }

//   std::string front_topic_;
//   std::string back_topic_;
//   std::string output_topic_;
//   std::string output_frame_;

//   double front_yaw_offset_;
//   double back_yaw_offset_;
//   double max_allowed_range_;

//   rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

//   message_filters::Subscriber<sensor_msgs::msg::LaserScan> front_sub_;
//   message_filters::Subscriber<sensor_msgs::msg::LaserScan> back_sub_;
//   std::shared_ptr<Synchronizer> sync_;
// };

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<DualScanMerger>());
//   rclcpp::shutdown();
//   return 0;
// }

