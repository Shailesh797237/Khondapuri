#ifndef PLC_COMMUNICATION_PLC_BRIDGE_HPP_
#define PLC_COMMUNICATION_PLC_BRIDGE_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "snap7.h"

namespace plc_communication {

struct BitSignal {
    std::string name;
    int byte_offset;
    int bit_offset;
    bool writable{false};
};

struct DBBlock {
    int db_number;
    int start_byte;
    int size_bytes;
    std::vector<BitSignal> signals;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr raw_pub;
};

class PLCBridgeNode : public rclcpp::Node {
public:
    explicit PLCBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~PLCBridgeNode() override;

    bool write_bit(int db_number, int byte_offset, int bit_offset, bool value);

private:
    void init_parameters();
    void setup_db_blocks();
    void timer_callback();

    bool ensure_connected();
    void disconnect_plc();
    static bool extract_bit(const uint8_t * buffer, int byte_index, int bit_index);

    // Node Parameters
    std::string plc_ip_;
    int plc_rack_;
    int plc_slot_;
    int poll_rate_ms_;
    double reconnect_interval_sec_;
    std::string topic_prefix_;
    bool publish_raw_db_;
    int db_safety_no_;
    int db_actuator_no_;
    int db_actuator_size_;

    // Snap7 Client
    std::recursive_mutex client_mutex_;
    S7Object client_{0};
    bool is_connected_{false};
    rclcpp::Time last_reconnect_attempt_{0, 0, RCL_ROS_TIME};

    // DB Blocks, Publishers, and Subscribers
    std::vector<DBBlock> db_blocks_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> signal_publishers_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> subscriptions_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connection_status_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace plc_communication

#endif  // PLC_COMMUNICATION_PLC_BRIDGE_HPP_
