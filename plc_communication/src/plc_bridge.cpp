#include "plc_communication/plc_bridge.hpp"

using namespace std::chrono_literals;

namespace plc_communication {

PLCBridgeNode::PLCBridgeNode(const rclcpp::NodeOptions & options)
: Node("plc_communication_node", options),
  client_(0),
  is_connected_(false)
{
    RCLCPP_INFO(get_logger(), "Starting PLC Communication Bridge Node...");

    init_parameters();
    setup_db_blocks();

    // Create Snap7 client object
    client_ = Cli_Create();
    if (!client_) {
        RCLCPP_FATAL(get_logger(), "Failed to create Snap7 client instance!");
        return;
    }

    // Timer for polling PLC
    std::chrono::milliseconds interval(poll_rate_ms_);
    timer_ = this->create_wall_timer(
        interval,
        std::bind(&PLCBridgeNode::timer_callback, this)
    );

    RCLCPP_INFO(get_logger(), "PLC Communication Node initialized. Target IP: %s (Rack: %d, Slot: %d), Polling: %d ms",
                plc_ip_.c_str(), plc_rack_, plc_slot_, poll_rate_ms_);
}

PLCBridgeNode::~PLCBridgeNode()
{
    RCLCPP_INFO(get_logger(), "Shutting down PLC Communication Bridge Node...");
    disconnect_plc();
    std::lock_guard<std::recursive_mutex> lock(client_mutex_);
    if (client_) {
        Cli_Destroy(&client_);
        client_ = 0;
    }
}

void PLCBridgeNode::init_parameters()
{
    plc_ip_ = this->declare_parameter<std::string>("plc_ip", "192.168.1.21");
    plc_rack_ = this->declare_parameter<int>("plc_rack", 0);
    plc_slot_ = this->declare_parameter<int>("plc_slot", 1);
    poll_rate_ms_ = this->declare_parameter<int>("poll_rate_ms", 100);
    reconnect_interval_sec_ = this->declare_parameter<double>("reconnect_interval_sec", 2.0);
    topic_prefix_ = this->declare_parameter<std::string>("topic_prefix", "/plc");
    publish_raw_db_ = this->declare_parameter<bool>("publish_raw_db", false);
    db_safety_no_ = this->declare_parameter<int>("db_safety_no", 1);
    db_actuator_no_ = this->declare_parameter<int>("db_actuator_no", 3);
    db_actuator_size_ = this->declare_parameter<int>("db_actuator_size", 3);

    // Normalize topic prefix
    if (!topic_prefix_.empty() && topic_prefix_.back() == '/') {
        topic_prefix_.pop_back();
    }
}

void PLCBridgeNode::setup_db_blocks()
{
    // Publisher for PLC connection health status
    std::string conn_topic = topic_prefix_ + "/connected";
    connection_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(conn_topic, 10);

    // ─────────────────────────────────────────────────────────────
    // 1. Safety DB Block (Default: DB1)
    // ─────────────────────────────────────────────────────────────
    DBBlock safety_block;
    safety_block.db_number = db_safety_no_;
    safety_block.start_byte = 0;
    safety_block.size_bytes = 4;
    safety_block.signals = {
        {"back_estop",             0, 3, false},
        {"reset",                  0, 3, false},
        {"work_over",              0, 4, false},
        {"front_estop",            0, 5, false},
        {"STO",                    1, 0, false},
        {"front_lidar_warn",       1, 5, false},
        {"back_lidar_warn",        1, 6, false},
        {"back_lidar_trip",        1, 7, false},
        {"front_lidar_trip",       2, 0, false},
        {"front_lidar_alarm",      2, 1, false},
        {"back_lidar_alarm",       2, 2, false},
        {"front_bumper",           2, 3, false},
        {"back_bumper",            2, 4, false},
        {"emerg_lidar_sto",        2, 5, false},
        {"yellow_light_indicator", 2, 6, false},
        {"green_light_indicator",  2, 7, false},
        {"reset_light",            3, 0, false},
        {"auto",                   3, 1, false},
        {"manual",                 3, 2, false}
    };

    if (publish_raw_db_) {
        std::string raw_topic = topic_prefix_ + "/db" + std::to_string(safety_block.db_number) + "/raw";
        safety_block.raw_pub = this->create_publisher<std_msgs::msg::UInt8MultiArray>(raw_topic, 10);
    }
    db_blocks_.push_back(safety_block);

    // ─────────────────────────────────────────────────────────────
    // 2. Actuator DB Block (Default: DB3) - Read & Trigger/Write
    // ─────────────────────────────────────────────────────────────
    DBBlock actuator_block;
    actuator_block.db_number = db_actuator_no_;
    actuator_block.start_byte = 0;
    actuator_block.size_bytes = db_actuator_size_;
    actuator_block.signals = {
        {"manual_act_forward", 0, 0, true},
        {"manual_act_reverse", 0, 1, true},
        {"act_fwd_complete",   0, 2, true},
        {"act_rev_complete",   0, 3, true},
        {"safety_bypass",      0, 4, true},
        {"tag_done",           0, 5, true},
        {"field_1",            0, 6, true},
        {"field_2",            0, 7, true},
        {"front_right_light",  1, 0, true},
        {"front_left_light",   1, 1, true},
        {"back_right_light",   1, 2, true},
        {"back_left_light",    1, 3, true},
        {"enable_sto",         1, 4, true},
        {"battery_sto",        1, 6, true},
        {"auto_act_fwd",       2, 0, true},
        {"auto_act_reverse",   2, 1, true}
    };

    if (publish_raw_db_) {
        std::string raw_topic = topic_prefix_ + "/db" + std::to_string(actuator_block.db_number) + "/raw";
        actuator_block.raw_pub = this->create_publisher<std_msgs::msg::UInt8MultiArray>(raw_topic, 10);
    }
    db_blocks_.push_back(actuator_block);

    // Create publishers for all signals across all DB blocks
    for (const auto & block : db_blocks_) {
        for (const auto & sig : block.signals) {
            std::string topic = topic_prefix_ + "/" + sig.name;
            if (signal_publishers_.find(sig.name) == signal_publishers_.end()) {
                signal_publishers_[sig.name] = this->create_publisher<std_msgs::msg::Bool>(topic, 10);
            }
        }
    }

    // Legacy / Convenience alias publishers
    if (signal_publishers_.find("act_forward") == signal_publishers_.end()) {
        signal_publishers_["act_forward"] = this->create_publisher<std_msgs::msg::Bool>(topic_prefix_ + "/act_forward", 10);
    }
    if (signal_publishers_.find("act_reverse") == signal_publishers_.end()) {
        signal_publishers_["act_reverse"] = this->create_publisher<std_msgs::msg::Bool>(topic_prefix_ + "/act_reverse", 10);
    }
    if (signal_publishers_.find("auto_act_forward") == signal_publishers_.end()) {
        signal_publishers_["auto_act_forward"] = this->create_publisher<std_msgs::msg::Bool>(topic_prefix_ + "/auto_act_forward", 10);
    }

    // Create subscribers for writable / trigger signals
    rclcpp::SubscriptionOptions sub_options;
    sub_options.ignore_local_publications = true;

    for (const auto & block : db_blocks_) {
        for (const auto & sig : block.signals) {
            if (sig.writable) {
                int db_num = block.db_number;
                int byte_off = sig.byte_offset;
                int bit_off = sig.bit_offset;
                std::string sig_name = sig.name;

                auto cb = [this, db_num, byte_off, bit_off, sig_name](const std_msgs::msg::Bool::SharedPtr msg) {
                    RCLCPP_INFO(this->get_logger(), "Trigger received for '%s' -> setting DB%d.DBX%d.%d = %s",
                                sig_name.c_str(), db_num, byte_off, bit_off, msg->data ? "TRUE" : "FALSE");
                    this->write_bit(db_num, byte_off, bit_off, msg->data);
                };

                // 1. Direct status topic subscription (e.g. /plc/manual_act_forward)
                std::string direct_topic = topic_prefix_ + "/" + sig.name;
                subscriptions_.push_back(
                    this->create_subscription<std_msgs::msg::Bool>(direct_topic, 10, cb, sub_options)
                );

                // 2. Command topic subscription (e.g. /plc/cmd/manual_act_forward)
                std::string cmd_topic = topic_prefix_ + "/cmd/" + sig.name;
                subscriptions_.push_back(
                    this->create_subscription<std_msgs::msg::Bool>(cmd_topic, 10, cb)
                );
            }
        }
    }

    // Convenience alias subscribers:
    // act_forward (DB3.DBX0.0)
    auto cb_act_fwd = [this](const std_msgs::msg::Bool::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Trigger alias 'act_forward' -> DB%d.DBX0.0 = %s",
                    this->db_actuator_no_, msg->data ? "TRUE" : "FALSE");
        this->write_bit(this->db_actuator_no_, 0, 0, msg->data);
    };
    subscriptions_.push_back(this->create_subscription<std_msgs::msg::Bool>(topic_prefix_ + "/act_forward", 10, cb_act_fwd, sub_options));
    subscriptions_.push_back(this->create_subscription<std_msgs::msg::Bool>(topic_prefix_ + "/cmd/act_forward", 10, cb_act_fwd));

    // act_reverse (DB3.DBX0.1)
    auto cb_act_rev = [this](const std_msgs::msg::Bool::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Trigger alias 'act_reverse' -> DB%d.DBX0.1 = %s",
                    this->db_actuator_no_, msg->data ? "TRUE" : "FALSE");
        this->write_bit(this->db_actuator_no_, 0, 1, msg->data);
    };
    subscriptions_.push_back(this->create_subscription<std_msgs::msg::Bool>(topic_prefix_ + "/act_reverse", 10, cb_act_rev, sub_options));
    subscriptions_.push_back(this->create_subscription<std_msgs::msg::Bool>(topic_prefix_ + "/cmd/act_reverse", 10, cb_act_rev));

    // auto_act_forward (DB3.DBX2.0)
    auto cb_auto_fwd = [this](const std_msgs::msg::Bool::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Trigger alias 'auto_act_forward' -> DB%d.DBX2.0 = %s",
                    this->db_actuator_no_, msg->data ? "TRUE" : "FALSE");
        this->write_bit(this->db_actuator_no_, 2, 0, msg->data);
    };
    subscriptions_.push_back(this->create_subscription<std_msgs::msg::Bool>(topic_prefix_ + "/auto_act_forward", 10, cb_auto_fwd, sub_options));
    subscriptions_.push_back(this->create_subscription<std_msgs::msg::Bool>(topic_prefix_ + "/cmd/auto_act_forward", 10, cb_auto_fwd));

    RCLCPP_INFO(get_logger(), "Configured %zu DB blocks with %zu signal topics and %zu trigger subscriptions under '%s/*'",
                db_blocks_.size(), signal_publishers_.size(), subscriptions_.size(), topic_prefix_.c_str());
}

bool PLCBridgeNode::write_bit(int db_number, int byte_offset, int bit_offset, bool value)
{
    std::lock_guard<std::recursive_mutex> lock(client_mutex_);
    if (!is_connected_ || !client_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Cannot write DB%d.DBX%d.%d: PLC is not connected",
            db_number, byte_offset, bit_offset
        );
        return false;
    }

    uint8_t byte_val = 0;
    int read_res = Cli_DBRead(client_, db_number, byte_offset, 1, &byte_val);
    if (read_res != 0) {
        RCLCPP_ERROR(
            get_logger(),
            "Failed reading DB%d byte %d before write (S7 error %d). Resetting connection.",
            db_number, byte_offset, read_res
        );
        disconnect_plc();
        return false;
    }

    if (value) {
        byte_val |= static_cast<uint8_t>(1 << bit_offset);
    } else {
        byte_val &= static_cast<uint8_t>(~(1 << bit_offset));
    }

    int write_res = Cli_DBWrite(client_, db_number, byte_offset, 1, &byte_val);
    if (write_res != 0) {
        RCLCPP_ERROR(
            get_logger(),
            "Failed writing DB%d byte %d (S7 error %d). Resetting connection.",
            db_number, byte_offset, write_res
        );
        disconnect_plc();
        return false;
    }

    RCLCPP_INFO(
        get_logger(),
        "✅ PLC WRITE OK: DB%d.DBX%d.%d = %s (byte: 0x%02X)",
        db_number, byte_offset, bit_offset, value ? "TRUE" : "FALSE", byte_val
    );
    return true;
}

bool PLCBridgeNode::extract_bit(const uint8_t * buffer, int byte_index, int bit_index)
{
    return (buffer[byte_index] & (1 << bit_index)) != 0;
}

void PLCBridgeNode::disconnect_plc()
{
    std::lock_guard<std::recursive_mutex> lock(client_mutex_);
    if (client_ && is_connected_) {
        Cli_Disconnect(client_);
    }
    is_connected_ = false;
}

bool PLCBridgeNode::ensure_connected()
{
    std::lock_guard<std::recursive_mutex> lock(client_mutex_);
    if (is_connected_) {
        int connected = 0;
        Cli_GetConnected(client_, &connected);
        if (connected) {
            return true;
        }
        RCLCPP_WARN(get_logger(), "Lost connection to PLC at %s", plc_ip_.c_str());
        disconnect_plc();
    }

    // Rate-limit connection attempts
    rclcpp::Time now = this->now();
    if ((now - last_reconnect_attempt_).seconds() < reconnect_interval_sec_) {
        return false;
    }
    last_reconnect_attempt_ = now;

    int res = Cli_ConnectTo(client_, plc_ip_.c_str(), plc_rack_, plc_slot_);
    if (res == 0) {
        is_connected_ = true;
        RCLCPP_INFO(get_logger(), "Connected to PLC at %s (Rack: %d, Slot: %d)",
                    plc_ip_.c_str(), plc_rack_, plc_slot_);
        return true;
    } else {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Connecting to PLC at %s failed (S7 error code: %d). Will retry...",
            plc_ip_.c_str(), res
        );
        return false;
    }
}

void PLCBridgeNode::timer_callback()
{
    std::lock_guard<std::recursive_mutex> lock(client_mutex_);
    bool connected = ensure_connected();

    // Publish connection status
    auto conn_msg = std_msgs::msg::Bool();
    conn_msg.data = connected;
    connection_status_pub_->publish(conn_msg);

    if (!connected) {
        return;
    }

    // Read configured DB blocks
    for (const auto & block : db_blocks_) {
        std::vector<uint8_t> buffer(block.size_bytes, 0);

        int res = Cli_DBRead(client_, block.db_number, block.start_byte, block.size_bytes, buffer.data());
        if (res != 0) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Error reading DB %d (start: %d, size: %d): S7 error %d. Resetting connection.",
                block.db_number, block.start_byte, block.size_bytes, res
            );
            disconnect_plc();
            return;
        }

        // Publish individual boolean signals
        for (const auto & sig : block.signals) {
            if (sig.byte_offset < block.size_bytes) {
                auto msg = std_msgs::msg::Bool();
                msg.data = extract_bit(buffer.data(), sig.byte_offset, sig.bit_offset);
                signal_publishers_[sig.name]->publish(msg);

                // Publish aliases
                if (sig.name == "manual_act_forward" && signal_publishers_.count("act_forward")) {
                    signal_publishers_["act_forward"]->publish(msg);
                } else if (sig.name == "manual_act_reverse" && signal_publishers_.count("act_reverse")) {
                    signal_publishers_["act_reverse"]->publish(msg);
                } else if (sig.name == "auto_act_fwd" && signal_publishers_.count("auto_act_forward")) {
                    signal_publishers_["auto_act_forward"]->publish(msg);
                }
            } else {
                RCLCPP_WARN_ONCE(
                    get_logger(),
                    "Signal '%s' byte_offset %d exceeds DB %d buffer size %d!",
                    sig.name.c_str(), sig.byte_offset, block.db_number, block.size_bytes
                );
            }
        }

        // Publish raw DB byte array if enabled
        if (publish_raw_db_ && block.raw_pub) {
            auto raw_msg = std_msgs::msg::UInt8MultiArray();
            raw_msg.data = buffer;
            block.raw_pub->publish(raw_msg);
        }
    }
}

}  // namespace plc_communication
