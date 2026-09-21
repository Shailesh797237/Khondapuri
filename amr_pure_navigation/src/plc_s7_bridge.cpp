// ================================================================
//  plc_s7_bridge.cpp  v4 — all build errors fixed
//
//  FIXES:
//    1. S7Object = unsigned long → init with 0 not nullptr
//    2. YAML::Node ref → use value not reference
//    3. try/catch block structure fixed
//    4. extern "C" include fixed in header
// ================================================================

#include "amr_pure_navigation/plc_s7_bridge_node.hpp"
#include <sstream>
#include <iomanip>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace plc_s7_bridge
{

// ─── Constructor ──────────────────────────────────────────────────
PlcS7BridgeNode::PlcS7BridgeNode(const rclcpp::NodeOptions & options)
: Node("plc_s7_bridge", options)
{
  declare_parameter<std::string>("plc_ip",           "192.168.1.21");
  declare_parameter<int>        ("rack",             0);
  declare_parameter<int>        ("slot",             1);
  declare_parameter<std::string>("station_map_file", "");

  get_parameter("plc_ip",           plc_ip_);
  get_parameter("rack",             rack_);
  get_parameter("slot",             slot_);
  get_parameter("station_map_file", station_map_file_);

  loadConfig(station_map_file_);

  RCLCPP_INFO(get_logger(), "============================================");
  RCLCPP_INFO(get_logger(), "  PLC S7 Bridge v4 — S7-1200 via snap7 C++");
  RCLCPP_INFO(get_logger(), "  PLC      : %s  rack=%d slot=%d",
    plc_ip_.c_str(), rack_, slot_);
  RCLCPP_INFO(get_logger(), "  DB       : DB%d  cmd=DBW%d  fb=DBX%d.x",
    db_number_, command_offset_, feedback_offset_);
  RCLCPP_INFO(get_logger(), "  Stations : %zu", station_map_.size());
  RCLCPP_INFO(get_logger(), "============================================");
  RCLCPP_INFO(get_logger(), "  Manual commands via /plc_manual:");
  RCLCPP_INFO(get_logger(), "    INPUT:I0.0    read feedback bit");
  RCLCPP_INFO(get_logger(), "    STATUS        show all bits");
  RCLCPP_INFO(get_logger(), "============================================");

  done_pub_   = create_publisher<std_msgs::msg::String>("/plc_done",     10);
  status_pub_ = create_publisher<std_msgs::msg::String>("/plc_status",   10);
  io_pub_     = create_publisher<std_msgs::msg::String>("/plc_io_state", 10);

  station_sub_ = create_subscription<std_msgs::msg::String>(
    "/station_reached", 10,
    std::bind(&PlcS7BridgeNode::stationCallback, this, std::placeholders::_1));

  manual_sub_ = create_subscription<std_msgs::msg::String>(
    "/plc_manual", 10,
    std::bind(&PlcS7BridgeNode::manualCallback, this, std::placeholders::_1));

  diag_timer_ = create_wall_timer(1s,
    std::bind(&PlcS7BridgeNode::diagnosticLoop, this));

  // ── FIX 1: S7Object is unsigned long — init with 0 not nullptr ─
  client_ = Cli_Create();
  if (client_ == 0) {
    RCLCPP_FATAL(get_logger(), "Failed to create snap7 client!");
    return;
  }
  connectPLC();
}

// ─── Destructor ───────────────────────────────────────────────────
PlcS7BridgeNode::~PlcS7BridgeNode()
{
  if (connected_) {
    RCLCPP_INFO(get_logger(), "Shutdown — clearing command word");
    writeCommandWord(0);
  }
  disconnectPLC();
  // ── FIX 1: destroy and set to 0 not nullptr ──────────────────
  if (client_ != 0) {
    Cli_Destroy(&client_);
    client_ = 0;
  }
}

// ─── Connect ──────────────────────────────────────────────────────
bool PlcS7BridgeNode::connectPLC()
{
  std::lock_guard<std::mutex> lock(s7_mutex_);
  if (client_ == 0) return false;
  if (connected_) {
    Cli_Disconnect(client_);
    connected_ = false;
  }
  int rc = Cli_ConnectTo(client_, plc_ip_.c_str(), rack_, slot_);
  if (rc != 0) {
    RCLCPP_ERROR(get_logger(), "PLC connect failed: %s", errorText(rc).c_str());
    return false;
  }
  connected_ = true;
  RCLCPP_INFO(get_logger(), "✅ Connected to S7-1200 at %s", plc_ip_.c_str());
  publishStatus("CONNECTED:" + plc_ip_);
  return true;
}

void PlcS7BridgeNode::disconnectPLC()
{
  std::lock_guard<std::mutex> lock(s7_mutex_);
  if (client_ != 0 && connected_) {
    Cli_Disconnect(client_);
    connected_ = false;
  }
}

bool PlcS7BridgeNode::isConnected()
{
  if (client_ == 0 || !connected_) return false;
  int state = 0;
  return (Cli_GetConnected(client_, &state) == 0 && state != 0);
}

// ─── Write command word ───────────────────────────────────────────
bool PlcS7BridgeNode::writeCommandWord(int value)
{
  if (!connected_ || client_ == 0) {
    RCLCPP_WARN(get_logger(), "writeCommandWord: not connected");
    return false;
  }
  uint8_t buf[2];
  buf[0] = (value >> 8) & 0xFF;
  buf[1] = (value)      & 0xFF;
  std::lock_guard<std::mutex> lock(s7_mutex_);
  int rc = Cli_DBWrite(client_, db_number_, command_offset_, 2, buf);
  if (rc != 0) {
    RCLCPP_ERROR(get_logger(), "writeCommandWord DB%d.DBW%d=%d FAILED: %s",
      db_number_, command_offset_, value, errorText(rc).c_str());
    connected_ = false;
    return false;
  }
  RCLCPP_INFO(get_logger(),
    "📤 DB%d.DBW%d = %d", db_number_, command_offset_, value);
  return true;
}

// ─── Read command word ────────────────────────────────────────────
bool PlcS7BridgeNode::readCommandWord(int & value)
{
  if (!connected_ || client_ == 0) return false;
  uint8_t buf[2] = {0};
  std::lock_guard<std::mutex> lock(s7_mutex_);
  int rc = Cli_DBRead(client_, db_number_, command_offset_, 2, buf);
  if (rc != 0) { connected_ = false; return false; }
  value = (int)(int16_t)((buf[0] << 8) | buf[1]);
  return true;
}

// ─── Read one feedback bit ────────────────────────────────────────
bool PlcS7BridgeNode::readFeedbackBit(int bit_index, bool & value)
{
  if (!connected_ || client_ == 0) return false;
  uint8_t buf[1] = {0};
  {
    std::lock_guard<std::mutex> lock(s7_mutex_);
    int rc = Cli_DBRead(client_, db_number_, feedback_offset_, 1, buf);
    if (rc != 0) {
      RCLCPP_ERROR(get_logger(), "readFeedbackBit DB%d.DBX%d.%d FAILED: %s",
        db_number_, feedback_offset_, bit_index, errorText(rc).c_str());
      connected_ = false;
      return false;
    }
  }
  value = ((buf[0] >> bit_index) & 0x01) != 0;
  return true;
}

// ─── Read all 8 feedback bits ─────────────────────────────────────
bool PlcS7BridgeNode::readAllFeedbackBits(uint8_t & byte_val)
{
  if (!connected_ || client_ == 0) return false;
  uint8_t buf[1] = {0};
  std::lock_guard<std::mutex> lock(s7_mutex_);
  int rc = Cli_DBRead(client_, db_number_, feedback_offset_, 1, buf);
  if (rc != 0) { connected_ = false; return false; }
  byte_val = buf[0];
  return true;
}

// ─── Station Callback ─────────────────────────────────────────────
void PlcS7BridgeNode::stationCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string raw  = msg->data;
  std::string name = raw;
  std::string fb_hint = "";

  auto colon = raw.find(':');
  if (colon != std::string::npos) {
    name    = raw.substr(0, colon);
    fb_hint = raw.substr(colon + 1);
  }

  RCLCPP_INFO(get_logger(),
    "📍 Station arrived: [%s]  plc_feedback=%s",
    name.c_str(), fb_hint.empty() ? "none" : fb_hint.c_str());
  RCLCPP_INFO(get_logger(),
    "   IPC will write command to DB%d.DBW%d",
    db_number_, command_offset_);
  RCLCPP_INFO(get_logger(),
    "   IPC will read feedback from DB%d.DBX%d.x",
    db_number_, feedback_offset_);

  auto it = station_map_.find(name);
  if (it == station_map_.end()) {
    RCLCPP_WARN(get_logger(),
      "Station '%s' not found in station_map.yaml — check name matches command_map",
      name.c_str());
    std_msgs::msg::String done; done.data = name + ":NO_CONFIG";
    done_pub_->publish(done);
    return;
  }

  StationAction action = it->second;
  if (!fb_hint.empty() && action.input_i.empty()) {
    action.input_i     = fb_hint;
    action.wait_confirm = true;
  }

  std::thread([this, action]() { handleStation(action); }).detach();
}

// ─── Handle Station ───────────────────────────────────────────────
void PlcS7BridgeNode::handleStation(const StationAction & action)
{
  auto cit = command_map_.find(action.station_name);
  int cmd_value = (cit != command_map_.end()) ? cit->second : 0;

  if (cmd_value == 0) {
    RCLCPP_INFO(get_logger(), "[%s] cmd=0 → done immediately",
      action.station_name.c_str());
    std_msgs::msg::String done; done.data = action.station_name;
    done_pub_->publish(done);
    return;
  }

  // STEP 1: Write command
  RCLCPP_INFO(get_logger(), "⚙  [%s] → DB%d.DBW%d = %d",
    action.station_name.c_str(), db_number_, command_offset_, cmd_value);
  publishStatus(action.station_name + ":SENDING_CMD");

  if (!writeCommandWord(cmd_value)) {
    std_msgs::msg::String done; done.data = action.station_name + ":WRITE_FAIL";
    done_pub_->publish(done);
    return;
  }

  // STEP 2: Wait for feedback bit
  if (action.wait_confirm && !action.input_i.empty()) {
    auto iit = input_map_.find(action.input_i);
    if (iit != input_map_.end()) {
      int bit = iit->second;
      RCLCPP_INFO(get_logger(),
        "⏳ [%s] Waiting %s → DB%d.DBX%d.%d = TRUE (timeout=%.0fs)",
        action.station_name.c_str(), action.input_i.c_str(),
        db_number_, feedback_offset_, bit, confirm_timeout_s_);
      publishStatus(action.station_name + ":WAITING:" + action.input_i);

      auto start = std::chrono::steady_clock::now();
      while (rclcpp::ok()) {
        bool val = false;
        if (readFeedbackBit(bit, val) && val) {
          RCLCPP_INFO(get_logger(), "✅ [%s] %s = TRUE — PLC done!",
            action.station_name.c_str(), action.input_i.c_str());
          break;
        }
        double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - start).count();
        if (elapsed >= confirm_timeout_s_) {
          RCLCPP_WARN(get_logger(), "⚠ [%s] Timeout %.0fs",
            action.station_name.c_str(), confirm_timeout_s_);
          break;
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
          "⏳ [%s] Waiting %s ... %.0f/%.0fs",
          action.station_name.c_str(), action.input_i.c_str(),
          elapsed, confirm_timeout_s_);
        std::this_thread::sleep_for(100ms);
      }
    }
  }

  // STEP 3: Write 0 for write_zero_sec_
  RCLCPP_INFO(get_logger(), "🔄 [%s] Clearing cmd for %.0fs...",
    action.station_name.c_str(), write_zero_sec_);
  publishStatus(action.station_name + ":CLEARING");

  auto t0 = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    writeCommandWord(0);
    if (std::chrono::duration<double>(
          std::chrono::steady_clock::now() - t0).count() >= write_zero_sec_)
      break;
    std::this_thread::sleep_for(500ms);
  }

  // STEP 4: Done
  std_msgs::msg::String done; done.data = action.station_name;
  done_pub_->publish(done);
  publishStatus(action.station_name + ":DONE");
  RCLCPP_INFO(get_logger(), "✅ /plc_done → [%s]", action.station_name.c_str());
}

// ─── Manual Callback ──────────────────────────────────────────────
void PlcS7BridgeNode::manualCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string cmd = msg->data;
  RCLCPP_INFO(get_logger(), "🖐 Manual: [%s]", cmd.c_str());

  std::vector<std::string> tok;
  std::istringstream ss(cmd);
  std::string t;
  while (std::getline(ss, t, ':')) tok.push_back(t);
  if (tok.empty()) return;

  // STATUS
  if (tok[0] == "STATUS") {
    uint8_t fbyte = 0;
    int cur_cmd   = 0;
    readAllFeedbackBits(fbyte);
    readCommandWord(cur_cmd);

    std::string active = "idle";
    for (auto & kv : command_map_)
      if (kv.second == cur_cmd) { active = kv.first; break; }

    std::ostringstream oss;
    oss << "Command: DB" << db_number_ << ".DBW" << command_offset_
        << " = " << cur_cmd << " (" << active << ")\n";
    oss << "Feedback bits:\n";
    for (auto & kv : input_map_) {
      bool bit = ((fbyte >> kv.second) & 1) != 0;
      oss << "  " << kv.first << " = " << (bit ? "TRUE" : "FALSE") << "\n";
    }
    RCLCPP_INFO(get_logger(), "\n%s", oss.str().c_str());
    publishStatus(oss.str());
    return;
  }

  // INPUT:I0.x
  if (tok[0] == "INPUT" && tok.size() >= 2) {
    auto it = input_map_.find(tok[1]);
    if (it == input_map_.end()) {
      RCLCPP_WARN(get_logger(), "Unknown input '%s'", tok[1].c_str());
      return;
    }
    bool val = false;
    if (readFeedbackBit(it->second, val))
      RCLCPP_INFO(get_logger(), "📖 %s = %s",
        tok[1].c_str(), val ? "TRUE ✅" : "FALSE ⬜");
    return;
  }

  RCLCPP_WARN(get_logger(), "Commands: INPUT:I0.x  |  STATUS");
}

// ─── Diagnostic Loop ──────────────────────────────────────────────
void PlcS7BridgeNode::diagnosticLoop()
{
  if (!isConnected()) {
    if (connected_) {
      connected_ = false;
      RCLCPP_WARN(get_logger(), "PLC disconnected — reconnecting...");
      publishStatus("DISCONNECTED");
    }
    static auto last = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last).count() >= reconnect_sec_) {
      last = std::chrono::steady_clock::now();
      connectPLC();
    }
    return;
  }

  uint8_t fbyte = 0;
  int cur_cmd   = 0;
  if (readAllFeedbackBits(fbyte) && readCommandWord(cur_cmd)) {
    std::string active = "idle";
    for (auto & kv : command_map_)
      if (kv.second == cur_cmd) { active = kv.first; break; }

    std::ostringstream oss;
    oss << "CMD=" << cur_cmd << "(" << active << ")  ";
    for (int b = 0; b < 8; b++)
      oss << "I0." << b << ":" << (((fbyte >> b) & 1) ? "1" : "0") << " ";

    std_msgs::msg::String m; m.data = oss.str();
    io_pub_->publish(m);
  }
}

// ─── Load Config ──────────────────────────────────────────────────
bool PlcS7BridgeNode::loadConfig(const std::string & path)
{
  if (path.empty()) {
    RCLCPP_WARN(get_logger(), "No config file"); return false;
  }
  std::ifstream f(path);
  if (!f.good()) {
    RCLCPP_ERROR(get_logger(), "Cannot open: %s", path.c_str()); return false;
  }

  try {
    YAML::Node root = YAML::LoadFile(path);

    // ── FIX 2: use value copy not reference for YAML::Node ───────
    if (root["plc_config"]) {
      YAML::Node c = root["plc_config"];   // value, not reference
      db_number_         = c["db_number"]         ? c["db_number"].as<int>()          : db_number_;
      command_offset_    = c["command_offset"]     ? c["command_offset"].as<int>()     : command_offset_;
      feedback_offset_   = c["feedback_offset"]    ? c["feedback_offset"].as<int>()    : feedback_offset_;
      confirm_timeout_s_ = c["confirm_timeout_s"]  ? c["confirm_timeout_s"].as<double>(): confirm_timeout_s_;
      write_zero_sec_    = c["write_zero_sec"]     ? c["write_zero_sec"].as<double>()  : write_zero_sec_;
      RCLCPP_INFO(get_logger(),
        "  Config: DB%d  cmd=DBW%d  fb=DBX%d.x  timeout=%.0fs  zero=%.0fs",
        db_number_, command_offset_, feedback_offset_,
        confirm_timeout_s_, write_zero_sec_);
    }

    // command_map
    if (root["command_map"]) {
      YAML::Node cm = root["command_map"];
      for (auto it = cm.begin(); it != cm.end(); ++it) {
        std::string k = it->first.as<std::string>();
        int v         = it->second.as<int>();
        command_map_[k] = v;
        RCLCPP_INFO(get_logger(), "  Cmd: %-15s → %d", k.c_str(), v);
      }
    }

    // feedback_map
    if (root["feedback_map"]) {
      YAML::Node fm = root["feedback_map"];
      for (auto it = fm.begin(); it != fm.end(); ++it) {
        std::string k = it->first.as<std::string>();
        int v         = it->second.as<int>();
        input_map_[k] = v;
        RCLCPP_INFO(get_logger(), "  Feedback: %s → bit %d", k.c_str(), v);
      }
    }

    // ── FIX 3: stations inside same try block (no missing brace) ─
    if (root["stations"]) {
      for (auto s : root["stations"]) {
        StationAction a;
        a.station_name  = s["name"].as<std::string>();
        a.input_i       = s["wait_input"]   ? s["wait_input"].as<std::string>()  : "";
        a.wait_confirm  = s["wait_confirm"] ? s["wait_confirm"].as<bool>()       : false;
        station_map_[a.station_name] = a;
        RCLCPP_INFO(get_logger(),
          "  Station %-15s  wait=%s  input=%s",
          a.station_name.c_str(),
          a.wait_confirm ? "YES" : "NO",
          a.input_i.empty() ? "none" : a.input_i.c_str());
      }
    }

    RCLCPP_INFO(get_logger(), "Loaded %zu stations", station_map_.size());
    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "YAML error: %s", e.what());
    return false;
  }
}

// ─── Helpers ──────────────────────────────────────────────────────
void PlcS7BridgeNode::publishStatus(const std::string & text)
{
  std_msgs::msg::String m; m.data = text; status_pub_->publish(m);
}

std::string PlcS7BridgeNode::errorText(int code)
{
  char buf[256] = {0};
  Cli_ErrorText(code, buf, sizeof(buf));
  return std::string(buf);
}

}  // namespace plc_s7_bridge

// ─── main ─────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<plc_s7_bridge::PlcS7BridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}












// // ================================================================
// //  plc_s7_bridge.cpp  v4 — all build errors fixed
// //
// //  FIXES:
// //    1. S7Object = unsigned long → init with 0 not nullptr
// //    2. YAML::Node ref → use value not reference
// //    3. try/catch block structure fixed
// //    4. extern "C" include fixed in header
// // ================================================================

// #include "amr_pure_navigation/plc_s7_bridge_node.hpp"
// #include <sstream>
// #include <iomanip>
// #include <chrono>
// #include <thread>

// using namespace std::chrono_literals;

// namespace plc_s7_bridge
// {

// // ─── Constructor ──────────────────────────────────────────────────
// PlcS7BridgeNode::PlcS7BridgeNode(const rclcpp::NodeOptions & options)
// : Node("plc_s7_bridge", options)
// {
//   declare_parameter<std::string>("plc_ip",           "192.168.1.21");
//   declare_parameter<int>        ("rack",             0);
//   declare_parameter<int>        ("slot",             1);
//   declare_parameter<std::string>("station_map_file", "");

//   get_parameter("plc_ip",           plc_ip_);
//   get_parameter("rack",             rack_);
//   get_parameter("slot",             slot_);
//   get_parameter("station_map_file", station_map_file_);

//   loadConfig(station_map_file_);

//   RCLCPP_INFO(get_logger(), "============================================");
//   RCLCPP_INFO(get_logger(), "  PLC S7 Bridge v4 — S7-1200 via snap7 C++");
//   RCLCPP_INFO(get_logger(), "  PLC      : %s  rack=%d slot=%d",
//     plc_ip_.c_str(), rack_, slot_);
//   RCLCPP_INFO(get_logger(), "  DB       : DB%d  cmd=DBW%d  fb=DBX%d.x",
//     db_number_, command_offset_, feedback_offset_);
//   RCLCPP_INFO(get_logger(), "  Stations : %zu", station_map_.size());
//   RCLCPP_INFO(get_logger(), "============================================");
//   RCLCPP_INFO(get_logger(), "  Manual commands via /plc_manual:");
//   RCLCPP_INFO(get_logger(), "    INPUT:I0.0    read feedback bit");
//   RCLCPP_INFO(get_logger(), "    STATUS        show all bits");
//   RCLCPP_INFO(get_logger(), "============================================");

//   done_pub_   = create_publisher<std_msgs::msg::String>("/plc_done",     10);
//   status_pub_ = create_publisher<std_msgs::msg::String>("/plc_status",   10);
//   io_pub_     = create_publisher<std_msgs::msg::String>("/plc_io_state", 10);

//   station_sub_ = create_subscription<std_msgs::msg::String>(
//     "/station_reached", 10,
//     std::bind(&PlcS7BridgeNode::stationCallback, this, std::placeholders::_1));

//   manual_sub_ = create_subscription<std_msgs::msg::String>(
//     "/plc_manual", 10,
//     std::bind(&PlcS7BridgeNode::manualCallback, this, std::placeholders::_1));

//   diag_timer_ = create_wall_timer(1s,
//     std::bind(&PlcS7BridgeNode::diagnosticLoop, this));

//   // ── FIX 1: S7Object is unsigned long — init with 0 not nullptr ─
//   client_ = Cli_Create();
//   if (client_ == 0) {
//     RCLCPP_FATAL(get_logger(), "Failed to create snap7 client!");
//     return;
//   }
//   connectPLC();
// }

// // ─── Destructor ───────────────────────────────────────────────────
// PlcS7BridgeNode::~PlcS7BridgeNode()
// {
//   if (connected_) {
//     RCLCPP_INFO(get_logger(), "Shutdown — clearing command word");
//     writeCommandWord(0);
//   }
//   disconnectPLC();
//   // ── FIX 1: destroy and set to 0 not nullptr ──────────────────
//   if (client_ != 0) {
//     Cli_Destroy(&client_);
//     client_ = 0;
//   }
// }

// // ─── Connect ──────────────────────────────────────────────────────
// bool PlcS7BridgeNode::connectPLC()
// {
//   std::lock_guard<std::mutex> lock(s7_mutex_);
//   if (client_ == 0) return false;
//   if (connected_) {
//     Cli_Disconnect(client_);
//     connected_ = false;
//   }
//   int rc = Cli_ConnectTo(client_, plc_ip_.c_str(), rack_, slot_);
//   if (rc != 0) {
//     RCLCPP_ERROR(get_logger(), "PLC connect failed: %s", errorText(rc).c_str());
//     return false;
//   }
//   connected_ = true;
//   RCLCPP_INFO(get_logger(), "✅ Connected to S7-1200 at %s", plc_ip_.c_str());
//   publishStatus("CONNECTED:" + plc_ip_);
//   return true;
// }

// void PlcS7BridgeNode::disconnectPLC()
// {
//   std::lock_guard<std::mutex> lock(s7_mutex_);
//   if (client_ != 0 && connected_) {
//     Cli_Disconnect(client_);
//     connected_ = false;
//   }
// }

// bool PlcS7BridgeNode::isConnected()
// {
//   if (client_ == 0 || !connected_) return false;
//   int state = 0;
//   return (Cli_GetConnected(client_, &state) == 0 && state != 0);
// }

// // ─── Write command word ───────────────────────────────────────────
// bool PlcS7BridgeNode::writeCommandWord(int value)
// {
//   if (!connected_ || client_ == 0) {
//     RCLCPP_WARN(get_logger(), "writeCommandWord: not connected");
//     return false;
//   }
//   uint8_t buf[2];
//   buf[0] = (value >> 8) & 0xFF;
//   buf[1] = (value)      & 0xFF;
//   std::lock_guard<std::mutex> lock(s7_mutex_);
//   int rc = Cli_DBWrite(client_, db_number_, command_offset_, 2, buf);
//   if (rc != 0) {
//     RCLCPP_ERROR(get_logger(), "writeCommandWord DB%d.DBW%d=%d FAILED: %s",
//       db_number_, command_offset_, value, errorText(rc).c_str());
//     connected_ = false;
//     return false;
//   }
//   RCLCPP_INFO(get_logger(),
//     "📤 DB%d.DBW%d = %d", db_number_, command_offset_, value);
//   return true;
// }

// // ─── Read command word ────────────────────────────────────────────
// bool PlcS7BridgeNode::readCommandWord(int & value)
// {
//   if (!connected_ || client_ == 0) return false;
//   uint8_t buf[2] = {0};
//   std::lock_guard<std::mutex> lock(s7_mutex_);
//   int rc = Cli_DBRead(client_, db_number_, command_offset_, 2, buf);
//   if (rc != 0) { connected_ = false; return false; }
//   value = (int)(int16_t)((buf[0] << 8) | buf[1]);
//   return true;
// }

// // ─── Read one feedback bit ────────────────────────────────────────
// bool PlcS7BridgeNode::readFeedbackBit(int bit_index, bool & value)
// {
//   if (!connected_ || client_ == 0) return false;
//   uint8_t buf[1] = {0};
//   {
//     std::lock_guard<std::mutex> lock(s7_mutex_);
//     int rc = Cli_DBRead(client_, db_number_, feedback_offset_, 1, buf);
//     if (rc != 0) {
//       RCLCPP_ERROR(get_logger(), "readFeedbackBit DB%d.DBX%d.%d FAILED: %s",
//         db_number_, feedback_offset_, bit_index, errorText(rc).c_str());
//       connected_ = false;
//       return false;
//     }
//   }
//   value = ((buf[0] >> bit_index) & 0x01) != 0;
//   return true;
// }

// // ─── Read all 8 feedback bits ─────────────────────────────────────
// bool PlcS7BridgeNode::readAllFeedbackBits(uint8_t & byte_val)
// {
//   if (!connected_ || client_ == 0) return false;
//   uint8_t buf[1] = {0};
//   std::lock_guard<std::mutex> lock(s7_mutex_);
//   int rc = Cli_DBRead(client_, db_number_, feedback_offset_, 1, buf);
//   if (rc != 0) { connected_ = false; return false; }
//   byte_val = buf[0];
//   return true;
// }

// // ─── Station Callback ─────────────────────────────────────────────
// void PlcS7BridgeNode::stationCallback(const std_msgs::msg::String::SharedPtr msg)
// {
//   std::string raw  = msg->data;
//   std::string name = raw;
//   std::string fb_hint = "";

//   auto colon = raw.find(':');
//   if (colon != std::string::npos) {
//     name    = raw.substr(0, colon);
//     fb_hint = raw.substr(colon + 1);
//   }

//   RCLCPP_INFO(get_logger(), "📍 Station: [%s]  feedback=%s",
//     name.c_str(), fb_hint.empty() ? "none" : fb_hint.c_str());

//   auto it = station_map_.find(name);
//   if (it == station_map_.end()) {
//     RCLCPP_WARN(get_logger(), "Station '%s' not in map", name.c_str());
//     std_msgs::msg::String done; done.data = name + ":NO_CONFIG";
//     done_pub_->publish(done);
//     return;
//   }

//   StationAction action = it->second;
//   if (!fb_hint.empty() && action.input_i.empty()) {
//     action.input_i     = fb_hint;
//     action.wait_confirm = true;
//   }

//   std::thread([this, action]() { handleStation(action); }).detach();
// }

// // ─── Handle Station ───────────────────────────────────────────────
// void PlcS7BridgeNode::handleStation(const StationAction & action)
// {
//   auto cit = command_map_.find(action.station_name);
//   int cmd_value = (cit != command_map_.end()) ? cit->second : 0;

//   if (cmd_value == 0) {
//     RCLCPP_INFO(get_logger(), "[%s] cmd=0 → done immediately",
//       action.station_name.c_str());
//     std_msgs::msg::String done; done.data = action.station_name;
//     done_pub_->publish(done);
//     return;
//   }

//   // STEP 1: Write command
//   RCLCPP_INFO(get_logger(), "⚙  [%s] → DB%d.DBW%d = %d",
//     action.station_name.c_str(), db_number_, command_offset_, cmd_value);
//   publishStatus(action.station_name + ":SENDING_CMD");

//   if (!writeCommandWord(cmd_value)) {
//     std_msgs::msg::String done; done.data = action.station_name + ":WRITE_FAIL";
//     done_pub_->publish(done);
//     return;
//   }

//   // STEP 2: Wait for feedback bit
//   if (action.wait_confirm && !action.input_i.empty()) {
//     auto iit = input_map_.find(action.input_i);
//     if (iit != input_map_.end()) {
//       int bit = iit->second;
//       RCLCPP_INFO(get_logger(),
//         "⏳ [%s] Waiting %s → DB%d.DBX%d.%d = TRUE (timeout=%.0fs)",
//         action.station_name.c_str(), action.input_i.c_str(),
//         db_number_, feedback_offset_, bit, confirm_timeout_s_);
//       publishStatus(action.station_name + ":WAITING:" + action.input_i);

//       auto start = std::chrono::steady_clock::now();
//       while (rclcpp::ok()) {
//         bool val = false;
//         if (readFeedbackBit(bit, val) && val) {
//           RCLCPP_INFO(get_logger(), "✅ [%s] %s = TRUE — PLC done!",
//             action.station_name.c_str(), action.input_i.c_str());
//           break;
//         }
//         double elapsed = std::chrono::duration<double>(
//           std::chrono::steady_clock::now() - start).count();
//         if (elapsed >= confirm_timeout_s_) {
//           RCLCPP_WARN(get_logger(), "⚠ [%s] Timeout %.0fs",
//             action.station_name.c_str(), confirm_timeout_s_);
//           break;
//         }
//         RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
//           "⏳ [%s] Waiting %s ... %.0f/%.0fs",
//           action.station_name.c_str(), action.input_i.c_str(),
//           elapsed, confirm_timeout_s_);
//         std::this_thread::sleep_for(100ms);
//       }
//     }
//   }

//   // STEP 3: Write 0 for write_zero_sec_
//   RCLCPP_INFO(get_logger(), "🔄 [%s] Clearing cmd for %.0fs...",
//     action.station_name.c_str(), write_zero_sec_);
//   publishStatus(action.station_name + ":CLEARING");

//   auto t0 = std::chrono::steady_clock::now();
//   while (rclcpp::ok()) {
//     writeCommandWord(0);
//     if (std::chrono::duration<double>(
//           std::chrono::steady_clock::now() - t0).count() >= write_zero_sec_)
//       break;
//     std::this_thread::sleep_for(500ms);
//   }

//   // STEP 4: Done
//   std_msgs::msg::String done; done.data = action.station_name;
//   done_pub_->publish(done);
//   publishStatus(action.station_name + ":DONE");
//   RCLCPP_INFO(get_logger(), "✅ /plc_done → [%s]", action.station_name.c_str());
// }

// // ─── Manual Callback ──────────────────────────────────────────────
// void PlcS7BridgeNode::manualCallback(const std_msgs::msg::String::SharedPtr msg)
// {
//   std::string cmd = msg->data;
//   RCLCPP_INFO(get_logger(), "🖐 Manual: [%s]", cmd.c_str());

//   std::vector<std::string> tok;
//   std::istringstream ss(cmd);
//   std::string t;
//   while (std::getline(ss, t, ':')) tok.push_back(t);
//   if (tok.empty()) return;

//   // STATUS
//   if (tok[0] == "STATUS") {
//     uint8_t fbyte = 0;
//     int cur_cmd   = 0;
//     readAllFeedbackBits(fbyte);
//     readCommandWord(cur_cmd);

//     std::string active = "idle";
//     for (auto & kv : command_map_)
//       if (kv.second == cur_cmd) { active = kv.first; break; }

//     std::ostringstream oss;
//     oss << "Command: DB" << db_number_ << ".DBW" << command_offset_
//         << " = " << cur_cmd << " (" << active << ")\n";
//     oss << "Feedback bits:\n";
//     for (auto & kv : input_map_) {
//       bool bit = ((fbyte >> kv.second) & 1) != 0;
//       oss << "  " << kv.first << " = " << (bit ? "TRUE" : "FALSE") << "\n";
//     }
//     RCLCPP_INFO(get_logger(), "\n%s", oss.str().c_str());
//     publishStatus(oss.str());
//     return;
//   }

//   // INPUT:I0.x
//   if (tok[0] == "INPUT" && tok.size() >= 2) {
//     auto it = input_map_.find(tok[1]);
//     if (it == input_map_.end()) {
//       RCLCPP_WARN(get_logger(), "Unknown input '%s'", tok[1].c_str());
//       return;
//     }
//     bool val = false;
//     if (readFeedbackBit(it->second, val))
//       RCLCPP_INFO(get_logger(), "📖 %s = %s",
//         tok[1].c_str(), val ? "TRUE ✅" : "FALSE ⬜");
//     return;
//   }

//   RCLCPP_WARN(get_logger(), "Commands: INPUT:I0.x  |  STATUS");
// }

// // ─── Diagnostic Loop ──────────────────────────────────────────────
// void PlcS7BridgeNode::diagnosticLoop()
// {
//   if (!isConnected()) {
//     if (connected_) {
//       connected_ = false;
//       RCLCPP_WARN(get_logger(), "PLC disconnected — reconnecting...");
//       publishStatus("DISCONNECTED");
//     }
//     static auto last = std::chrono::steady_clock::now();
//     if (std::chrono::duration<double>(
//           std::chrono::steady_clock::now() - last).count() >= reconnect_sec_) {
//       last = std::chrono::steady_clock::now();
//       connectPLC();
//     }
//     return;
//   }

//   uint8_t fbyte = 0;
//   int cur_cmd   = 0;
//   if (readAllFeedbackBits(fbyte) && readCommandWord(cur_cmd)) {
//     std::string active = "idle";
//     for (auto & kv : command_map_)
//       if (kv.second == cur_cmd) { active = kv.first; break; }

//     std::ostringstream oss;
//     oss << "CMD=" << cur_cmd << "(" << active << ")  ";
//     for (int b = 0; b < 8; b++)
//       oss << "I0." << b << ":" << (((fbyte >> b) & 1) ? "1" : "0") << " ";

//     std_msgs::msg::String m; m.data = oss.str();
//     io_pub_->publish(m);
//   }
// }

// // ─── Load Config ──────────────────────────────────────────────────
// bool PlcS7BridgeNode::loadConfig(const std::string & path)
// {
//   if (path.empty()) {
//     RCLCPP_WARN(get_logger(), "No config file"); return false;
//   }
//   std::ifstream f(path);
//   if (!f.good()) {
//     RCLCPP_ERROR(get_logger(), "Cannot open: %s", path.c_str()); return false;
//   }

//   try {
//     YAML::Node root = YAML::LoadFile(path);

//     // ── FIX 2: use value copy not reference for YAML::Node ───────
//     if (root["plc_config"]) {
//       YAML::Node c = root["plc_config"];   // value, not reference
//       db_number_         = c["db_number"]         ? c["db_number"].as<int>()          : db_number_;
//       command_offset_    = c["command_offset"]     ? c["command_offset"].as<int>()     : command_offset_;
//       feedback_offset_   = c["feedback_offset"]    ? c["feedback_offset"].as<int>()    : feedback_offset_;
//       confirm_timeout_s_ = c["confirm_timeout_s"]  ? c["confirm_timeout_s"].as<double>(): confirm_timeout_s_;
//       write_zero_sec_    = c["write_zero_sec"]     ? c["write_zero_sec"].as<double>()  : write_zero_sec_;
//       RCLCPP_INFO(get_logger(),
//         "  Config: DB%d  cmd=DBW%d  fb=DBX%d.x  timeout=%.0fs  zero=%.0fs",
//         db_number_, command_offset_, feedback_offset_,
//         confirm_timeout_s_, write_zero_sec_);
//     }

//     // command_map
//     if (root["command_map"]) {
//       YAML::Node cm = root["command_map"];
//       for (auto it = cm.begin(); it != cm.end(); ++it) {
//         std::string k = it->first.as<std::string>();
//         int v         = it->second.as<int>();
//         command_map_[k] = v;
//         RCLCPP_INFO(get_logger(), "  Cmd: %-15s → %d", k.c_str(), v);
//       }
//     }

//     // feedback_map
//     if (root["feedback_map"]) {
//       YAML::Node fm = root["feedback_map"];
//       for (auto it = fm.begin(); it != fm.end(); ++it) {
//         std::string k = it->first.as<std::string>();
//         int v         = it->second.as<int>();
//         input_map_[k] = v;
//         RCLCPP_INFO(get_logger(), "  Feedback: %s → bit %d", k.c_str(), v);
//       }
//     }

//     // ── FIX 3: stations inside same try block (no missing brace) ─
//     if (root["stations"]) {
//       for (auto s : root["stations"]) {
//         StationAction a;
//         a.station_name  = s["name"].as<std::string>();
//         a.input_i       = s["wait_input"]   ? s["wait_input"].as<std::string>()  : "";
//         a.wait_confirm  = s["wait_confirm"] ? s["wait_confirm"].as<bool>()       : false;
//         station_map_[a.station_name] = a;
//         RCLCPP_INFO(get_logger(),
//           "  Station %-15s  wait=%s  input=%s",
//           a.station_name.c_str(),
//           a.wait_confirm ? "YES" : "NO",
//           a.input_i.empty() ? "none" : a.input_i.c_str());
//       }
//     }

//     RCLCPP_INFO(get_logger(), "Loaded %zu stations", station_map_.size());
//     return true;

//   } catch (const std::exception & e) {
//     RCLCPP_ERROR(get_logger(), "YAML error: %s", e.what());
//     return false;
//   }
// }

// // ─── Helpers ──────────────────────────────────────────────────────
// void PlcS7BridgeNode::publishStatus(const std::string & text)
// {
//   std_msgs::msg::String m; m.data = text; status_pub_->publish(m);
// }

// std::string PlcS7BridgeNode::errorText(int code)
// {
//   char buf[256] = {0};
//   Cli_ErrorText(code, buf, sizeof(buf));
//   return std::string(buf);
// }

// }  // namespace plc_s7_bridge

// // ─── main ─────────────────────────────────────────────────────────
// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<plc_s7_bridge::PlcS7BridgeNode>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }
