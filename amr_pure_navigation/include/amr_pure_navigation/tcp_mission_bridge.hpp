// ═══════════════════════════════════════════════════════════════════
//  tcp_mission_bridge.hpp
//
//  TCP ↔ ROS2 bridge for external mission control.
//
//  ┌──────────┐   TCP:5000   ┌───────────────────┐   ROS2 topics
//  │  Client  │ ──────────►  │ TcpMissionBridge   │ ──► /queue_missions
//  │ (Master) │ ◄──────────  │   (this node)      │ ◄── /nav_status
//  └──────────┘  responses   └───────────────────┘     /queue_status
//                                                      /mission_list
//
//  Client sends:   "tag1 to tag3\n"
//  Bridge:         publishes on /queue_missions
//  Bridge:         relays /nav_status + /queue_status back to client
//
//  Two-way communication:
//    Client → Robot:  mission commands
//    Robot → Client:  mission started / running / reached / errors
// ═══════════════════════════════════════════════════════════════════
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <mutex>
#include <set>
#include <string>
#include <thread>

class TcpMissionBridge : public rclcpp::Node
{
public:
  TcpMissionBridge();
  ~TcpMissionBridge() override;

private:
  // ── TCP server ─────────────────────────────────────────────────
  void serverThread();           // main accept loop
  void clientHandler(int fd);    // per-client read/write loop
  bool sendToClient(const std::string & msg);  // broadcast to all clients
  bool sendToFD(int fd, const std::string & msg); // send to specific client

  int               server_fd_{-1};
  std::set<int>     client_fds_;
  std::mutex        clients_mutex_;
  std::thread       server_thread_;
  std::atomic<bool> running_{true};

  int         tcp_port_{5000};

  // ── ROS2 pub/sub ───────────────────────────────────────────────
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   queue_missions_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   load_mission_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr queue_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_list_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr station_reached_sub_;

  // ── Callbacks ──────────────────────────────────────────────────
  void navStatusCallback(const std_msgs::msg::String::SharedPtr msg);
  void queueStatusCallback(const std_msgs::msg::String::SharedPtr msg);
  void missionListCallback(const std_msgs::msg::String::SharedPtr msg);
  void stationReachedCallback(const std_msgs::msg::String::SharedPtr msg);

  // ── Mission State Tracking ─────────────────────────────────────
  std::string       last_target_tag_;
  std::mutex        target_tag_mutex_;
  std::atomic<bool> is_running_sent_{false};
  std::atomic<bool> has_reached_sent_{false};

  // ── Cached mission list for validation ─────────────────────────
  std::string cached_mission_list_;
  std::mutex  list_mutex_;
};

