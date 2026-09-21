// ═══════════════════════════════════════════════════════════════════
//  tcp_mission_bridge.cpp
//
//  TCP server ↔ ROS2 bridge for external mission control.
//
//  Usage (robot side — launched automatically):
//    ros2 run amr_pure_navigation tcp_mission_bridge
//
//  Usage (client side — e.g. Python on master PC):
//    import socket
//    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
//    s.connect(("172.20.10.2", 5000))
//    s.sendall(b"tag1 to tag3\n")
//    while True:
//        data = s.recv(4096)
//        if not data: break
//        print(data.decode())
//
//  Protocol (newline-delimited text):
//    Client → Robot:
//      "tag1 to tag3"       → queue range mission
//      "tag1,tag5,home"     → queue specific missions
//      "tag1"               → single mission
//      "list"               → get available missions
//      "status"             → get queue status
//      "cancel"             → cancel current mission
//      "stop"               → stop robot (0 cycles)
//
//    Robot → Client:
//      "OK: Mission queued: tag1 to tag3"
//      "STARTED: tag1"
//      "RUNNING: tag2"
//      "REACHED: tag3"
//      "COMPLETE: all 3 missions done"
//      "ERROR: Invalid mission 'xyz'. Available: tag1,tag2,..."
// ═══════════════════════════════════════════════════════════════════

#include "amr_pure_navigation/tcp_mission_bridge.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <sstream>

// ─── Constructor ──────────────────────────────────────────────────
TcpMissionBridge::TcpMissionBridge()
: Node("tcp_mission_bridge")
{
  // ── Parameters ─────────────────────────────────────────────────
  declare_parameter("tcp_port", 5000);
  tcp_port_ = get_parameter("tcp_port").as_int();

  // ── Publishers ─────────────────────────────────────────────────
  queue_missions_pub_ = create_publisher<std_msgs::msg::String>("/queue_missions", 10);
  load_mission_pub_   = create_publisher<std_msgs::msg::String>("/load_mission",   10);

  // ── Subscribers ────────────────────────────────────────────────
  nav_status_sub_ = create_subscription<std_msgs::msg::String>(
    "/nav_status", 10,
    std::bind(&TcpMissionBridge::navStatusCallback, this, std::placeholders::_1));

  queue_status_sub_ = create_subscription<std_msgs::msg::String>(
    "/queue_status", 10,
    std::bind(&TcpMissionBridge::queueStatusCallback, this, std::placeholders::_1));

  mission_list_sub_ = create_subscription<std_msgs::msg::String>(
    "/mission_list", 10,
    std::bind(&TcpMissionBridge::missionListCallback, this, std::placeholders::_1));

  station_reached_sub_ = create_subscription<std_msgs::msg::String>(
    "/station_reached", 10,
    std::bind(&TcpMissionBridge::stationReachedCallback, this, std::placeholders::_1));

  // ── Start TCP server thread ────────────────────────────────────
  server_thread_ = std::thread(&TcpMissionBridge::serverThread, this);

  RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════");
  RCLCPP_INFO(get_logger(), "  TCP Mission Bridge");
  RCLCPP_INFO(get_logger(), "  Listening on port %d", tcp_port_);
  RCLCPP_INFO(get_logger(), "  → Client commands forwarded to /queue_missions");
  RCLCPP_INFO(get_logger(), "  ← Status from /nav_status, /queue_status");
  RCLCPP_INFO(get_logger(), "═══════════════════════════════════════════");
}

// ─── Destructor ───────────────────────────────────────────────────
TcpMissionBridge::~TcpMissionBridge()
{
  running_ = false;

  // Close all connected clients
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    for (int fd : client_fds_) {
      ::shutdown(fd, SHUT_RDWR);
      ::close(fd);
    }
    client_fds_.clear();
  }

  // Close server socket to unblock accept()
  if (server_fd_ >= 0) {
    ::shutdown(server_fd_, SHUT_RDWR);
    ::close(server_fd_);
    server_fd_ = -1;
  }

  if (server_thread_.joinable()) {
    server_thread_.join();
  }
}

// ─── TCP Server Thread ───────────────────────────────────────────
void TcpMissionBridge::serverThread()
{
  // Create socket
  server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd_ < 0) {
    RCLCPP_ERROR(get_logger(), "❌ Failed to create TCP socket: %s", strerror(errno));
    return;
  }

  // Allow immediate port reuse
  int opt = 1;
  ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  // Bind
  struct sockaddr_in addr{};
  addr.sin_family      = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port        = htons(static_cast<uint16_t>(tcp_port_));

  if (::bind(server_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(get_logger(), "❌ Failed to bind port %d: %s", tcp_port_, strerror(errno));
    ::close(server_fd_);
    server_fd_ = -1;
    return;
  }

  if (::listen(server_fd_, 10) < 0) {
    RCLCPP_ERROR(get_logger(), "❌ Failed to listen: %s", strerror(errno));
    ::close(server_fd_);
    server_fd_ = -1;
    return;
  }

  RCLCPP_INFO(get_logger(), "✅ TCP server listening on 0.0.0.0:%d (multi-client enabled)", tcp_port_);

  // Accept loop — accepts multiple clients simultaneously
  while (running_) {
    struct sockaddr_in client_addr{};
    socklen_t client_len = sizeof(client_addr);
    int new_fd = ::accept(server_fd_,
                          reinterpret_cast<struct sockaddr*>(&client_addr),
                          &client_len);

    if (new_fd < 0) {
      if (!running_) break;  // shutting down
      RCLCPP_WARN(get_logger(), "accept() failed: %s", strerror(errno));
      continue;
    }

    char ip_str[INET_ADDRSTRLEN];
    ::inet_ntop(AF_INET, &client_addr.sin_addr, ip_str, sizeof(ip_str));
    RCLCPP_INFO(get_logger(), "✅ Client connected: %s:%d",
      ip_str, ntohs(client_addr.sin_port));

    // Store client fd
    {
      std::lock_guard<std::mutex> lock(clients_mutex_);
      client_fds_.insert(new_fd);
    }

    // Welcome messages disabled for clean protocol output

    // Spawn thread for this client so accept loop continues
    std::thread([this, new_fd, ip_str_str = std::string(ip_str)]() {
      clientHandler(new_fd);

      // Client disconnected
      {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        client_fds_.erase(new_fd);
      }
      ::close(new_fd);
      RCLCPP_INFO(get_logger(), "🔌 Client %s disconnected", ip_str_str.c_str());
    }).detach();
  }
}

// ─── Client Handler ──────────────────────────────────────────────
void TcpMissionBridge::clientHandler(int fd)
{
  char buffer[4096];
  std::string partial;  // for incomplete lines

  while (running_) {
    ssize_t n = ::recv(fd, buffer, sizeof(buffer) - 1, 0);

    if (n <= 0) {
      // Client disconnected or error
      break;
    }

    buffer[n] = '\0';
    partial += buffer;

    auto processCommand = [this, fd](const std::string & raw_line) {
      std::string line = raw_line;
      size_t start = line.find_first_not_of(" \t\r\n");
      size_t end   = line.find_last_not_of(" \t\r\n");
      if (start == std::string::npos) return;
      line = line.substr(start, end - start + 1);
      if (line.empty()) return;

      RCLCPP_INFO(get_logger(), " TCP received (fd=%d): '%s'", fd, line.c_str());

      std::string lower = line;
      std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

      if (lower == "list") {
        std_msgs::msg::String msg;
        msg.data = "list";
        load_mission_pub_->publish(msg);

        std::lock_guard<std::mutex> lock(list_mutex_);
        if (!cached_mission_list_.empty()) {
          sendToFD(fd, cached_mission_list_);
        }
      } else if (lower == "status") {
        std_msgs::msg::String msg;
        msg.data = "status";
        queue_missions_pub_->publish(msg);

        if (has_reached_sent_) {
          std::string target;
          {
            std::lock_guard<std::mutex> lock(target_tag_mutex_);
            target = last_target_tag_;
          }
          sendToFD(fd, "reach destinastion " + target);
        } else if (is_running_sent_) {
          sendToFD(fd, "running");
        }
      } else if (lower == "cancel" || lower == "clear" || lower == "stop") {
        is_running_sent_  = false;
        has_reached_sent_ = false;
        std_msgs::msg::String msg;
        msg.data = "cancel";
        queue_missions_pub_->publish(msg);
        sendToClient("stopped");
      } else {
        // Mission command (e.g., "tag1# to tag3#", "tag1#")
        std::string target_tag = line;
        size_t to_pos = lower.find(" to ");
        if (to_pos != std::string::npos) {
          target_tag = line.substr(to_pos + 4);
        } else if (line.find(',') != std::string::npos) {
          size_t comma_pos = line.rfind(',');
          target_tag = line.substr(comma_pos + 1);
        }

        size_t s = target_tag.find_first_not_of(" \t\r\n");
        size_t e = target_tag.find_last_not_of(" \t\r\n");
        if (s != std::string::npos && e != std::string::npos) {
          target_tag = target_tag.substr(s, e - s + 1);
        }

        {
          std::lock_guard<std::mutex> lock(target_tag_mutex_);
          last_target_tag_ = target_tag;
        }

        has_reached_sent_ = false;

        std_msgs::msg::String msg;
        msg.data = line;
        queue_missions_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "🚀 Published to /queue_missions: '%s' (destination: '%s')",
          line.c_str(), target_tag.c_str());

        // Immediately send running status upon receiving command
        sendToClient("running");
        is_running_sent_ = true;
      }
    };

    // Process complete lines (newline-delimited)
    std::string::size_type pos;
    while ((pos = partial.find('\n')) != std::string::npos) {
      std::string line = partial.substr(0, pos);
      partial = partial.substr(pos + 1);
      processCommand(line);
    }

    if (!partial.empty() && partial.find('\n') == std::string::npos) {
      processCommand(partial);
      partial.clear();
    }
  }
}

// ─── Send to Single Client (fd) ──────────────────────────────────
bool TcpMissionBridge::sendToFD(int fd, const std::string & msg)
{
  if (fd < 0) return false;

  std::string formatted_msg = msg;
  while (!formatted_msg.empty() && (formatted_msg.back() == '\n' || formatted_msg.back() == '\r')) {
    formatted_msg.pop_back();
  }

  ssize_t sent = ::send(fd, formatted_msg.c_str(), formatted_msg.size(), MSG_NOSIGNAL);
  if (sent < 0) {
    RCLCPP_WARN(get_logger(), "Failed to send to client fd=%d: %s", fd, strerror(errno));
    return false;
  }
  return true;
}

// ─── Send to All Connected Clients (broadcast) ───────────────────
bool TcpMissionBridge::sendToClient(const std::string & msg)
{
  std::lock_guard<std::mutex> lock(clients_mutex_);
  if (client_fds_.empty()) return false;

  std::string formatted_msg = msg;
  while (!formatted_msg.empty() && (formatted_msg.back() == '\n' || formatted_msg.back() == '\r')) {
    formatted_msg.pop_back();
  }

  for (int fd : client_fds_) {
    ::send(fd, formatted_msg.c_str(), formatted_msg.size(), MSG_NOSIGNAL);
  }
  return true;
}

// ─── ROS2 Callbacks → Forward to TCP Client ──────────────────────

void TcpMissionBridge::navStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string & data = msg->data;

  if (data.find("RUNNING:") != std::string::npos || data.find("STARTED:") != std::string::npos) {
    if (!is_running_sent_) {
      sendToClient("running");
      is_running_sent_ = true;
    }
  } else if (data.find("QUEUE_COMPLETE:") != std::string::npos) {
    if (!has_reached_sent_) {
      std::string target;
      {
        std::lock_guard<std::mutex> lock(target_tag_mutex_);
        target = last_target_tag_;
      }
      if (target.empty()) target = "destination";
      sendToClient("reach destinastion " + target);
      has_reached_sent_ = true;
      is_running_sent_  = false;
    }
  }
}

void TcpMissionBridge::queueStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string & data = msg->data;

  if (data.find("QUEUE_STARTED:") != std::string::npos || data.find("RUNNING:") != std::string::npos) {
    if (!is_running_sent_) {
      sendToClient("running");
      is_running_sent_ = true;
    }
  } else if (data.find("QUEUE_COMPLETE:") != std::string::npos) {
    if (!has_reached_sent_) {
      std::string target;
      {
        std::lock_guard<std::mutex> lock(target_tag_mutex_);
        target = last_target_tag_;
      }
      if (target.empty()) target = "destination";
      sendToClient("reach destinastion " + target);
      has_reached_sent_ = true;
      is_running_sent_  = false;
    }
  }
}

void TcpMissionBridge::missionListCallback(const std_msgs::msg::String::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(list_mutex_);
    cached_mission_list_ = msg->data;
  }
  sendToClient(msg->data);
}

void TcpMissionBridge::stationReachedCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string tag = msg->data;
  size_t colon_pos = tag.find(':');
  if (colon_pos != std::string::npos) {
    tag = tag.substr(0, colon_pos);
  }

  if (tag.rfind("station_", 0) == 0) {
    std::lock_guard<std::mutex> lock(target_tag_mutex_);
    if (!last_target_tag_.empty()) {
      tag = last_target_tag_;
    }
  }

  if (!has_reached_sent_) {
    sendToClient("reach destinastion " + tag);
    has_reached_sent_ = true;
    is_running_sent_  = false;
  }
}


// ─── main ─────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TcpMissionBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
