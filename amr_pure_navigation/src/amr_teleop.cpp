// ================================================================
//  amr_teleop.cpp  —  Custom C++ Teleop with Speed Ramping
//
//  CONTROLS:
//    W = Forward  (hold to gradually increase speed)
//    X = Backward (hold to gradually increase reverse speed)
//    S = STOP     (immediate full stop)
//    A = Turn Right
//    D = Turn Left
//    Q = Quit
//
//  SPEED BEHAVIOR:
//    Hold W → speed ramps up:  0.1 → 0.2 → 0.3 → ... → max
//    Press S → immediate stop: 0.0
//    Hold X → reverse ramps:  -0.1 → -0.2 → ... → -max
//    Release key → speed holds (does not drop until S or reverse)
//
//  TOPICS:
//    Publishes: /diff_drive_controller/cmd_vel
// ================================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/select.h>

// ── Terminal colors ───────────────────────────────────────────────
#define RED     "\033[91m"
#define GREEN   "\033[92m"
#define YELLOW  "\033[93m"
#define CYAN    "\033[96m"
#define BOLD    "\033[1m"
#define RESET   "\033[0m"
#define CLEAR   "\033[2J\033[H"

// ── Speed config ──────────────────────────────────────────────────
static const double LINEAR_STEP   = 0.02;   // m/s added per ramp tick
static const double ANGULAR_STEP  = 0.1;    // rad/s added per ramp tick
static const double LINEAR_MAX    = 0.8;    // m/s max forward/backward
static const double ANGULAR_MAX   = 1.5;    // rad/s max turn
static const double RAMP_HZ       = 20.0;   // ramp update rate

// ── Get single keypress (non-blocking) ───────────────────────────
char get_key()
{
  struct timeval tv;
  tv.tv_sec  = 0;
  tv.tv_usec = 50000;  // 50ms timeout

  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);

  int ret = select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
  if (ret > 0) {
    char c;
    read(STDIN_FILENO, &c, 1);
    return c;
  }
  return 0;
}

// ── Speed bar visual ──────────────────────────────────────────────
std::string speedBar(double current, double max_val, int width = 20)
{
  double ratio = std::abs(current) / max_val;
  int filled   = static_cast<int>(ratio * width);
  filled       = std::max(0, std::min(filled, width));
  std::string bar = "";
  for (int i = 0; i < filled; i++)  bar += "█";
  for (int i = filled; i < width; i++) bar += "░";
  return bar;
}

// ── Print live status ─────────────────────────────────────────────
void printStatus(double linear, double angular, char last_key)
{
  std::string direction;
  if      (linear  >  0.01 && std::abs(angular) < 0.01) direction = GREEN  "▲  FORWARD"  RESET;
  else if (linear  < -0.01 && std::abs(angular) < 0.01) direction = RED    "▼  BACKWARD" RESET;
  else if (angular >  0.01 && std::abs(linear)  < 0.01) direction = CYAN   "◄  TURN LEFT"  RESET;
  else if (angular < -0.01 && std::abs(linear)  < 0.01) direction = CYAN   "►  TURN RIGHT" RESET;
  else if (linear  >  0.01 && angular >  0.01)           direction = CYAN   "↖  FWD LEFT"   RESET;
  else if (linear  >  0.01 && angular < -0.01)           direction = CYAN   "↗  FWD RIGHT"  RESET;
  else                                                    direction = YELLOW "■  STOPPED"    RESET;

  char key_display[8] = "none";
  if (last_key != 0) {
    key_display[0] = last_key;
    key_display[1] = '\0';
  }

  printf(CLEAR);
  printf(BOLD "================================================\n" RESET);
  printf(BOLD "   AMR TELEOP  —  C++ Speed Ramping\n" RESET);
  printf(BOLD "================================================\n" RESET);
  printf("\n");
  printf("  Direction  :  %s\n", direction.c_str());
  printf("  Last key   :  [%s]\n", key_display);
  printf("\n");
  printf("  " BOLD "Linear  speed" RESET " : %+.3f m/s\n", linear);
  printf("  [" CYAN "%s" RESET "]  max=%.1f\n", speedBar(linear, LINEAR_MAX).c_str(), LINEAR_MAX);
  printf("\n");
  printf("  " BOLD "Angular speed" RESET " : %+.3f rad/s\n", angular);
  printf("  [" CYAN "%s" RESET "]  max=%.1f\n", speedBar(angular, ANGULAR_MAX).c_str(), ANGULAR_MAX);
  printf("\n");
  printf(BOLD "------------------------------------------------\n" RESET);
  printf("  W = Forward (ramp up)     S = STOP\n");
  printf("  X = Backward (ramp up)    \n");
  printf("  A = Turn Right            D = Turn Left\n");
  printf("  Q = Quit\n");
  printf(BOLD "================================================\n" RESET);
  fflush(stdout);
}

// ── Main ──────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("amr_teleop");

  // Parameter: topic name
  node->declare_parameter<std::string>("cmd_vel_topic", "/diff_drive_controller/cmd_vel");
  std::string topic = node->get_parameter("cmd_vel_topic").as_string();

  auto pub = node->create_publisher<geometry_msgs::msg::Twist>(topic, 10);

  RCLCPP_INFO(node->get_logger(), "AMR Teleop started on: %s", topic.c_str());

  // ── Save and set raw terminal ────────────────────────────────
  struct termios old_settings, new_settings;
  tcgetattr(STDIN_FILENO, &old_settings);
  new_settings = old_settings;
  new_settings.c_lflag &= ~(ICANON | ECHO);  // raw mode, no echo
  new_settings.c_cc[VMIN]  = 0;
  new_settings.c_cc[VTIME] = 0;
  tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);

  // ── State ────────────────────────────────────────────────────
  double linear  = 0.0;
  double angular = 0.0;
  char   last_key = 0;
  bool   running  = true;

  auto rate = rclcpp::WallRate(RAMP_HZ);

  printStatus(linear, angular, last_key);

  while (rclcpp::ok() && running)
  {
    char key = get_key();

    if (key != 0) {
      last_key = key;

      switch (key)
      {
        // ── W: Forward ramp up ────────────────────────────────
        case 'w':
        case 'W':
          // If going backward — first ramp down to 0
          if (linear < 0.0) {
            linear = std::min(linear + LINEAR_STEP * 2, 0.0);
          } else {
            linear = std::min(linear + LINEAR_STEP, LINEAR_MAX);
          }
          angular = 0.0;
          break;

        // ── X: Backward ramp up ───────────────────────────────
        case 'x':
        case 'X':
          // If going forward — first ramp down to 0
          if (linear > 0.0) {
            linear = std::max(linear - LINEAR_STEP * 2, 0.0);
          } else {
            linear = std::max(linear - LINEAR_STEP, -LINEAR_MAX);
          }
          angular = 0.0;
          break;

        // ── S: Immediate STOP ─────────────────────────────────
        case 's':
        case 'S':
          linear  = 0.0;
          angular = 0.0;
          break;

        // ── A: Turn Right ─────────────────────────────────────
        case 'a':
        case 'A':
          angular = std::min(angular + ANGULAR_STEP, ANGULAR_MAX);
          break;

        // ── D: Turn Left ──────────────────────────────────────
        case 'd':
        case 'D':
          angular = std::max(angular - ANGULAR_STEP, -ANGULAR_MAX);
          break;

        // ── Q / CTRL+C: Quit ──────────────────────────────────
        case 'q':
        case 'Q':
        case '\x03':
          running = false;
          break;

        default:
          break;
      }
    }

    // ── Publish cmd_vel ───────────────────────────────────────
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = linear;
    cmd.angular.z = angular;
    pub->publish(cmd);

    // ── Refresh display ───────────────────────────────────────
    printStatus(linear, angular, last_key);

    rclcpp::spin_some(node);
    rate.sleep();
  }

  // ── Cleanup ──────────────────────────────────────────────────
  // Stop robot
  geometry_msgs::msg::Twist stop_cmd;
  pub->publish(stop_cmd);

  // Restore terminal
  tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);

  printf("\n" YELLOW "Teleop stopped. Robot halted.\n" RESET "\n");

  rclcpp::shutdown();
  return 0;
}