#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cmath>
#include <string>
#include <chrono>
#include <memory>

#include "fprotocol.h"
#include "RobotProto.h"

/* ============================================================
 *  Serial (C layer)
 * ============================================================ */
static int g_serial_fd = -1;

static int32_t serial_read_cb(int16_t, uint8_t *buf, int32_t size)
{
  if (g_serial_fd < 0) return 0;
  ssize_t r = ::read(g_serial_fd, buf, size);
  return (r > 0) ? static_cast<int32_t>(r) : 0;
}

static int32_t serial_write_cb(int16_t, uint8_t *buf, int32_t size)
{
  if (g_serial_fd < 0) return -1;
  ssize_t w = ::write(g_serial_fd, buf, size);
  return (w > 0) ? static_cast<int32_t>(w) : -1;
}

/* ============================================================
 *  ProtocolBridge
 * ============================================================ */
class ProtocolBridge
{
public:
  explicit ProtocolBridge(rclcpp::Node *node)
  : node_(node)
  {
    instance_ = this;

    handler_ = fprotocol_init(serial_read_cb, serial_write_cb);
    fprotocol_add_other_node(handler_, 0x0001, robot_index_info);

    RCLCPP_INFO(node_->get_logger(), "ProtocolBridge initialized");
  }

  ~ProtocolBridge()
  {
    if (handler_)
      fprotocol_delete(handler_);
    instance_ = nullptr;
  }

  void tick()
  {
    if (handler_)
      fprotocol_tick(handler_);
  }

  /* -------- outbound -------- */
  void send_led(bool on)
  {
    led = on ? 1 : 0;
    ::write_led(handler_, 0x0001, 0);
  }

  /* -------- inbound callbacks -------- */
  static int16_t on_led(uint16_t, uint32_t, uint16_t)
  {
    if (instance_ && instance_->node_)
    {
      RCLCPP_INFO(instance_->node_->get_logger(),
                  "[PROTO] LED callback from MCU");
    }
    return 0;
  }

  static int16_t on_cmd_vel(uint16_t, uint32_t, uint16_t)
  {
    if (instance_ && instance_->node_)
    {
      RCLCPP_INFO(instance_->node_->get_logger(),
                  "[PROTO] CMD_VEL callback from MCU");
    }
    return 0;
  }

private:
  rclcpp::Node *node_;
  fprotocol_handler *handler_{nullptr};

  static ProtocolBridge *instance_;
};

ProtocolBridge *ProtocolBridge::instance_ = nullptr;

/* ============================================================
 *  RobotProto C callbacks (must be global)
 * ============================================================ */
int16_t callback_led(uint16_t t, uint32_t f, uint16_t e)
{
  (void)t; (void)f; (void)e;
  return ProtocolBridge::on_led(t, f, e);
}

int16_t callback_cmd_vel(uint16_t t, uint32_t f, uint16_t e)
{
  (void)t; (void)f; (void)e;
  return ProtocolBridge::on_cmd_vel(t, f, e);
}

/* ============================================================
 *  ServerNode
 * ============================================================ */
class ServerNode : public rclcpp::Node
{
public:
  ServerNode()
  : Node("server_ros2_cpp_node"),
    led_interval_(std::chrono::milliseconds(250))
  {
    declare_parameter("serial_port", "/dev/ttyACM0");
    declare_parameter("baud_rate", 115200);

    get_parameter("serial_port", serial_port_);
    get_parameter("baud_rate", baud_rate_);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    g_serial_fd = open_serial(serial_port_, baud_rate_);
    if (g_serial_fd < 0)
    {
      RCLCPP_FATAL(get_logger(), "Failed to open serial %s",
                   serial_port_.c_str());
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(get_logger(), "Serial %s opened @ %d",
                serial_port_.c_str(), baud_rate_);

    bridge_ = std::make_unique<ProtocolBridge>(this);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&ServerNode::on_timer, this)
    );
  }

  ~ServerNode()
  {
    if (g_serial_fd >= 0)
      ::close(g_serial_fd);
  }

private:
  int open_serial(const std::string &port, int baud)
  {
    int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;

    termios tty{};
    tcgetattr(fd, &tty);
    cfmakeraw(&tty);

    speed_t speed = B115200;
    if (baud == 9600) speed = B9600;
    else if (baud == 57600) speed = B57600;
    else if (baud == 115200) speed = B115200;

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(fd, TCSANOW, &tty);

    return fd;
  }

  void publish_odom()
  {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_footprint";
    msg.pose.pose.position.x = odom.x;
    msg.pose.pose.position.y = odom.y;
    msg.pose.pose.position.z = odom.z;
    odom_pub_->publish(msg);
  }

  void on_timer()
  {
    bridge_->tick();
    publish_odom();

    auto t = now();
    if ((t - last_led_time_) >= rclcpp::Duration(led_interval_))
    {
      led_state_ = !led_state_;
      bridge_->send_led(led_state_);
      last_led_time_ = t;
    }
  }

private:
  std::string serial_port_;
  int baud_rate_{115200};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<ProtocolBridge> bridge_;

  bool led_state_{false};
  rclcpp::Time last_led_time_{0, 0, RCL_ROS_TIME};
  std::chrono::milliseconds led_interval_;
};

/* ============================================================
 *  main
 * ============================================================ */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServerNode>());
  rclcpp::shutdown();
  return 0;
}
