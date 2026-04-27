#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class BinosenseControlExample : public rclcpp::Node
{
public:
  BinosenseControlExample()
  : Node("binosense_control_example"),
    mode_(PositionMode),
    current_step_(0)
  {
    this->declare_parameter<std::string>("mode", "position");

    std::string mode_str;
    this->get_parameter("mode", mode_str);

    if (mode_str == "twist") {
      mode_ = TwistMode;
    } else if (mode_str == "sweep") {
      mode_ = SweepMode;
    } else {
      mode_ = PositionMode;
    }

    position_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "motor_command", 10);

    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", 10);

    motor_state_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "motor_state", 10,
      std::bind(&BinosenseControlExample::motorStateCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      100ms,
      std::bind(&BinosenseControlExample::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Binosense Control Example Started");
    printHelp();
  }

private:
  enum ControlMode
  {
    PositionMode,
    TwistMode,
    SweepMode
  };

  void printHelp()
  {
    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "=== Binosense Control Example ===");
    RCLCPP_INFO(this->get_logger(), "Control modes:");
    RCLCPP_INFO(this->get_logger(), "  - position: Send position commands (default)");
    RCLCPP_INFO(this->get_logger(), "  - twist: Send twist commands");
    RCLCPP_INFO(this->get_logger(), "  - sweep: Auto sweep motion");
    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "Usage: ros2 run binosense_ros2 binosense_control_example --ros-args -p mode:=<mode>");
    RCLCPP_INFO(this->get_logger(), " ");
  }

  void motorStateCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() >= 6) {
      for (int i = 0; i < 6; i++) {
        current_positions_[i] = msg->data[i];
      }
    }
  }

  void timerCallback()
  {
    switch (mode_) {
      case PositionMode:
        sendPositionCommands();
        break;
      case TwistMode:
        sendTwistCommands();
        break;
      case SweepMode:
        sendSweepMotion();
        break;
    }
  }

  void sendPositionCommands()
  {
    static bool first = true;
    if (first) {
      RCLCPP_INFO(this->get_logger(), "Position Mode: Sending home position");
      first = false;
    }

    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data.resize(6, 0.0f);
    position_pub_->publish(msg);
  }

  void sendTwistCommands()
  {
    static bool first = true;
    if (first) {
      RCLCPP_INFO(this->get_logger(), "Twist Mode: Use keyboard to control (not implemented in this example)");
      first = false;
    }
  }

  void sendSweepMotion()
  {
    static bool first = true;
    if (first) {
      RCLCPP_INFO(this->get_logger(), "Sweep Mode: Performing auto sweep");
      first = false;
    }

    current_step_++;
    float t = current_step_ * 0.02f;

    float yaw = 15.0f * std::sin(t);
    float pitch = 10.0f * std::sin(t * 0.5f);

    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data.resize(6);
    msg.data[0] = pitch;
    msg.data[1] = 0.0f;
    msg.data[2] = yaw;
    msg.data[3] = pitch;
    msg.data[4] = 0.0f;
    msg.data[5] = yaw;

    position_pub_->publish(msg);
  }

  ControlMode mode_;
  int current_step_;
  float current_positions_[6] = {0};

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr position_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr motor_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BinosenseControlExample>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
