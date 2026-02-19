#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"

class PtJointCommandBridge : public rclcpp::Node
{
public:
  PtJointCommandBridge()
  : Node("pt_joint_command_bridge")
  {
    arm_joint1_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/model/x3plus_bot/joint/arm_joint1/cmd_pos", 10);
    arm_joint2_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/model/x3plus_bot/joint/arm_joint2/cmd_pos", 10);

    pt_yaw_sub_ = create_subscription<std_msgs::msg::Float32>(
      "pt_yaw_angle", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg)
      {
        std_msgs::msg::Float64 out;
        out.data = static_cast<double>(msg->data);
        arm_joint1_pub_->publish(out);
      });

    pt_pitch_sub_ = create_subscription<std_msgs::msg::Float32>(
      "pt_pitch_angle", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg)
      {
        std_msgs::msg::Float64 out;
        out.data = static_cast<double>(msg->data);
        arm_joint2_pub_->publish(out);
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr arm_joint1_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr arm_joint2_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pt_yaw_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pt_pitch_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PtJointCommandBridge>());
  rclcpp::shutdown();
  return 0;
}
