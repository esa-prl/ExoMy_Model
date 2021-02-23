#include "rclcpp/rclcpp.hpp"
#include <exomy_msgs/msg/motor_commands.hpp>
#include <exomy_sim_msgs/msg/joint_command.hpp>
#include <exomy_sim_msgs/msg/joint_command_array.hpp>

class JointCommandNode : public rclcpp::Node
{
    public:
        JointCommandNode()
        : Node("joint_command_node")
        {
            motor_commands_sub_ = this->create_subscription<exomy_msgs::msg::MotorCommands>(
                "motor_commands",
                10,
                std::bind(&JointCommandNode::motor_commands_callback, this, std::placeholders::_1));
            
            joint_command_pub_ = this->create_publisher<exomy_sim_msgs::msg::JointCommandArray>(
                "joint_cmds",
                10
            );
        }
    
    private:
        void motor_commands_callback(const exomy_msgs::msg::MotorCommands::SharedPtr msg) const
        {
            // Create JointCommandArray Msg
            exomy_sim_msgs::msg::JointCommandArray joint_command_array_msg;

            rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

            int i = 0;
            for(std::string name : drive_joint_names)
            {
                exomy_sim_msgs::msg::JointCommand joint_command_msg;
                joint_command_msg.header.stamp = clock->now();
                joint_command_msg.name = name;
                joint_command_msg.mode = "VELOCITY";
                joint_command_msg.value = float(msg->motor_speeds[i])/20.0;
                joint_command_array_msg.joint_command_array.push_back(joint_command_msg);
                i++;
            }

            int j = 0;
            for(std::string name : steer_joint_names)
            {
                exomy_sim_msgs::msg::JointCommand joint_command_msg;
                joint_command_msg.header.stamp = clock->now();
                joint_command_msg.name = name;
                joint_command_msg.mode = "POSITION";
                joint_command_msg.value = -msg->motor_angles[j]/180.0*M_PI;
                joint_command_array_msg.joint_command_array.push_back(joint_command_msg);
                j++;
            }
            joint_command_array_msg.header.stamp = clock->now();
            joint_command_pub_->publish(joint_command_array_msg);

        }

        const std::list<std::string> drive_joint_names = {"DRV_LF_joint", "DRV_RF_joint", "DRV_LM_joint", "DRV_RM_joint", "DRV_LR_joint", "DRV_RR_joint"};
        const std::list<std::string> steer_joint_names = {"STR_LF_joint", "STR_RF_joint", "STR_LM_joint", "STR_RM_joint", "STR_LR_joint", "STR_RR_joint"};

        rclcpp::Subscription<exomy_msgs::msg::MotorCommands>::SharedPtr  motor_commands_sub_;
        rclcpp::Publisher<exomy_sim_msgs::msg::JointCommandArray>::SharedPtr joint_command_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointCommandNode>());
  rclcpp::shutdown();
  return 0;
}