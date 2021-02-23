#include <rover_gazebo_plugins/rover_gazebo_joint_plugin.hpp>

#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>

#include <exomy_sim_msgs/msg/joint_command.hpp>
#include <exomy_sim_msgs/msg/joint_command_array.hpp>

#include <ignition/math/Rand.hh>
#include <regex>

namespace gazebo_plugins
{
    class RoverGazeboJointPluginPrivate
    {
    public:
        /// Callback to be called at every simulation iteration.
        /// \param[in] _info Updated simulation info.
        void OnUpdate(const gazebo::common::UpdateInfo &_info);

        /// Callback when a joint command array is received
        /// \param[in] _msg Array of joint commands.
        void OnJointCmdArray(exomy_sim_msgs::msg::JointCommandArray::SharedPtr _msg);

        /// A pointer to the GazeboROS node.
        gazebo_ros::Node::SharedPtr ros_node_;

        /// Subscriber to joint command arrays
        rclcpp::Subscription<exomy_sim_msgs::msg::JointCommandArray>::SharedPtr joint_cmd_array_sub_;

        /// Subscriber to joint commands
        rclcpp::Subscription<exomy_sim_msgs::msg::JointCommand>::SharedPtr joint_cmd_sub_;

        /// Connection to event called at every world iteration.
        gazebo::event::ConnectionPtr update_connection_;

        /// Pointer to model
        gazebo::physics::ModelPtr model_;

        /// Update period in seconds.
        double update_period_;

        /// Last update time.
        gazebo::common::Time last_update_time_;

        /// Protect variables accessed on callbacks.
        std::mutex lock_;

        /// Joint controller that is managing controllers of all model joints
        gazebo::physics::JointControllerPtr joint_controller_;

        /// Callback to be called at every simulation iteration.
        /// \param[in] _sdf Pointer to sdf parent containing children with PID parameters.
        bool LoadPIDParametersFromSDF(const sdf::ElementPtr _sdf);

        /// Position pid parameters read from sdf
        // std::map<std::string, ignition::math::Vector3d> position_pid_parameters;
        std::list<PIDParameter> position_pid_parameters;

        /// Velocity pid parameters read from sdf
        // std::map<std::string, ignition::math::Vector3d> velocity_pid_parameters;
        std::list<PIDParameter> velocity_pid_parameters;
    };

    RoverGazeboJointPlugin::RoverGazeboJointPlugin()
        : impl_(std::make_unique<RoverGazeboJointPluginPrivate>())
    {
    }

    RoverGazeboJointPlugin::~RoverGazeboJointPlugin()
    {
    }

    bool RoverGazeboJointPluginPrivate::LoadPIDParametersFromSDF(const sdf::ElementPtr _sdf)
    {
        GZ_ASSERT(_sdf, "_sdf element is null");

        // Get the position PID parameters
        sdf::ElementPtr position_pid_group = _sdf->GetElement("position_pids");
        sdf::ElementPtr position_pid = position_pid_group->GetFirstElement();

        while (position_pid)
        {
            PIDParameter pid_parameter;
            pid_parameter.identifier = position_pid->GetName();
            pid_parameter.pid_values = position_pid->Get<ignition::math::Vector3d>();
            if(position_pid->HasAttribute("regex"))
            {
                pid_parameter.regex = position_pid->GetAttribute("regex")->GetAsString(); 
            }

            this->position_pid_parameters.push_back(pid_parameter);

            position_pid = position_pid->GetNextElement();
        }

        // Get the velocity PID parameters
        sdf::ElementPtr velocity_pid_group = _sdf->GetElement("velocity_pids");
        sdf::ElementPtr velocity_pid = velocity_pid_group->GetFirstElement();

        while (velocity_pid)
        {
            PIDParameter pid_parameter;
            pid_parameter.identifier = velocity_pid->GetName();
            pid_parameter.pid_values = velocity_pid->Get<ignition::math::Vector3d>();
            if(velocity_pid->HasAttribute("regex"))
            {
                pid_parameter.regex = velocity_pid->GetAttribute("regex")->GetAsString(); 
            }

            this->velocity_pid_parameters.push_back(pid_parameter);

 
            velocity_pid = velocity_pid->GetNextElement();
        }

        return true;
    }

    void RoverGazeboJointPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // Get the model
        impl_->model_ = _model;

        // Initialize ROS node
        impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

        // Get the joint controller of the model
        impl_->joint_controller_ = impl_->model_->GetJointController();

        // Load all the parameters from the sdf
        impl_->LoadPIDParametersFromSDF(_sdf);

        // Set the PIDs of the joints accordingly to the given parameters
        for (auto const &position_pid_parameter_element : impl_->position_pid_parameters)
        {
            auto const pid_identifier = position_pid_parameter_element.identifier;
            auto const pid_values = position_pid_parameter_element.pid_values;

            std::regex pid_regex("(^|_|:)"+pid_identifier+"($|_)");
            if (!position_pid_parameter_element.regex.empty())
            {
                pid_regex = std::regex(position_pid_parameter_element.regex);
            }
            
            bool parameter_has_joint = false;

            RCLCPP_DEBUG(impl_->ros_node_->get_logger(), "Position PID parameter: %s, Values: %f %f %f", pid_identifier.c_str(), pid_values.X(), pid_values.Y(), pid_values.Z());

            for (auto const &joint_element : impl_->joint_controller_->GetJoints())
            {
                auto const joint_name = joint_element.first;
                auto const joint = joint_element.second;

                if(std::regex_search(joint_name, pid_regex))
                {
                    parameter_has_joint = true;
                    impl_->joint_controller_->SetPositionPID(joint_name, gazebo::common::PID(pid_values.X(), pid_values.Y(), pid_values.Z()));
                    impl_->joint_controller_->SetPositionTarget(joint_name, 0.0);
                    RCLCPP_DEBUG(impl_->ros_node_->get_logger(), "Set position PID on joint: %s to P: %f I: %f D: %f", joint_name.c_str(), pid_values.X(), pid_values.Y(), pid_values.Z());
                }
            }

            if(!parameter_has_joint)
            {
                RCLCPP_WARN(impl_->ros_node_->get_logger(), "No joint found for position pid identifier %s. Maybe there is a typo or your config is not up to date.", pid_identifier.c_str());
            }
        }

        for (auto const &velocity_pid_parameter_element : impl_->velocity_pid_parameters)
        {
            auto const pid_identifier = velocity_pid_parameter_element.identifier;
            auto const pid_values = velocity_pid_parameter_element.pid_values;

            auto regex = std::regex("(^|_|:)"+pid_identifier+"($|_)");
            if (!velocity_pid_parameter_element.regex.empty())
            {
                regex = std::regex(velocity_pid_parameter_element.regex);
            }

            bool parameter_has_joint = false;

            RCLCPP_DEBUG(impl_->ros_node_->get_logger(), "Velocity PID parameter: %s, Values: %f %f %f", pid_identifier.c_str(), pid_values.X(), pid_values.Y(), pid_values.Z());

            for (auto const &joint_element : impl_->joint_controller_->GetJoints())
            {
                auto const joint_name = joint_element.first;
                auto const joint = joint_element.second;

                if(std::regex_search(joint_name, regex))
                {
                    parameter_has_joint = true;
                    impl_->joint_controller_->SetVelocityPID(joint_name, gazebo::common::PID(pid_values.X(), pid_values.Y(), pid_values.Z()));
                    impl_->joint_controller_->SetVelocityTarget(joint_name, 0.0);
                    RCLCPP_DEBUG(impl_->ros_node_->get_logger(), "Set velocity PID on joint: %s to P: %f I: %f D: %f", joint_name.c_str(), pid_values.X(), pid_values.Y(), pid_values.Z());
                }
            }
            if(!parameter_has_joint)
            {
                RCLCPP_WARN(impl_->ros_node_->get_logger(), "No joint found for velocity pid identifier %s. Maybe there is a typo or your config is not up to date.", pid_identifier.c_str());
            }
        }

        // Update rate
        auto update_rate = _sdf->Get<double>("update_rate", 100.0).first;
        if (update_rate > 0.0)
        {
            impl_->update_period_ = 1.0 / update_rate;
        }
        else
        {
            impl_->update_period_ = 0.0;
        }
        impl_->last_update_time_ = _model->GetWorld()->SimTime();

        // Subscribe to joint command array topic
        impl_->joint_cmd_array_sub_ = impl_->ros_node_->create_subscription<exomy_sim_msgs::msg::JointCommandArray>(
            "joint_cmds", rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&RoverGazeboJointPluginPrivate::OnJointCmdArray, impl_.get(), std::placeholders::_1));

        // Listen to the update event (broadcast every simulation iteration)
        impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&RoverGazeboJointPluginPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
    }

    void RoverGazeboJointPlugin::Reset()
    {
        impl_->last_update_time_ = impl_->model_->GetWorld()->SimTime();
        impl_->joint_controller_->Reset();
    }

    void RoverGazeboJointPluginPrivate::OnUpdate(const gazebo::common::UpdateInfo &_info)
    {
        std::lock_guard<std::mutex> lock(lock_);

        // Check for update constraints
        double seconds_since_last_update = (_info.simTime - last_update_time_).Double();
        if (seconds_since_last_update < update_period_)
        {
            return;
        }

        // Update all joint controllers
        joint_controller_->Update();
    }

    void RoverGazeboJointPluginPrivate::OnJointCmdArray(exomy_sim_msgs::msg::JointCommandArray::SharedPtr _msg)
    {
        std::lock_guard<std::mutex> scoped_lock(lock_);

        // Iterate over received joint command array and set controller targets
        for (auto const cmd : _msg->joint_command_array)
        {
            if (cmd.mode == "POSITION")
            {
                if (!joint_controller_->SetPositionTarget(model_->GetJoint(cmd.name)->GetScopedName(), cmd.value))
                {
                    RCLCPP_WARN(ros_node_->get_logger(), "Joint %s from received command was not found in model.", cmd.name.c_str());
                }
            }
            else if (cmd.mode == "VELOCITY")
            {
                // if (!joint_controller_->SetVelocityTarget(model_->GetJoint(cmd.name)->GetScopedName(), cmd.value))
                // {
                //     RCLCPP_WARN(ros_node_->get_logger(), "Joint %s from received command was not found in model.", cmd.name.c_str());
                // }
                // This prohibits the wheels from backspinning
                model_->GetJoint(cmd.name)->SetParam("fmax", 0, 10.0);
                model_->GetJoint(cmd.name)->SetParam("vel", 0, double(cmd.value));
            }
            else
            {
                RCLCPP_WARN(ros_node_->get_logger(), "Undefined mode in joint command received: %s", cmd.mode.c_str());
            }
        }
    }

    GZ_REGISTER_MODEL_PLUGIN(RoverGazeboJointPlugin)
} // namespace gazebo_plugins
