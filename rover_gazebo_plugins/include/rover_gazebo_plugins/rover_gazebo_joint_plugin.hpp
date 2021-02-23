#ifndef GAZEBO_PLUGINS_ROVER_GAZEBO_JOINT_PLUGIN_HPP_
#define GAZEBO_PLUGINS_ROVER_GAZEBO_JOINT_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>

namespace gazebo_plugins
{
class RoverGazeboJointPluginPrivate;
/// Set the PID parameters of joints
///
/// The plugin iterates over all joints
/// and sets the PID if the joint name and the PID identifier meet the regex:
/// "(^|_|:)pid_identifier($|_)"
/// If you wanna use another regex, you can set it with the "regex" attribute
/// Only the specified regex (without the pid_identifier) is used in that case as shown in the example bellow.
/// The order of the parameters matter, if you define first the PID parameters 
/// for a group of joints and afterwards for a specific joint of that group,
/// the first value is overwritten with the second one.
/**
    Example usage:
    \code{.xml}
      <plugin name="rover_gazebo_joint_plugin" 
            filename="librover_gazebo_joint_plugin.so">

        <!-- Add a namespace -->
        <ros>
           <namespace>/my_namespace</namespace>
        </ros>

        <!-- Update rate in Hz -->
        <update_rate>100.0</update_rate>
        
        <!-- Parameters for position PIDs -->
        <position_pids>
          <DEP>350.0 0.1 0.0</DEP>
          <DEP regex="(^|_|:)DEP_LF($|_)">350.0 0.1 0.0</DEP>
          <STR>20.0 0.1 0.0</STR>
        </position_pids>

        <!-- Parameters for velocity PIDs -->
        <velocity_pids>
          <DRV>5.0 0.1 0.0</DRV>
        </velocity_pids>
      </plugin
  \endcode
  
*/

class RoverGazeboJointPlugin : public gazebo::ModelPlugin
{
public:
    /// Constructor
    RoverGazeboJointPlugin();
    /// Destructor
    ~RoverGazeboJointPlugin();

protected:
    // Documentation inherited
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    // Documentation inherited
    void Reset() override;

private:
    /// Private data pointer
    std::unique_ptr<RoverGazeboJointPluginPrivate> impl_;
};

struct PIDParameter
{
    std::string identifier;
    ignition::math::Vector3d pid_values;
    std::string regex;
};

} // namespace gazebo_plugins

#endif // GAZEBO_PLUGINS_ROVER_GAZEBO_JOINT_PLUGIN_HPP_
