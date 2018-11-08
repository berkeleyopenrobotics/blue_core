#include "blue_hardware_interface/blue_hardware_interface.h"
#include "blue_hardware_interface/blue_kinematics.h"
#include "blue_msgs/MotorState.h"

#include <ros/assert.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

namespace ti = transmission_interface;

BlueHW::BlueHW(ros::NodeHandle &nh) : nh_(nh) { 
  // Read robot parameters
  loadParams();

  // Motor driver bringup
  motor_driver_.init(
      params_.serial_port,
      params_.motor_ids);

  // Build helper for robot dynamics
  // (Used to compute gravity compensation torques)
  dynamics_.init(
      params_.robot_description,
      params_.baselink,
      params_.endlink);

  // Set up kinematics
  kinematics_.init(
      params_.joint_names,
      params_.differential_pairs,
      params_.gear_ratios);

  // Register joint interfaces with the controller manager
  registerInterface(&kinematics_.joint_state_interface);
  registerInterface(&kinematics_.joint_effort_interface);

  // ROS communications setup
  motor_states_.name = params_.motor_names;
  motor_state_publisher_ = nh.advertise<blue_msgs::MotorState>(
      "blue_hardware/motor_states", 1);
  joint_startup_calibration_service_ = nh.advertiseService(
      "blue_hardware/joint_startup_calibration",
      &BlueHW::jointStartupCalibration,
      this);

  for (auto id : params_.motor_ids)
    motor_commands_[id] = 0.0;
}

void BlueHW::read() {
  // Motor communication! Simultaneously write commands and read state
  motor_driver_.update(motor_commands_, motor_states_);

  // Publish the motor states, in case anybody's listening
  motor_states_.header.stamp = ros::Time::now();
  motor_state_publisher_.publish(motor_states_);

  // std::vector<float> => <double> casting
  const blue_msgs::MotorState *m = &motor_states_;
  std::vector<double> positions(m->position.begin(), m->position.end());
  std::vector<double> velocities(m->velocity.begin(), m->velocity.end());
  std::vector<double> accel_x(m->accel_x.begin(), m->accel_x.end());
  std::vector<double> accel_y(m->accel_y.begin(), m->accel_y.end());
  std::vector<double> accel_z(m->accel_z.begin(), m->accel_z.end());

  std::vector<double> efforts;
  for (int i = 0; i < params_.motor_ids.size(); i++) {
    efforts.push_back(
        motor_states_.quadrature_current[i] / params_.current_to_torque_ratios[i]);
  }

  // Update kinematics with motor states
  kinematics_.setActuatorStates(
      positions,
      velocities,
      efforts,
      accel_x,
      accel_y,
      accel_z);

  // Update orientation for gravity compensation
  dynamics_.setGravityVector(
      kinematics_.getGravityVector());
}

void BlueHW::write() {
  // Compute gravity compensation
  auto gravity_comp_torques = dynamics_.computeGravityComp(
      kinematics_.getJointPos(),
      kinematics_.getJointVel());

  // Apply gravity compensation fine tuning terms
  for (int i = 0; i < gravity_comp_torques.size(); i++)
    gravity_comp_torques[i] *= params_.id_torque_gains[i];

  // Get actuator commands, using the gravity comp torques as a feedforward
  auto actuator_commands = kinematics_.getActuatorCommands(
      gravity_comp_torques,
      params_.softstop_torque_limit, // TODO: clean up softstop code
      params_.softstop_min_angles,
      params_.softstop_max_angles,
      params_.softstop_tolerance);

  // Post-process motor commands
  for (int i = 0; i < actuator_commands.size(); i++) {
    // Convert torque to current
    // TODO: use driver internal torque control mode
    actuator_commands[i] = actuator_commands[i] * params_.current_to_torque_ratios[i];

    // Apply current limit
    actuator_commands[i] = std::max(
        std::min(actuator_commands[i], params_.motor_current_limits[i]),
        -params_.motor_current_limits[i]);

    // Update our command map
    motor_commands_[params_.motor_ids[i]] = actuator_commands[i];
  }
}

template <typename TParam>
void BlueHW::getParam(const std::string name, TParam& dest) {
  // Try to find a parameter and explode if it doesn't exist
  ROS_ASSERT_MSG(
    nh_.getParam(name, dest),
    "Could not find %s parameter in namespace %s",
    name.c_str(),
    nh_.getNamespace().c_str()
  );
}

bool BlueHW::jointStartupCalibration(
    blue_msgs::JointStartupCalibration::Request &request,
    blue_msgs::JointStartupCalibration::Response &response
) {
  kinematics_.setJointOffsets(request.joint_positions);
  response.success = true;

  return true;
}

void BlueHW::loadParams() {
  // Motor driver stuff
  getParam("blue_hardware/serial_port", params_.serial_port);
  getParam("blue_hardware/motor_names", params_.motor_names);
  std::vector<int> temp_motor_ids;
  getParam("blue_hardware/motor_ids", temp_motor_ids);
  for (int id : temp_motor_ids)
    params_.motor_ids.push_back(id);

  // Parameters for parsing URDF
  getParam("robot_description", params_.robot_description);
  getParam("blue_hardware/baselink", params_.baselink);
  getParam("blue_hardware/endlink", params_.endlink);

  // Read data needed for kinematics
  getParam("blue_hardware/joint_names", params_.joint_names);
  getParam("blue_hardware/gear_ratios", params_.gear_ratios);
  getParam("blue_hardware/differential_pairs", params_.differential_pairs);

  // Torque => current conversion stuff
  getParam("blue_hardware/motor_current_limits", params_.motor_current_limits);
  getParam("blue_hardware/current_to_torque_ratios", params_.current_to_torque_ratios);

  // Gravity compensation tuning
  getParam("blue_hardware/id_torque_gains", params_.id_torque_gains);

  // Soft stops
  // TODO: hacky and temporary
  getParam("blue_hardware/softstop_torque_limit", params_.softstop_torque_limit);
  getParam("blue_hardware/softstop_min_angles", params_.softstop_min_angles);
  getParam("blue_hardware/softstop_max_angles", params_.softstop_max_angles);
  getParam("blue_hardware/softstop_tolerance", params_.softstop_tolerance);

  // Links to attach accelerometer measurements to
  getParam("blue_hardware/accel_links", params_.accel_links);
}
