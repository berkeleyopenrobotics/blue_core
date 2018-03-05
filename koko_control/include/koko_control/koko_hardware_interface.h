#ifndef KOKO_HARDWARE_INTERFACE_H
#define KOKO_HARDWARE_INTERFACE_H

#include <ros/ros.h>
#include <vector>
#include <string>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/differential_transmission.h>
#include <transmission_interface/transmission_interface.h>
#include <sensor_msgs/JointState.h>
#include <koko_hardware_drivers/MotorState.h>

#include <geometry_msgs/Vector3.h>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/segment.hpp>

namespace ti = transmission_interface;

class KokoHW: public hardware_interface::RobotHW
{

public:
  KokoHW(ros::NodeHandle &nh);
  void read();
  void write();

private:

  void motorStateCallback(const koko_hardware_drivers::MotorState::ConstPtr& msg);
  void calibrationStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void gravityVectorCallback(const geometry_msgs::Vector3ConstPtr& grav);

  void computeInverseDynamics();
  void buildDynamicChain(KDL::Chain &chain);

  // TODO: is this fully utilized? - brent
  struct JointParams
  {
    std::string joint_name;
    hardware_interface::JointHandle joint;
    double id_gain;
    double d_error;
    double p_error_last;
    double max_torque;
    double min_torque;
    double max_angle;
    double min_angle;
    double cmd;
    std::vector<double> err_dot_history;
    int err_dot_filter_length;
    int current_err_filter_insert;
  };

  bool read_from_motors_;

  // ROS control hardware interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::EffortJointInterface joint_effort_interface_;

  // Publishers and subscribers
  ros::Subscriber motor_state_sub_;
  std::vector<ros::Publisher> motor_cmd_publishers_;
  ros::Subscriber joint_state_tracker_sub;
  ros::Subscriber gravity_vector_sub_;

  // Parameters read in from configuration
  std::vector<std::string> joint_names_;
  std::vector<std::string> motor_names_;
  std::vector<double> gear_ratios_;
  std::vector<double> current_slope_;
  std::vector<int> paired_constraints_;
  std::vector<double> min_angles_;
  std::vector<double> max_angles_;
  std::vector<double> joint_torque_directions_;
  double softstop_torque_limit_;
  double softstop_tolerance_;

  // Calibration
  int calibration_counter_;
  bool is_calibrated_;
  std::vector<double> joint_pos_initial_;
  std::vector<double> actuator_pos_initial_;
  int num_joints_;

  // Transmission interfaces
  ti::ActuatorToJointStateInterface actuator_to_joint_interface_;
  ti::JointToActuatorEffortInterface joint_to_actuator_interface_;
  std::vector<ti::SimpleTransmission *> simple_transmissions_;
  std::vector<ti::DifferentialTransmission *> differential_transmissions_;

  // Actuator and joint space data
  std::vector<ti::ActuatorData> actuator_states_;
  std::vector<ti::ActuatorData> actuator_commands_;
  std::vector<ti::JointData> joint_states_;
  std::vector<ti::JointData> joint_commands_;

  // Owned by our ActuatorData and JointData objects
  std::vector<double> actuator_pos_;
  std::vector<double> actuator_vel_;
  std::vector<double> actuator_eff_;
  std::vector<double> actuator_cmd_;
  std::vector<double> joint_pos_;
  std::vector<double> joint_vel_;
  std::vector<double> joint_eff_;
  std::vector<double> joint_cmd_;

  // TODO: are these redundant? - brent
  std::vector<double> cmd;
  std::vector<double> pos;
  std::vector<double> vel;
  std::vector<double> eff;

  // Gravity Compensation
  KDL::Vector gravity_vector_;
  KDL::Chain kdl_chain_;
  KDL::JntArray id_torques;
  std::vector<JointParams*> joint_params_;
};

#endif // KOKO_HARDWARE_INTERFACE
