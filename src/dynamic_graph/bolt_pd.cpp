/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Dynamic graph wrapper around the BoltPD class.
 */

#include "mim_control/dynamic_graph/bolt_pd.hpp"

#include "Eigen/Eigen"
#include "dynamic-graph/factory.h"
#include "mim_control/dynamic_graph/signal_utils.hpp"

namespace mim_control
{
namespace dynamic_graph
{
typedef Eigen::Map<const pinocchio::SE3::Quaternion> QuatConstMap;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(BoltPD, "BoltPD");

BoltPD::BoltPD(const std::string& name)
    :  // Inheritance.
      dynamicgraph::Entity(name),
      // Input signals.
      define_input_signal(robot_configuration_sin_, "VectorXd"),
      define_input_signal(robot_velocity_sin_, "VectorXd"),
      define_input_signal(des_robot_configuration_sin_, "VectorXd"),
      define_input_signal(des_robot_velocity_sin_, "VectorXd"),
      // Output signals.
      define_output_signal(torque_sout_,
                           "inner",
                           one_iteration_sout_,
                           &BoltPD::torque_callback),
      define_output_signal(joint_torque_sout_,
                           "inner",
                           one_iteration_sout_,
                           &BoltPD::joint_torque_callback),
      // Inner signal.
      one_iteration_sout_(
          boost::bind(
              &BoltPD::one_iteration_callback, this, _1, _2),
          robot_configuration_sin_
              << robot_velocity_sin_ 
              << des_robot_configuration_sin_
              << des_robot_velocity_sin_,
          make_signal_string(
              false, CLASS_NAME, name, "bool", "one_iteration_sout"))
{

    std::cout << "signal init" << std::endl;
    signalRegistration(
        robot_configuration_sin_
        << robot_velocity_sin_ 
        << des_robot_configuration_sin_ << des_robot_velocity_sin_ 
        << torque_sout_ << joint_torque_sout_);

    std::cout << "registered signals" << std::endl;
}

void BoltPD::initialize(const pinocchio::Model& pinocchio_model)
{
    pd_controller_.initialize(pinocchio_model);
}

dynamicgraph::Vector& BoltPD::torque_callback(
    dynamicgraph::Vector& signal_data, int time)
{
    one_iteration_sout_.access(time);
    signal_data = pd_controller_.get_torques();
    return signal_data;
}

dynamicgraph::Vector& BoltPD::joint_torque_callback(
    dynamicgraph::Vector& signal_data, int time)
{
    one_iteration_sout_.access(time);
    signal_data = pd_controller_.get_joint_torques();
    return signal_data;
}

bool& BoltPD::one_iteration_callback(bool& signal_data, int time)
{
    const dynamicgraph::Vector& robot_configuration =
        robot_configuration_sin_.access(time);
    const dynamicgraph::Vector& robot_velocity =
        robot_velocity_sin_.access(time);
    const dynamicgraph::Vector& des_robot_configuration =
        des_robot_configuration_sin_.access(time);
    const dynamicgraph::Vector& des_robot_velocity =
        des_robot_velocity_sin_.access(time);


    pd_controller_.run(robot_configuration,
                        robot_velocity,
                        des_robot_configuration,
                        des_robot_velocity);
    signal_data = true;
    return signal_data;
}

}  // namespace dynamic_graph
}  // namespace mim_control
