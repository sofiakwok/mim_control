/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Dynamic graph wrapper around the LQRController class.
 *
 */

#pragma once

// clang-format off
#include "pinocchio/fwd.hpp"
// clang-format on
#include "dynamic-graph/all-signals.h"
#include "dynamic-graph/entity.h"
#include "mim_control/lqr_controller.hpp"

namespace mim_control
{
namespace dynamic_graph
{
/**
 * @brief Entity around the LQRController class of this package.
 */
class LQRController : public dynamicgraph::Entity
{
    DYNAMIC_GRAPH_ENTITY_DECL();

public:
    /**
     * @brief Construct a new LQRController object using its Dynamic Graph
     * name.
     *
     * @param name
     */
    LQRController(const std::string& name);

    /**
     * @brief Initialize the internal data. None real-time safe method.
     *
     * @param pinocchio_model rigid body model of the robot
     * @param root_frame_name root frame name where the spring starts(Ex. Hip)
     * @param end_frame_name frame name where the spring ends(Ex. end effector)
     */
    void initialize(const pinocchio::Model& pinocchio_model);

    /*
     * Input Signals.
     */

    /** @brief Robot generalized coordinates (q). */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> robot_configuration_sin_;

    /** @brief Robot generalized velocity (dq). */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> robot_velocity_sin_;

    /** @brief Robot generalized coordinates (q). */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> des_robot_configuration_sin_;

    /** @brief Robot generalized velocity (dq). */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> des_robot_velocity_sin_;

    /*
     * Output Signals.
     */

    /** @brief Output joint torques. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int> torque_sout_;

    /** @brief Output generalized torques. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        joint_torque_sout_;

    /** @brief Internal transition signal. */
    dynamicgraph::SignalTimeDependent<bool, int> one_iteration_sout_;

protected:
    /**
     * @brief Callback function of the torque_sout_ signal.
     *
     * @param torque
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector& torque_callback(dynamicgraph::Vector& torque,
                                          int time);

    /**
     * @brief Callback function of the joint_torque_sout_ signal.
     *
     * @param signal_data
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector& joint_torque_callback(
        dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Internally calls the LQRController class.
     *
     * @param signal_data
     * @param time
     * @return true
     * @return false
     */
    bool& one_iteration_callback(bool& signal_data, int time);

    /** @brief Actual controller class we wrap around. */
    mim_control::LQRController lqr_controller_;
};

}  // namespace dynamic_graph
}  // namespace lqr_control
