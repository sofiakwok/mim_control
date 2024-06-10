/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief This is the implementation for impedance controller between any two
 * frames of the robot.
 *
 */

#pragma once

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"

namespace mim_control
{
/**
 * @brief Impedance controller between any two frames of the robot.
 */
class LQRController
{
public:
    typedef Eigen::Array<double, 6, 1> Array6d;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;

    /**
     * @brief Construct a new LQRController object.
     */
    LQRController();

    /**
     * @brief Initialize the internal data. None real-time safe method.
     *
     * @param pinocchio_model rigid body model of the robot
     * @param root_frame_name root frame name where the spring starts(Ex. Hip)
     * @param end_frame_name frame name where the spring ends(Ex. end effector)
     */
    void initialize(const pinocchio::Model& pinocchio_model);

    /**
     * @brief Computes the desired joint torques
     *
     * \f$
     * \tau = J^T (k_p (x^{des} - x) +
     *        k_d (\dot{x}^{des} - \dot{x}) -k_f f)
     * \f$
     *
     * with:
     * - \f$ \tau \f$ the joint torques,
     * - \f$ J \f$ the Jacobian of the sub-kinematic-tree between the root and
     *   the end frame,
     * - \f$ k_p \f$ the gain proportional to the frame placement error,
     * - \f$ x_{des} \f$ desired end frame placement with respect to the
     *   desired root frame.
     * - \f$ x \f$ measured end frame placement with respect to the
     *   measured root frame.
     * - \f$ k_d \f$ is the derivative gain applied to the time derivative of
     *   the error.
     * - \f$ \dot{x}^{des} \f$ desired end frame velocity with respect to the
     *   desired root frame.
     * - \f$ \dot{x} \f$ measured end frame velocity with respect to the
     *   measured root frame.
     * - \f$ k_f \f$ the gain over the feed forward force,
     * - \f$ f \f$ the feed forward force,
     *
     * @param robot_configuration robot generalized coordinates configuration.
     * @param robot_velocity robot generalized coordinates velocity.
     * @param gain_proportional 6d vector for the proportional gains on {x, y,
     * z, roll, pitch, yaw}.
     * @param gain_derivative 6d vector for the proportional gains on {x, y, z,
     * roll, pitch, yaw}.
     * @param gain_feed_forward_force gain multiplying the feed forward force.
     * @param desired_end_frame_placement desired end frame placement relative
     * to the desired root joint.
     * @param desired_end_frame_velocity desired end frame velocity relative to
     * the desired root joint.
     * @param feed_forward_force feed forward force applied to the foot by the
     * environment.
     */
    void run(Eigen::Ref<const Eigen::VectorXd> robot_configuration,
             Eigen::Ref<const Eigen::VectorXd> robot_velocity,
             Eigen::Ref<const Eigen::VectorXd> des_robot_configuration,
             Eigen::Ref<const Eigen::VectorXd> des_robot_velocity);

    void run_controller(Eigen::Ref<const Eigen::VectorXd> robot_configuration,
             Eigen::Ref<const Eigen::VectorXd> robot_velocity,
             Eigen::Ref<const Eigen::VectorXd> des_robot_configuration,
             Eigen::Ref<const Eigen::VectorXd> des_robot_velocity);


    /**
     * @brief Get the computed torques from the impedance controller.
     *
     * @return Eigen::VectorXd&
     */
    const Eigen::VectorXd& get_torques();

    /**
     * @brief Get the computed joint torques from the impedance controller.
     *
     * @return Eigen::VectorXd&
     */
    const Eigen::VectorXd& get_joint_torques();

private:  // attributes
    /** @brief Rigid body dynamics model. */
    pinocchio::Model pinocchio_model_;

    /** @brief Cache of the rigid body dynamics algorithms. */
    pinocchio::Data pinocchio_data_;

    /** @brief Output torques. */
    Eigen::VectorXd torques_;

    /** @brief Output torques. */
    Eigen::VectorXd joint_torques_;

    /** @brief Checks out if the Pinocchio rigid body model of the robot
     * contains a free-flyer. This is used to return the command i.e. the
     * torques to be applied to the joints only. */
    bool pinocchio_model_has_free_flyer_;

    Eigen::MatrixXd Q_;
    Eigen::MatrixXd Qf_;
    Eigen::MatrixXd R_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd K_;
    Eigen::MatrixXd Kinf_;

};

}  // namespace lqr_control
