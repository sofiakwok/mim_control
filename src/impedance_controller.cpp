/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Implementation of the ImpedanceController class.
 */

#include "mim_control/impedance_controller.hpp"
#include "odri_control_interface/utils.hpp"

#include "pinocchio/algorithm/frames.hpp"

namespace mim_control
{
ImpedanceController::ImpedanceController()
{
}

void ImpedanceController::initialize(const pinocchio::Model& pinocchio_model,
                                     const std::string& root_frame_name,
                                     const std::string& end_frame_name)
{
    // Copy the arguments internally.
    pinocchio_model_ = pinocchio_model;

    // Create the cache of the rigid body dynamics algorithms
    pinocchio_data_ = pinocchio::Data(pinocchio_model_);

    // Copy the arguments internally.
    root_frame_name_ = root_frame_name;
    end_frame_name_ = end_frame_name;

    // Fetch the index of the frame in the robot model.
    root_frame_index_ = pinocchio_model_.getFrameId(root_frame_name_);
    end_frame_index_ = pinocchio_model_.getFrameId(end_frame_name_);

    // initialize the size of the vectors.
    end_jacobian_.resize(6, pinocchio_model_.nv);
    end_jacobian_.fill(0.);
    impedance_jacobian_.resize(6, pinocchio_model_.nv);
    impedance_jacobian_.fill(0.);

    // Defines if the model has a freeflyer.
    pinocchio_model_has_free_flyer_ =
        pinocchio_model_.joints[1].shortname() == "JointModelFreeFlyer";

    // output
    torques_.resize(pinocchio_model_.nv, 1);
    torques_.fill(0.);
    if (pinocchio_model_has_free_flyer_)
        joint_torques_.resize(pinocchio_model_.nv, 1);
    else
    {
        joint_torques_.resize(pinocchio_model_.nv - 6, 1);
    }

    // Intermediate variables.
    root_orientation_ = pinocchio::SE3::Identity();
    end_orientation_ = pinocchio::SE3::Identity();

    // getting PD gain parameters from yaml file
    std::string yaml_path = "/home/sofia/bolt_hardware/workspace/src/robot_properties_bolt/src/robot_properties_bolt/resources/odri_control_interface/bolt_rw_gains.yaml";
    auto gains = odri_control_interface::PDFromYamlFile(yaml_path);
    Eigen::VectorXd pd_values = *gains; 
    kp_ = pd_values(4);
    kd_ = pd_values(5);
    std::cout << "kp: " << kp_ << std::endl; 
    std::cout << "kd: " << kd_ << std::endl;
}

void ImpedanceController::run(
    Eigen::Ref<const Eigen::VectorXd> robot_configuration,
    Eigen::Ref<const Eigen::VectorXd> robot_velocity,
    Eigen::Ref<const Array6d> gain_proportional,
    Eigen::Ref<const Array6d> gain_derivative,
    const double& gain_feed_forward_force,
    const pinocchio::SE3& desired_end_frame_placement,
    const pinocchio::Motion& desired_end_frame_velocity,
    const pinocchio::Force& feed_forward_force, 
    const double& output_torque,
    Eigen::Ref<const Eigen::VectorXd> desired_joint_pos)
{
    assert(robot_configuration.size() == pinocchio_model_.nq &&
           "robot_configuration is not of the good size.");
    assert(robot_velocity.size() == pinocchio_model_.nv &&
           "robot_velocity is not of the good size.");

    // Get the current frame placements and velocity.
    pinocchio::forwardKinematics(
        pinocchio_model_, pinocchio_data_, robot_configuration, robot_velocity);
    pinocchio::updateFramePlacement(
        pinocchio_model_, pinocchio_data_, root_frame_index_);
    pinocchio::updateFramePlacement(
        pinocchio_model_, pinocchio_data_, end_frame_index_);

    // Compute the jacobians
    pinocchio::computeJointJacobians(
        pinocchio_model_, pinocchio_data_, robot_configuration);

    run_precomputed_data(robot_configuration,
        robot_velocity, 
        pinocchio_data_,
        gain_proportional,
        gain_derivative,
        gain_feed_forward_force,
        desired_end_frame_placement,
        desired_end_frame_velocity,
        feed_forward_force, 
        output_torque, 
        desired_joint_pos
    );
}

void ImpedanceController::run_precomputed_data(
    Eigen::Ref<const Eigen::VectorXd> robot_configuration,
    Eigen::Ref<const Eigen::VectorXd> robot_velocity,
    pinocchio::Data& pinocchio_data,
    Eigen::Ref<const Array6d> gain_proportional,
    Eigen::Ref<const Array6d> gain_derivative,
    const double& gain_feed_forward_force,
    const pinocchio::SE3& desired_end_frame_placement,
    const pinocchio::Motion& desired_end_frame_velocity,
    const pinocchio::Force& feed_forward_force, 
    const double& output_torque, 
    Eigen::Ref<const Eigen::VectorXd> desired_joint_pos)
{
    root_placement_ = pinocchio_data.oMf[root_frame_index_];
    end_placement_ = pinocchio_data.oMf[end_frame_index_];
    root_velocity_ = pinocchio::getFrameVelocity(
        pinocchio_model_, pinocchio_data, root_frame_index_, pinocchio::WORLD);
    end_velocity_ = pinocchio::getFrameVelocity(
        pinocchio_model_, pinocchio_data, end_frame_index_, pinocchio::WORLD);

    // Orientations
    root_orientation_.rotation() = root_placement_.rotation();
    end_orientation_.rotation() = end_placement_.rotation();

    // Actual end frame placement in root frame.
    actual_end_frame_placement_ = root_placement_.actInv(end_placement_);

    // Placement error.
    err_se3_.head<3>() = root_orientation_.rotation() *
                         (desired_end_frame_placement.translation() -
                          actual_end_frame_placement_.translation());
    err_se3_.tail<3>() =
        pinocchio::log3(desired_end_frame_placement.rotation().transpose() *
                        actual_end_frame_placement_.rotation());

    // Actual end frame velocity in root frame.
    actual_end_frame_velocity_ =
        end_placement_.actInv(end_velocity_ - root_velocity_);

    // Velocity error.
    err_vel_ = end_orientation_.act(desired_end_frame_velocity -
                                    actual_end_frame_velocity_);

    // Compute the force to be applied to the environment.
    impedance_force_ = gain_proportional * err_se3_.array();
    impedance_force_ +=
        (gain_derivative * err_vel_.toVector().array()).matrix();
    impedance_force_ -=
        (gain_feed_forward_force * feed_forward_force.toVector().array())
            .matrix();

    // std::cout << "err_s3_ " << impedance_force_ << std::endl;
    // std::cout << "err_vel_ " << impedance_force_ << std::endl;
    // std::cout << "feed_forward_force: " << impedance_force_ << std::endl;

    // Get the jacobian.
    pinocchio::getFrameJacobian(pinocchio_model_,
                                pinocchio_data,
                                end_frame_index_,
                                pinocchio::LOCAL_WORLD_ALIGNED,
                                end_jacobian_);

    impedance_jacobian_ = end_jacobian_;

    // compute the output torques
    // std::cout << "impedance force: " << impedance_force_ << std::endl;
    // std::cout << "impedance jacobian: " << impedance_jacobian_ << std::endl;
    Eigen::VectorXd wbc_torque(12, 1);
    wbc_torque = output_torque * (impedance_jacobian_.transpose() * impedance_force_);

    //write PD controller to hold robot 
    /*const int nj = 6; //number of joints
    X_ = robot_configuration.tail<nj>();
    X_des_ = desired_joint_pos;

    V_ = robot_velocity.tail<nj>();
    V_des_ << 0, 0, 0, 0, 0, 0;

    pd_torque_scaling_ = 0; //1 - output_torque;

    pd_torque_ = pd_torque_scaling_ * ((kp_ * (X_des_ - X_) - kd_ * (V_des_ - V_)));

    // setting joint torques to be a combination of PD and WBC control
    torques_ = pd_torque_ + wbc_torque.tail<nj>();
    */
   torques_ = wbc_torque;

    if (pinocchio_model_has_free_flyer_)
        joint_torques_ = torques_.tail(pinocchio_model_.nv - 6);
    else
    {
        joint_torques_ = torques_;
    }
    return;
}

const Eigen::VectorXd& ImpedanceController::get_torques()
{
    return torques_;
}

const Eigen::VectorXd& ImpedanceController::get_joint_torques()
{
    return joint_torques_;
}

const ImpedanceController::Vector6d& ImpedanceController::get_impedance_force()
{
    return impedance_force_;
}

const pinocchio::FrameIndex& ImpedanceController::get_endframe_index()
{
    return end_frame_index_;
}

}  // namespace mim_control