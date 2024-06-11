/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Implementation of the LQRController class.
 */

#include "mim_control/lqr_controller.hpp"
#include "pinocchio/algorithm/frames.hpp"


namespace mim_control
{
LQRController::LQRController()
{
}

void LQRController::initialize(const pinocchio::Model& pinocchio_model)
{
    // Copy the arguments internally.
    pinocchio_model_ = pinocchio_model;

    // Create the cache of the rigid body dynamics algorithms
    pinocchio_data_ = pinocchio::Data(pinocchio_model_);

    int nu = 6;
    int nx = 25;

    Kinf_.resize(nu, nx);
    Kinf_.fill(0);

    std::ifstream fin ("/home/sofia/bolt_hardware/workspace/src/mim_control/src/Kinf.txt");
    if (fin.is_open())
    {
        for (int row = 0; row < nu; row++)
            for (int col = 0; col < nx; col++)
            {
                double item = 0.0;
                fin >> item;
                Kinf_(row, col) = item;
            }
        fin.close();
    } else {
        std::cout << "error opening file" << std::endl;
    }

    std::cout << "Kinf calculated: " << Kinf_ << std::endl;

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
}

void LQRController::run(
    Eigen::Ref<const Eigen::VectorXd> robot_configuration,
    Eigen::Ref<const Eigen::VectorXd> robot_velocity,
    Eigen::Ref<const Eigen::VectorXd> des_robot_configuration,
    Eigen::Ref<const Eigen::VectorXd> des_robot_velocity)
{
    assert(robot_configuration.size() == pinocchio_model_.nq &&
           "robot_configuration is not of the good size.");
    assert(robot_velocity.size() == pinocchio_model_.nv &&
           "robot_velocity is not of the good size.");

    run_controller(robot_configuration, 
    robot_velocity, 
    des_robot_configuration, 
    des_robot_velocity);
}

void LQRController::run_controller(
    Eigen::Ref<const Eigen::VectorXd> robot_configuration,
    Eigen::Ref<const Eigen::VectorXd> robot_velocity,
    Eigen::Ref<const Eigen::VectorXd> des_robot_configuration,
    Eigen::Ref<const Eigen::VectorXd> des_robot_velocity)
{
    Eigen::MatrixXd X(25, 1);
    X.block<13, 1>(0, 0) = robot_configuration;
    X.block<12, 1>(13, 0) = robot_velocity;
    Eigen::MatrixXd X_des(25, 1); 
    X_des.block<13, 1>(0, 0) = des_robot_configuration;
    X_des.block<12, 1>(13, 0) = des_robot_velocity;
    //std::cout << "x diff: " << X - X_des << std::endl;
    torques_ = -Kinf_ * (X - X_des);
    if (pinocchio_model_has_free_flyer_)
        joint_torques_ = torques_.tail(pinocchio_model_.nv - 6);
    else
    {
        joint_torques_ = torques_;
    }
    // std::cout << "joint torques: " << joint_torques_ << std::endl;
    return;
}

const Eigen::VectorXd& LQRController::get_torques()
{
    return torques_;
}

const Eigen::VectorXd& LQRController::get_joint_torques()
{
    return joint_torques_;
}

}  // namespace lqr_control