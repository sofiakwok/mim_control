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

    // load in A and B matrices
    /*Eigen::MatrixXd A = Eigen::MatrixXd::Zero(25, 25);
    std::ifstream fin("A.txt");
    for (int i = 0; i < 25; ++i) {
        for (int j = 0; j < 25; ++j) {
            double item = 0.0;
            fin >> item;
            A(i, j) = item;
        }
    }
    fin.close();
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(25, 6);
    std::ifstream finb("b.txt");
    for (int i = 0; i < 25; ++i) {
        for (int j = 0; j < 6; ++j) {
            double item = 0.0;
            finb >> item;
            b(i, j) = item;
        }
    }
    finb.close();

    Eigen::MatrixXd A_T = A.transpose();
    Eigen::MatrixXd b_T = b.transpose();
    */

    /*
    // TODO: calculate Kinf
    int ricatti_iter = 1000;
    int n = 2;
    Q_.resize(25, 25);
    Q_.setIdentity();
    Qf_.resize(25, 25);
    Qf_.setIdentity();
    R_.resize(6, 6);
    R_.setIdentity();
    //R_ *= 0.1;
    Kinf_.resize(6, 1);
    Kinf_.fill(0);

    Eigen::MatrixXd P_ = Eigen::MatrixXd::Zero(25, 25*n);
    Eigen::MatrixXd K_ = Eigen::MatrixXd::Zero(25, 25*(n-1));

    std::cout << "initialized matrices" << std::endl;

    for (int i = 0; i < ricatti_iter; i++){
        P_.resize(25, 25*n);
        P_.fill(0);
        K_.resize(25, 25*n-1);
        K_.fill(0);
        std::cout << "resized p and k: " << n << std::endl;
        P_.block<25,25>(0, 25*(n-1)) = Qf_;
        std::cout << "set last block to qf" << std::endl;
        for (int k = n-1; k > 1; k--){
            K_.block<25,25>(0, k*25) = (R_ + b_T*P_.block<25,25>(0, (k+1)*25)*b_T)*(b_T*P_.block<25,25>(0, (k+1)*25)*A).inverse();
            P_.block<25,25>(0, k*25) = Q_ + A_T*P_.block<25,25>(0, (k+1)*25)*(A - b*K_.block<25,25>(0, k*25));
            std::cout << "blocking K and P" << std::endl;
        }
        if ((P_.block<25,25>(0, 25) - P_.block<25,25>(0, 0)).norm() <= 1e-5){
            break;
        }
        std::cout << "checked norm" << std::endl;
        n += 1;
    }
    Kinf_ = K_.block<25,25>(0, 0);
    std::cout << "Kinf calculated" << std::endl;
    */
    // Eigen::MatrixXd Kinf_;
    // Kinf_.resize(6, 1);
    // Kinf_.fill(0);

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
    std::cout << "pinocchio model nv: " << pinocchio_model_.nv << std::endl;

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
    // Eigen::MatrixXd X = (robot_configuration, robot_velocity);
    // Eigen::MatrixXd X_des = (des_robot_configuration, des_robot_velocity);
    // torques_ = -Kinf_ * (X - X_des);
    torques_.fill(0);
    if (pinocchio_model_has_free_flyer_)
        joint_torques_ = torques_.tail(pinocchio_model_.nv - 6);
    else
    {
        joint_torques_ = torques_;
    }
    //std::cout << "joint torques: " << joint_torques_ << std::endl;
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