/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Implementation of the RWLQRController class.
 */

#include "mim_control/rw_lqr_controller.hpp"
#include "pinocchio/algorithm/frames.hpp"


namespace mim_control
{
RWLQRController::RWLQRController()
{
}

void RWLQRController::initialize(const pinocchio::Model& pinocchio_model)
{
    // Copy the arguments internally.
    pinocchio_model_ = pinocchio_model;

    // Create the cache of the rigid body dynamics algorithms
    pinocchio_data_ = pinocchio::Data(pinocchio_model_);

    int nu = 7;
    int nx = 27;

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

    //std::cout << "Kinf calculated: " << Kinf_ << std::endl;

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

void RWLQRController::run(
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

void RWLQRController::run_controller(
    Eigen::Ref<const Eigen::VectorXd> robot_configuration,
    Eigen::Ref<const Eigen::VectorXd> robot_velocity,
    Eigen::Ref<const Eigen::VectorXd> des_robot_configuration,
    Eigen::Ref<const Eigen::VectorXd> des_robot_velocity)
{
    /*int nx = 27;
    Eigen::MatrixXd X(nx, 1);
    X.block<14, 1>(0, 0) = robot_configuration;
    X.block<13, 1>(14, 0) = robot_velocity;
    Eigen::MatrixXd X_des(nx, 1); 
    X_des.block<14, 1>(0, 0) = des_robot_configuration;
    X_des.block<13, 1>(14, 0) = des_robot_velocity;
    std::cout << "X: " << X << std::endl;
    std::cout << "x diff: " << X - X_des << std::endl;
    torques_ = -Kinf_ * (X - X_des);
    if (pinocchio_model_has_free_flyer_)
        joint_torques_ = torques_.tail(pinocchio_model_.nv - 6);
    else
    {
        joint_torques_ = torques_;
    }
    std::cout << "joint torques: " << joint_torques_ << std::endl;*/

    // controlling only reaction wheel based on pitch angle
    // keeping all other joints locked at desired configuration
    des_ori_quat_.w() = des_robot_configuration[3];
    des_ori_quat_.vec()[0] = des_robot_configuration[0];
    des_ori_quat_.vec()[1] = des_robot_configuration[1];
    des_ori_quat_.vec()[2] = des_robot_configuration[2];

    ori_quat_.w() = robot_configuration[3];
    ori_quat_.vec()[0] = robot_configuration[0];
    ori_quat_.vec()[1] = robot_configuration[1];
    ori_quat_.vec()[2] = robot_configuration[2];

    //des_ori_se3_ = des_ori_quat_.toRotationMatrix().eulerAngles(2, 1, 0);
    //ori_se3_ = ori_quat_.toRotationMatrix().eulerAngles(2, 1, 0);

    //std::cout << "des ori: " << des_ori_se3_ << std::endl;
    //std::cout << "ori: " << ori_se3_ << std::endl;

    // Compute the pitch error
    ori_error_ = des_ori_quat_ * ori_quat_.conjugate();
    std::cout << "quat: w:" << ori_error_.w() << " ang:" << ori_error_.vec() << std::endl;
    // converting to axis-angle representation
    axis_error_ = ori_error_;
    std::cout << "axis error: " << axis_error_.axis() << " angle:" << axis_error_.angle() << std::endl;
    // float angle_rad = acos(ori_error_.w()) * 2;
    // float x = ori_error_.vec()[0] / sin(angle_rad/2);
    // float y = ori_error_.vec()[1] / sin(angle_rad/2);
    // float z = ori_error_.vec()[2] / sin(angle_rad/2);
    // std::cout << "pitch err: " << angle_rad << std::endl;
    double pitch_err = axis_error_.angle();
    
    // map pitch error to [-pi/2, pi/2] space
    double pitch_err_map = pitch_err;
    if (std::abs(pitch_err) > M_PI/2){
        int sign = (pitch_err > 0) - (pitch_err < 0);
        auto tmp = pitch_err - sign*M_PI;
        pitch_err_map = tmp;
    }
    // auto tmp = fmod(pitch_err - sign*M_PI, M_PI/2);
    // if (tmp < 0){
    //     tmp += M_PI/2;
    // }
    // double pitch_err_map = fmod(pitch_err, M_PI/2);
    std::cout << "mapped pitch err: " << pitch_err_map << std::endl;

    double rw_vel = robot_velocity[12];
    std::cout << "rw vel: " << rw_vel << std::endl;

    const int nj = 7; // number of joints to control
    const int nv = 7; // number of velocities to control
    Eigen::MatrixXd X(nj, 1);
    X = robot_configuration.block<nj, 1>(7, 0);
    Eigen::MatrixXd X_des(nj, 1); 
    X_des = des_robot_configuration.block<nj, 1>(7, 0);

    Eigen::MatrixXd V(nv, 1);
    V = robot_velocity.block<nv, 1>(6, 0);
    Eigen::MatrixXd V_des(nv, 1); 
    V_des = des_robot_velocity.block<nv, 1>(6, 0);

    // PD controller
    double kp = 8.0;
    double kd = 0.1;
    double kp_rw = 10.0;
    double kd_rw = 0.01;

    Eigen::VectorXd joint_control(nv, 1);
    joint_control = kp * (X_des - X) + kd * (V_des - V);
    joint_control[6] = kp_rw * (0 - pitch_err_map) + kd_rw * (0 - rw_vel);
    std::cout << "rw control: " << joint_control[6] << std::endl;
    joint_torques_ = joint_control; 
    return;
}

const Eigen::VectorXd& RWLQRController::get_torques()
{
    return torques_;
}

const Eigen::VectorXd& RWLQRController::get_joint_torques()
{
    return joint_torques_;
}

}  // namespace lqr_control