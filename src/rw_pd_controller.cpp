/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Implementation of the RWPDController class.
 */

#include "mim_control/rw_pd_controller.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "odri_control_interface/utils.hpp"

namespace mim_control
{
RWPDController::RWPDController()
{
}

void RWPDController::initialize(const pinocchio::Model& pinocchio_model)
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

void RWPDController::run(
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

void RWPDController::run_controller(
    Eigen::Ref<const Eigen::VectorXd> robot_configuration,
    Eigen::Ref<const Eigen::VectorXd> robot_velocity,
    Eigen::Ref<const Eigen::VectorXd> des_robot_configuration,
    Eigen::Ref<const Eigen::VectorXd> des_robot_velocity)
{
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
    ori_error_ = ori_quat_; //des_ori_quat_ * ori_quat_.conjugate();
    //std::cout << "quat error: w:" << ori_error_.w() << " ang:" << ori_error_.vec() << std::endl;
    Eigen::Vector3d u;
    u << robot_configuration[0], robot_configuration[1], robot_configuration[2];
    double s = ori_error_.w();
    //calculate yaw error 
    Eigen::Vector3d x_ref;
    x_ref << 1, 0, 0;
    auto yaw_vec = (2.0f * u.dot(x_ref) * u) + ((s*s - u.dot(u)) * x_ref) + (2.0f * s * u.cross(x_ref));
    double yaw_err = std::atan(yaw_vec[1] / yaw_vec[0]);
    //std::cout << "yaw err: " << yaw_err << std::endl;
    // calculate pitch error
    Eigen::Vector3d z_ref;
    z_ref << 0, 0, 1; 
    auto body_point = (2.0f * u.dot(z_ref) * u) + ((s*s - u.dot(u)) * z_ref) + (2.0f * s * u.cross(z_ref));
    Eigen::Matrix<double, 3, 3> yaw_transform;
    yaw_transform = Eigen::AngleAxisd(yaw_err, Eigen::Vector3d::UnitZ());
    auto body_frame = yaw_transform * body_point;
    //std::cout << "body mapped point: " << body_frame << std::endl;
    // projecting onto xz plane and calculating pitch from there
    double pitch_err = std::atan(body_frame[0] / body_frame[2]);
    // axis_error_ = ori_error_;
    // std::cout << "axis error: " << axis_error_.axis() << " angle:" << axis_error_.angle() << std::endl;
    // std::cout << "pitch err: " << angle_rad << std::endl;
    // double pitch_err = axis_error_.angle();
    
    // map pitch error to [-pi/2, pi/2] space
    double pitch_err_map = pitch_err;
    if (std::abs(pitch_err) > M_PI/2){
        int sign = (pitch_err > 0) - (pitch_err < 0);
        auto tmp = pitch_err - sign*M_PI;
        pitch_err_map = tmp;
    }
    //std::cout << "mapped pitch err: " << pitch_err_map << std::endl;
    if (std::abs(pitch_err_map) < 0.1){
        pitch_err_map = 0;
    }

    double rw_vel = robot_velocity[12];
    //std::cout << "rw vel: " << rw_vel << std::endl;

    const int nj = 7; // number of joints to control
    Eigen::VectorXd X(nj, 1);
    X = robot_configuration.tail<nj>();
    Eigen::VectorXd X_des(nj, 1); 
    X_des = des_robot_configuration.tail<nj>();

    Eigen::VectorXd V(nj, 1);
    V = robot_velocity.tail<nj>();
    Eigen::VectorXd V_des(nj, 1); 
    V_des = des_robot_velocity.tail<nj>();

    // // PD controller
    // // hardware settings
    // double kp = 5.0 * 0.25;
    // double kd = 0.01 * 0.25;
    // // sim settings
    // // kp = 5.0 * 0.25;
    // // kd = -0.1 * 0.25;
    // double kp_rw = -5.0 * 1.25;
    // double kd_rw = -0.1 * 1.25;

    // getting settings from yaml file
    std::string yaml_path = "/home/sofia/bolt_hardware/workspace/src/robot_properties_bolt/src/robot_properties_bolt/resources/odri_control_interface/bolt_rw_gains.yaml";
    auto gains = odri_control_interface::PDFromYamlFile(yaml_path);
    Eigen::VectorXd pd_values = *gains; 
    double kp = pd_values(0);
    double kd = pd_values(1);
    double kp_rw = pd_values(2);
    double kd_rw = pd_values(3);

    Eigen::VectorXd joint_control(nj, 1);
    joint_control = kp * (X_des - X) - kd * (V_des - V);
    //std::cout << "X: " << X << std::endl;
    joint_control[6] = kp_rw * (0 - pitch_err_map) - kd_rw * (0 - rw_vel);
    
    torques_ = joint_control;
    joint_torques_ = joint_control; 
    // rw torque turns wheel clockwise (as viewed from left side of robot)
    //std::cout << "torques: " << joint_torques_ << std::endl;
    return;
}

const Eigen::VectorXd& RWPDController::get_torques()
{
    return torques_;
}

const Eigen::VectorXd& RWPDController::get_joint_torques()
{
    return joint_torques_;
}

}  // namespace pd_control