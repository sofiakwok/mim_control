/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Python bindings for the StepperHead class
 */

#include "mim_control/rw_pd_controller.hpp"
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <boost/python.hpp>

namespace py = pybind11;

namespace mim_control
{
void bind_rw_pd_controller(py::module& module)
{
    py::class_<RWPDController>(module, "RWPDController")
        .def(py::init<>())

        // Public methods.
        .def("initialize",
             [](RWPDController& obj,
                py::object model) {
                 const pinocchio::Model& pinocchio_model =
                     boost::python::extract<const pinocchio::Model&>(
                         model.ptr());
                 obj.initialize(
                     pinocchio_model);
                 return;
             })
        .def("run",
             [](RWPDController& obj,
                const Eigen::VectorXd& robot_configuration,
                const Eigen::VectorXd& robot_velocity,
                const Eigen::VectorXd& des_robot_configuration,
                const Eigen::VectorXd& des_robot_velocity) {
                 
                 obj.run(robot_configuration,
                         robot_velocity,
                         des_robot_configuration,
                         des_robot_velocity);
                 return;
             })
        .def("get_torques",
             &RWPDController::get_torques,
             py::return_value_policy::reference)
        .def("get_joint_torques",
             &RWPDController::get_joint_torques,
             py::return_value_policy::reference);
}

}  // namespace pd_control