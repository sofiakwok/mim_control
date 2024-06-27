/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Expose the Device and the periodic call to python.
 */

#include "dynamic-graph/python/module.hh"
#include "dynamic-graph/python/signal.hh"
#include "mim_control/dynamic_graph/centroidal_force_qp_controller.hpp"
#include "mim_control/dynamic_graph/centroidal_pd_controller.hpp"
#include "mim_control/dynamic_graph/impedance_controller.hpp"
#include "mim_control/dynamic_graph/rw_impedance_controller.hpp"
#include "mim_control/dynamic_graph/lqr_controller.hpp"
#include "mim_control/dynamic_graph/rw_lqr_controller.hpp"
#include "mim_control/dynamic_graph/rw_pd_controller.hpp"

namespace dg = dynamicgraph;

typedef bp::return_value_policy<bp::reference_existing_object>
    reference_existing_object;

BOOST_PYTHON_MODULE(wbc)
{
    bp::import("dynamic_graph");

    using mim_control::dynamic_graph::ImpedanceController;
    dynamicgraph::python::exposeEntity<ImpedanceController>().def(
        "initialize",
        +[](ImpedanceController& ImpedanceController,
            const boost::python::object& pinocchio_model,
            const std::string& root_frame_name,
            const std::string& end_frame_name) {
            const pinocchio::Model& pinocchio_model_ref =
                boost::python::extract<const pinocchio::Model&>(
                    pinocchio_model);
            ImpedanceController.initialize(
                pinocchio_model_ref, root_frame_name, end_frame_name);
            return;
        },
        "Initialize the ImpedanceController.");

    using mim_control::dynamic_graph::RWImpedanceController;
    dynamicgraph::python::exposeEntity<RWImpedanceController>().def(
        "initialize",
        +[](RWImpedanceController& RWImpedanceController,
            const boost::python::object& pinocchio_model,
            const std::string& root_frame_name,
            const std::string& end_frame_name) {
            const pinocchio::Model& pinocchio_model_ref =
                boost::python::extract<const pinocchio::Model&>(
                    pinocchio_model);
            RWImpedanceController.initialize(
                pinocchio_model_ref, root_frame_name, end_frame_name);
            return;
        },
        "Initialize the RWImpedanceController.");

    using mim_control::dynamic_graph::LQRController;
    dynamicgraph::python::exposeEntity<LQRController>().def(
        "initialize",
        +[](LQRController& LQRController,
            const boost::python::object& pinocchio_model) {
            const pinocchio::Model& pinocchio_model_ref =
                boost::python::extract<const pinocchio::Model&>(
                    pinocchio_model);
            LQRController.initialize(
                pinocchio_model_ref);
            return;
        },
        "Initialize the LQRController.");

    using mim_control::dynamic_graph::RWLQRController;
    dynamicgraph::python::exposeEntity<RWLQRController>().def(
        "initialize",
        +[](RWLQRController& RWLQRController,
            const boost::python::object& pinocchio_model) {
            const pinocchio::Model& pinocchio_model_ref =
                boost::python::extract<const pinocchio::Model&>(
                    pinocchio_model);
            RWLQRController.initialize(
                pinocchio_model_ref);
            return;
        },
        "Initialize the RWLQRController.");

    using mim_control::dynamic_graph::RWPDController;
    dynamicgraph::python::exposeEntity<RWPDController>().def(
        "initialize",
        +[](RWPDController& RWPDController,
            const boost::python::object& pinocchio_model) {
            const pinocchio::Model& pinocchio_model_ref =
                boost::python::extract<const pinocchio::Model&>(
                    pinocchio_model);
            RWPDController.initialize(
                pinocchio_model_ref);
            return;
        },
        "Initialize the RWPDController.");

    using mim_control::dynamic_graph::CentroidalPDController;
    dynamicgraph::python::exposeEntity<CentroidalPDController>();

    using mim_control::dynamic_graph::CentroidalForceQPController;
    dynamicgraph::python::exposeEntity<CentroidalForceQPController>().def(
        "initialize",
        &CentroidalForceQPController::initialize,
        "Initialize the CentroidalForceQPController.");
}
