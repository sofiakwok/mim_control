"""LQR controller.

License BSD-3-Clause
Copyright (c) 2021, New York University and Max Planck Gesellschaft.

Author: Sofia Kwok
Date:   June 4, 2024
"""

import numpy as np

from dg_tools.utils import (
    constVector,
    constVectorOp,
    subtract_vec_vec,
    hom2pos,
    add_vec_vec,
    stack_two_vectors,
    selec_vector,
    zero_vec,
    basePoseQuat2PoseRPY,
    multiply_mat_vec
)
import dynamic_graph as dg
import dynamic_graph.sot.dynamic_pinocchio as dp

from dynamic_graph.sot.core.math_small_entities import Add_of_vector

import mim_control.dynamic_graph.wbc as lqr_control_dg


class RWLQRController:
    def __init__(
        self,
        prefix,
        pin_robot,
        endeff_names,
    ):
        self.prefix = prefix
        self.pin_robot = pin_robot
        self.nv = pin_robot.model.nv
        self.ne = len(endeff_names)

        self.dg_robot = dp.DynamicPinocchio(prefix + "_pinocchio")
        self.dg_robot.setModel(self.pin_robot.model)
        self.dg_robot.setData(self.pin_robot.data)

        self.w_com_ff_sin, self.w_com_ff_sout = constVectorOp(np.zeros(6))
        print("initialized lqr_graph variables")
        # LQR Controller
        self.lqr_ctrl = lqr_control_dg.RWLQRController(prefix + "_lqr_controller")
        print("created RWLQRController")
        self.lqr_ctrl.initialize(pin_robot.model)
        print("initialized pinmodel RWLQRController")

        # The final computed control.
        # self.joint_torques_sout = constVector(np.zeros(6))
        self.joint_torques_sout = self.lqr_ctrl.joint_torque_sout

        ###
        # Export all the signals for the user of the PyEntity.
        self.des_robot_configuration_sin = self.lqr_ctrl.des_robot_configuration_sin
        self.des_robot_velocity_sin = self.lqr_ctrl.des_robot_velocity_sin
        print("done initializing lqr_graph")

    def trace(self, robot=None):
        print("lqr trace robot: " + str(robot))
        if robot is None:
            try:
                robot = self.robot
            except:
                print("RWLQRController.trace(): No robot given, cannot trace the data.")
                return

        robot.add_trace(self.prefix + '_q', 'sout')
        robot.add_trace(self.prefix + '_dq', 'sout')

        # for eff_name, imp in zip(self.endeff_names, self.imps):
        #     # Actual position of the endeffector.
        #     robot.add_trace(self.prefix + '_pos_' + eff_name, 'sout')
        #     # Desired position of the endeffector.
        #     robot.add_trace(imp.name, 'desired_end_frame_placement_sin')
        
        robot.add_trace(self.lqr_ctrl.name, "torque_sout")
        robot.add_trace(self.lqr_ctrl.name, "joint_torque_sout")
        robot.add_trace(self.lqr_ctrl.name, "robot_configuration_sin")
        robot.add_trace(self.lqr_ctrl.name, "robot_velocity_sin")
        robot.add_trace(self.lqr_ctrl.name, "des_robot_configuration_sin")
        robot.add_trace(self.lqr_ctrl.name, "des_robot_velocity_sin")

    def plug(self, robot, base_position, base_velocity):
        self.plug_all_signals(
            robot.device.joint_positions,
            robot.device.joint_velocities,
            base_position,
            base_velocity,
            robot.device.ctrl_joint_torques
        )
    
    def plug_all_signals(
        self,
        joint_positions_sout,
        joint_velocities_sout,
        base_position_sout,
        base_velocity_sout,
        ctrl_joint_torque_sin
    ):
        # Args:
        #   robot; DGM robot device
        #   base_position: The base position as a 7 dim vector signal
        #   base_velocity: The base velocity as a 6 dim vector signal

        # Create the input to the dg_robot.
        base_pose_rpy = basePoseQuat2PoseRPY(base_position_sout)

        position = stack_two_vectors(
            base_pose_rpy, joint_positions_sout, 6, self.nv - 6
        )
        velocity = stack_two_vectors(
            base_velocity_sout, joint_velocities_sout, 6, self.nv - 6,
            self.prefix + '_dq'
        )

        dg.plug(position, self.dg_robot.signal("position"))
        dg.plug(velocity, self.dg_robot.signal("velocity"))
        self.dg_robot.signal("acceleration").value = np.array(
            self.nv
            * [
                0.0,
            ]
        )

        print("plugged to dg_robot")

        # create inputs to lqr controller
        position = stack_two_vectors(
            base_position_sout, joint_positions_sout, 7, self.nv - 6,
            self.prefix + '_q'
        )

        dg.plug(position, self.lqr_ctrl.robot_configuration_sin)
        dg.plug(velocity, self.lqr_ctrl.robot_velocity_sin)

        # Finally, plug the computed torques to the output.
        print("graph joint torques: " + str(self.joint_torques_sout.value))
        dg.plug(self.joint_torques_sout, ctrl_joint_torque_sin)

    def plug_base_as_com(self, base_position, base_velocity_world):
        """ Instead of the COM use the base as com. """
        dg.plug(
            selec_vector(base_position, 0, 3),
            self.lqr_ctrl.actual_com_position_sin
        )

        dg.plug(
            selec_vector(base_velocity_world, 0, 3),
            self.lqr_ctrl.actual_com_velocity_sin
        )


