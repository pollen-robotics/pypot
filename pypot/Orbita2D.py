#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import numpy as np
import argparse
import itertools

from pypot.dynamixel import DxlIO, get_available_ports
from pypot.dynamixel.protocol.v1 import DxlReadDataPacket, DxlSyncWritePacket
from pypot.dynamixel.conversion import dxl_code
from pypot.dynamixel import motor
from pypot.utils import trajectory

from reachy_sdk import ReachySDK
from reachy_sdk.trajectory import goto, goto_async
from reachy_sdk.trajectory.interpolation import InterpolationMode
from scipy.spatial.transform import Rotation as R

# from reachy_kinematics import arm_kinematics
# import PyKDL as kdl
# from reachy_kinematics.kdl_parser_py import urdf


class dxl_mt(object):
    def __init__(self, motor_ids, init_angles):
        self.motor_ids = motor_ids
        self.init_angles = init_angles
        self.motors_mt = {}
        for i, m in enumerate(self.motor_ids):
            self.motors_mt[m] = init_angles[i]

    def set_mt_position(self, pos_dict):
        for m, p in pos_dict.items():
            self.motors_mt[m] = p

    def get_mt_position(self, pos_dict, update=True):
        ret = {}
        for m, p in pos_dict.items():
            div = int(self.motors_mt[m])//360
            if self.motors_mt[m] < 0.0:  # the floor divide is rounded to towards the negative...
                div += 1

            ret[m] = p-(div)*360
        if update:
            self.set_mt_position(ret)
        return ret


def Orbita2D_ik(target, ratioA=1, ratioB=1):
    mat = np.array([[ratioA, ratioA], [ratioB, -ratioB]])
    return mat@np.array(target)


def ArmV2_kin(target, ratioA1=1, ratioB1=1, ratioA2=1, ratioB2=1):
    return np.concatenate([Orbita2D_ik(target[:2], ratioA=ratioA1, ratioB=ratioB1), Orbita2D_ik(target[2:], ratioA=ratioA2, ratioB=ratioB2)])


class Orbita2D(object):
    def __init__(self, ids, init_pos, ratioA, ratioB, offsets={}):

        self.dxl_mt = dxl_mt(ids, init_pos)
        self.ratioA = ratioA
        self.ratioB = ratioB
        self.mat = np.array([[self.ratioA, self.ratioA], [self.ratioB, -self.ratioB]])

        self.motor_offsets = {}
        for motor_id in ids:
            self.motor_offsets[motor_id] = 0.0
        for k, v in offsets.items():
            self.motor_offsets[k] = v

    # def set_offsets(self, motor_offsets):
    #     for k, v in motor_offsets.items():
    #         self.motor_offsets[k] = v

    def ik(self, target):

        ret = self.mat@np.array(np.radians(target))
        i = 0
        d = {}
        for k in self.dxl_mt.motors_mt.keys():
            d[k] = np.degrees(ret[i])+self.motor_offsets[k]
            # self.dxl_mt.get_mt_position({k:np.degrees(ret[i])})
            i += 1

        return self.dxl_mt.get_mt_position(d)

    def set_offsets(self, dxl):
        for id in self.dxl_mt.motor_ids:
            self.motor_offsets[id] = dxl.get_present_position([id])[0]


# def generate_solver(urdf_str: str):
#     """Create an FK/IK solvers for each arm (left/right)."""
#     success, urdf_tree = urdf.treeFromString(urdf_str)
#     if not success:
#         raise IOError('Could not parse the URDF!')

#     chain = urdf_tree.getChain('torso', 'left_tip')
#     fk_solver = kdl.ChainFkSolverPos_recursive(chain)

#     ik_solver = kdl.ChainIkSolverPos_LMA(
#         chain,
#         eps=1e-5,
#         maxiter=500,
#         eps_joints=1e-15,
#         L=np.array([1, 1, 1, 0.01, 0.01, 0.01])
#     )

#     return chain, fk_solver, ik_solver


# def ArmV2_ik(ik_solver, target_pose, ratioA1=1, ratioB1=1, ratioA2=1, ratioB2=1):
#     tpose = np.eye(4)
#     tpose[:3, 3] = np.array(target_pose)
#     target_j = arm_kinematics.inverse_kinematics(ik_solver, q0=np.array([0, 0, 0, 0.01]), target_pose=tpose, nb_joints=4)
#     return ArmV2_kin(target_j[1], ratioA1, ratioB1, ratioA2, ratioB2)


# def ArmV2_ik_rot(ik_solver, target_pose, ratioA1=1, ratioB1=1, ratioA2=1, ratioB2=1):
#     # tpose=np.eye(4)
#     # tpose[:3,3]=np.array(target_pose)
#     tpose = np.array(target_pose)
#     target_j = arm_kinematics.inverse_kinematics(ik_solver, q0=np.array([0, 0, 0, 0.01]), target_pose=tpose, nb_joints=4)
#     return ArmV2_kin(target_j[1], ratioA1, ratioB1, ratioA2, ratioB2)


def get_matrix(x, y, z, roll, pitch, yaw):
    '''
    return a transformation matrix
    '''
    rot = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()

    mat = np.matrix(np.zeros(4*4).reshape((4, 4)))
    mat[0:3, 0:3] = rot

    t = np.array([x, y, z, 1])
    t = t.reshape(4, 1)
    mat[:, 3] = t

    return np.array(mat)


def goto_cart(arm, solver, x, y, z, roll=0, pitch=0, yaw=0, duration=2):
    mat = get_matrix(x, y, z, roll, pitch, yaw)
    #p = arm.l_arm.inverse_kinematics(mat)
    p = ArmV2_ik_rot(solver, mat)
    # print(p)
    goto({joint: pos for joint, pos in zip(
        arm.joints.values(), np.degrees(p))}, duration=duration)


class DummyJoint(object):
    def __init__(self, dxl, motor_id):
        ''' quick and dirty hack...'''
        self.dxl = dxl
        self.motor_id = motor_id
        self._present_position = dxl.get_present_position([motor_id])
        self._goal_position = dxl.get_goal_position([motor_id])

    @property
    def present_position(self):
        self._present_position = self.dxl.get_present_position([self.motor_id])
        return self._present_position[0]

    @property
    def goal_position(self):
        self._goal_position = self.dxl.get_goal_position([self.motor_id])
        return self._goal_position

    @goal_position.setter
    def goal_position(self, g):
        self._goal_position = g
        self.dxl.set_goal_position({self.motor_id: g})


def orb2D_goto(dxl, orb, goal, duration=3):
    j1 = DummyJoint(dxl, orb.dxl_mt.motor_ids[0])
    j2 = DummyJoint(dxl, orb.dxl_mt.motor_ids[1])

    joints = {orb.dxl_mt.motor_ids[0]: j1, orb.dxl_mt.motor_ids[1]: j2}

    # p = goal
    # g = orb.ik(p[:2])

    g = orb.ik(goal)

    trajs = []
    for motor_id, joint in joints.items():
        trajs.append(trajectory.GotoMinJerk(joint, g[motor_id], duration))

    for t in trajs:
        t.start()


def torque_on(dxl, orbs):
    for orb in orbs:
        dxl.enable_torque(orb.dxl_mt.motor_ids)


def torque_off(dxl, orbs):
    for orb in orbs:
        dxl.disable_torque(orb.dxl_mt.motor_ids)
