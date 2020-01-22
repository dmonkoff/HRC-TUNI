#!/usr/bin/env python
import numpy as np
import math as m

# https://www.slideshare.net/RyanKeating13/ur5-ik
class ur5:
    def __init__(self):
        self.a = [0, -0.425, -0.39225, 0, 0, 0]
        self.d = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823]
        self.alpha = [m.pi / 2, 0, 0, m.pi / 2, -m.pi / 2, 0]

        # extra links for the corrected forearm link
        self.upper_arm_link = self.forearm_link = self.onrobot_rg2_link = np.eye(4)
        self.upper_arm_link[2, 3] = self.forearm_link[2, 3] = 0.136
        self.onrobot_rg2_link[2, 3] = 0.186
        self._joint_order = ('shoulder_pan_joint',
                             'shoulder_lift_joint',
                             'elbow_joint',
                             'wrist_1_joint',
                             'wrist_2_joint',
                             'wrist_3_joint')

    def DH(self, a, alpha, d, theta):
        dh = np.array([
            [m.cos(theta), -m.sin(theta) * m.cos(alpha), m.sin(theta) * m.sin(alpha), a * m.cos(theta)],
            [m.sin(theta), m.cos(theta) * m.cos(alpha), -m.cos(theta) * m.sin(alpha), a * m.sin(theta)],
            [0, m.sin(alpha), m.cos(alpha), d],
            [0, 0, 0, 1]
        ])
        for i in range(np.shape(dh)[0]):
            for j in range(np.shape(dh)[1]):
                if abs(dh[i, j]) < 0.0001:
                    dh[i, j] = 0.0
        return dh

    def sort_joints(self, joints_values, joint_names):
        sorted = np.zeros(6)
        for idx, i in enumerate(self._joint_order):
            sorted[idx] = joints_values[joint_names.index(self._joint_order[idx])]
        return sorted

    def fwd_kin(self, joints, gripper=None):
        T01 = self.DH(self.a[0], self.alpha[0], self.d[0], joints[0])
        T12 = self.DH(self.a[1], self.alpha[1], self.d[1], joints[1])
        T23 = self.DH(self.a[2], self.alpha[2], self.d[2], joints[2])
        T34 = self.DH(self.a[3], self.alpha[3], self.d[3], joints[3])
        T45 = self.DH(self.a[4], self.alpha[4], self.d[4], joints[4])
        T56 = self.DH(self.a[5], self.alpha[5], self.d[5], joints[5])
        if gripper is None:
            return np.dot(np.dot(np.dot(np.dot(np.dot(T01, T12), T23), T34), T45), T56)
        elif gripper == "onrobot_rg2":
            return np.dot(np.dot(np.dot(np.dot(np.dot(np.dot(T01, T12), T23), T34), T45), T56), self.onrobot_rg2_link)

    def get_translation(self, T):
        return T[0:3, 3]

    def get_link_xyz(self, joints):
        T01 = self.DH(self.a[0], self.alpha[0], self.d[0], joints[0])
        T12 = self.DH(self.a[1], self.alpha[1], self.d[1], joints[1])
        T23 = self.DH(self.a[2], self.alpha[2], self.d[2], joints[2])
        T34 = self.DH(self.a[3], self.alpha[3], self.d[3], joints[3])
        T45 = self.DH(self.a[4], self.alpha[4], self.d[4], joints[4])
        T56 = self.DH(self.a[5], self.alpha[5], self.d[5], joints[5])
        T02 = np.dot(T01, T12)
        T03 = np.dot(T02, T23)
        T04 = np.dot(T03, T34)
        T05 = np.dot(T04, T45)
        T06 = np.dot(T05, T56)
        T_upper_arm_link = np.dot(T01, self.upper_arm_link)
        T_forearm_link = np.dot(T02, self.forearm_link)

        output = np.zeros(shape=(9, 3))
        output[1] = self.get_translation(T01)
        output[2] = self.get_translation(T_upper_arm_link)
        output[3] = self.get_translation(T_forearm_link)
        output[4] = self.get_translation(T02)
        output[5] = self.get_translation(T03)
        output[6] = self.get_translation(T04)
        output[7] = self.get_translation(T05)
        output[8] = self.get_translation(T06)
        return output