from mujoco_py import load_model_from_path, MjSim, MjViewer, cymj
from mujoco_py.generated import const
import os
import time
import sympy as sm
import numpy as np


class DPendulum:
    """This class defines a two link arm which can be controlled using a jacobian controller"""
    def __init__(self, sim, model, l1=1, l2=1, theta1=3.14*0.75, theta2=-3.14/2):
        # simulation variables
        self.model = model
        self.sim = sim
        # initial position
        self.theta1_d = theta1
        self.theta2_d = theta2
        # length of the links
        self.l1 = l1
        self.l2 = l2
        self.compute_jacobian()
        # apply initial perturbation
        self.sim.data.qpos[0] = theta1
        self.sim.data.qpos[1] = theta2
        # initial time
        self.time = self.sim.data.time

    def compute_jacobian(self):
        """
        Computes the jacobian of the two link arm (computation performed only once)
        :return:
        """
        # define symbols
        self.theta1_sym = sm.Symbol('theta1')
        self.theta2_sym = sm.Symbol('theta2')
        # kinematic equations
        x = self.l1 * sm.cos(self.theta1_sym) + self.l2 * sm.cos(self.theta1_sym + self.theta2_sym)
        y = self.l1 * sm.sin(self.theta1_sym) + self.l2 * sm.sin(self.theta1_sym + self.theta2_sym)
        # jacobian matrix
        a11 = sm.diff(x, self.theta1_sym)
        a12 = sm.diff(x, self.theta2_sym)
        a21 = sm.diff(y, self.theta1_sym)
        a22 = sm.diff(y, self.theta2_sym)
        J = sm.Matrix([[a11, a12], [a21, a22]])
        self.J = sm.simplify(J)

    def evaluate_inv_jacobian(self, theta1, theta2):
        J_inv = self.J.subs({self.theta1_sym: theta1, self.theta2_sym: theta2}).pinv()
        return np.array(J_inv).astype(np.float)

    def controller_low_level(self):
        # torque actuator (PD Control using noisy data) + gravity compensation
        self.sim.data.ctrl[0] = -100. * (self.sim.data.sensordata[0] - self.theta1_d) \
                                -10.0 * self.sim.data.sensordata[1] \
                                + self.sim.data.qfrc_bias[0]
        self.sim.data.ctrl[1] = -100. * (self.sim.data.sensordata[2] - self.theta2_d) \
                                -10.0 * self.sim.data.sensordata[3] + \
                                + self.sim.data.qfrc_bias[1]

    def controller_high_level(self):
        # TODO: define curve using key poses
        v1_ref = 0.#-2 * np.sin(2 * 3.14 * 1 *  self.time)
        v2_ref = 2. * np.cos(2 * 3.14 * 1 *  self.time) # .5 * np.cos(self.time)
        print("Target velocity : ({}, {})".format(v1_ref, v2_ref))
        # evalutae jacobian
        J = self.evaluate_inv_jacobian(self.theta1_d, self.theta2_d)#(self.sim.data.sensordata[0], self.sim.data.sensordata[2])
        # compute desired joint velocity
        dq1_ref, dq2_ref = list(J.dot(np.array([v1_ref, v2_ref])))
        self.integrator(dq1_ref, dq2_ref)
        print("Time: {} Jacobian: {}".format(self.time, J))


    def integrator(self, dq1, dq2):
        dt = (self.sim.data.time - self.time)
        self.time = self.sim.data.time
        #print("dt is {}".format(dt))
        self.theta1_d = self.theta1_d + dt * dq1
        self.theta2_d = self.theta2_d + dt * dq2
        print("Desired joint positions: theta1 = {}, theta2 = {}.".format(self.theta1_d, self.theta2_d))

    def step(self):
        self.controller_high_level()
        self.controller_low_level()


def main():
    model = load_model_from_path("models/two_link_pendulum.xml")
    sim = MjSim(model)
    viewer = MjViewer(sim)
    sim_state = sim.get_state()

    #cymj.set_pid_control(sim.model, sim.data)

    dpendulum = DPendulum(sim, model)

    while True:
        sim.step()
        viewer.render()
        dpendulum.step()


main()
