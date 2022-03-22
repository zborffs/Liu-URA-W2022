import numpy as np
from numpy import pi
from functools import partial
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import control


class PathPlanner:
    @staticmethod
    def plan_path(start: np.ndarray, end: np.ndarray) -> np.ndarray:
        """
        we can make this more generic later, but for now we accept and return single floats representing positions
        :param start:
        :param end:
        :return:
        """
        return np.array([end])  # don't include start!


class FeedbackController:
    e_k_1 = 0.0
    e_k = 0.0
    u_k_1 = 0.0
    T_k_1 = 0.01
    T_k = 0.01

    def __init__(self, n, m: float, l: float, d: float, g: float):
        self.n = n
        self.theta_bar = np.arange(0, 2*pi, 2 * pi / n)
        self.model = np.zeros((n, 3))  # 3 is specific to this pendulum model!
        ml_squared = m * l ** 2
        for i in range(0, n):
            self.model[i] = np.array([-d / ml_squared, g / l * np.cos(self.theta_bar[i]), 1 / ml_squared])

    def get_region_index(self, theta):
        """
        gets the current state value of theta (i.e. the angle of the pendulum) and returns the region it is in by index
        :param theta:
        :return:
        """
        theta = np.where(theta > pi, theta - 2 * pi, theta)
        if np.cos(theta) >= 0:
            # we are in the up position, so return the index of the upper dynamics (i.e. 0)
            # print('upper')
            return 0
        else:
            # we are in the down position, so return the index of the lower dynamics (i.e. 1)
            # print('lower')
            return 1

    def approximate_dynamics(self, t, x, u):
        """
        linear approximation of the pendulum in different regions
        :param t:
        :param x:
        :param u:
        :return:
        """
        theta, thetadot = x

        region_index = self.get_region_index(theta)
        if region_index == 0:
            # upper position
            theta = np.where(theta > pi, theta - 2 * pi, theta)
        else:
            # lower position
            theta = np.mod(theta, 2 * pi)
        v = np.array([thetadot, theta - self.theta_bar[region_index], u])
        thetaddot = np.dot(self.model[region_index], v)
        return [
            thetadot,
            thetaddot
        ]

    def get_model(self):
        # self.model[i, 0] = a2
        # self.model[i, 1] = a3
        # self.model[i, 2] = a4
        # tf([a4], [1 -a2 -a3])
        # -> there are as many transfer-functions as there are self.model.shape[0]
        temp = []
        print(self.model.shape)
        for i in range(0, self.model.shape[0]):
            a2 = self.model[i, 0]
            a3 = self.model[i, 1]
            a4 = self.model[i, 2]
            transfer_function = control.tf([a4], [1, -a2, -a3])
            temp.append(transfer_function)
        return np.array(temp)

    def control(self, r_k, y_k, T_k):
        region_index = self.get_region_index(y_k)
        # K = 5.0
        if region_index == 0:
            # upper control
            # print('upper control')
            y_k = np.where(y_k > pi, y_k - 2 * pi, y_k)
            K = 4.5
        else:
            # lower control
            # print('lower control')
            y_k = np.mod(y_k, 2 * pi)
            K = -0.735
        return K * (r_k - y_k)


class Pendulum:
    line_actual = None
    circle_actual = None
    line_approx = None
    circle_approx = None
    time_text = None

    def __init__(self, m: float = 0.15, l: float = 0.5, d: float = 0.1, g: float = 9.81):
        """
        constructs the Pendulum model from model parameters as arguments
        :param m: mass at the end of the pendulum (in kg)
        :param l: length of the pendulum (in meters)
        :param d: damping coefficient of the pendulum
        :param g: acceleration due to gravity (in meters/second^2)
        """
        self.dim_n = 2
        self.dim_m = 1
        self.t = None
        self.x = np.zeros(self.dim_n)
        self.x_extended = np.zeros(2 * self.dim_n)
        self.waypoints = None

        # pendulum model parameters
        self.m = m
        self.l = l
        self.d = d
        self.g = g
        # self.Domain = [-2 * pi, 2 * pi, -2 * pi, 2 * pi]  # why should should the bound on rotational velocity (+/-) 2pi; should just be -pi to pi or 0 to 2*pi

        self.feedback_controller = FeedbackController(2, m, l, d, g)

    def integrate(self, x_initial: np.ndarray, x_goal: np.ndarray, t_interval: list[float, float]) -> None:
        """
        wrapper for the solve_ivp(...) integrator
        :param x_initial:
        :param x_goal:
        :param t_interval: the interval to simulate over (tuple_
        :return:
        """
        assert(len(t_interval) == 2)

        self.waypoints = PathPlanner.plan_path(x_initial, x_goal)

        self.x = x_initial
        self.x_extended = np.concatenate((self.x, self.x))
        dx_dt = partial(self.dynamics)
        sol = solve_ivp(dx_dt, t_interval, self.x_extended, method='RK45',
                        t_eval=np.linspace(t_interval[0], t_interval[1], num=500), rtol=1e-7, atol=1e-7,
                        dense_output=False, events=None, vectorized=False)
        self.t = sol.t
        self.x = sol.y

    def pop_waypoint(self) -> None:
        if self.waypoints.shape == () or self.waypoints.shape[0] == 0:
            return
        self.waypoints = np.delete(self.waypoints, (0), axis=0)

    def dynamics(self, t, x):
        """
        the differential equation update function [thetadot, thetaddot]
        :param t: current time
        :param x: the current state vector (2x1 vector, where first element is position of pend in rad, and second
        element if the rotational velocity of pend in rad/s
        :param u: the control input
        :return: 2x1 vector of the derivative of each state variable (derivative of first element is second element by
        definition, and the derivative of second element is equation of motion for the pendulum)
        """
        ml_squared = self.m * (self.l ** 2)
        theta, thetadot, theta_approx, thetadot_approx = x

        theta = np.where(theta > pi, theta - 2 * pi, theta)
        theta_approx = np.where(theta_approx > pi, theta_approx - 2 * pi, theta_approx)

        # check if we are satisfying the current waypoint
        eps = 0.001
        if np.abs(self.waypoints[0][0] - theta) < eps and np.abs(self.waypoints[0][1] - thetadot) < eps:
            # we met the waypoint, but don't remove it if it's the last waypoint
            if self.waypoints.shape[0] > 1:
                self.pop_waypoint()
        r = self.waypoints[0][0]  # assume the second waypoint is always 0

        u = self.feedback_controller.control(r, theta, 0.0)
        u_approx = self.feedback_controller.control(r, theta_approx, 0.0)
        # u = 0.0
        # u_approx = 0.0

        thetadot_approx, thetaddot_approx = self.feedback_controller.approximate_dynamics(t, np.array([theta_approx, thetadot_approx]), u_approx)

        return [
            thetadot,
            (self.g / self.l) * np.sin(theta) - (self.d / ml_squared) * thetadot + (1 / ml_squared) * u,
            thetadot_approx,
            thetaddot_approx
                ]

    def graph(self):
        plt.plot(self.t, self.x[0,:])
        plt.plot(self.t, self.x[1,:])
        plt.plot(self.t, self.x[2,:])
        plt.plot(self.t, self.x[3,:])
        plt.title('Pendulum Open-Loop')
        plt.xlabel('Time (s)')
        plt.legend(['theta (rad)', 'thetadot (rad/s)', 'approx. theta (rad)', 'approx. thetadot (rad/s)'], shadow=True)
        return plt

    def _update_animation(self, i):
        x_actual = self.l * np.sin(self.x[0,i])
        y_actual = self.l * np.cos(self.x[0,i])
        x_approx = self.l * np.sin(self.x[2,i])
        y_approx = self.l * np.cos(self.x[2,i])
        self.line_actual.set_data([0, x_actual], [0, y_actual])
        self.circle_actual.set_center((x_actual, y_actual))
        self.line_approx.set_data([0, x_approx], [0, y_approx])
        self.circle_approx.set_center((x_approx, y_approx))
        self.time_text.set_text('t = %.1fs' % self.t[i])

    def animate(self):
        fig = plt.figure()
        ax = fig.add_subplot(aspect='equal')
        x0_actual = self.l * np.sin(self.x[0,0])
        y0_actual = self.l * np.cos(self.x[0,0])
        x0_approx = self.l * np.sin(self.x[2,0])
        y0_approx = self.l * np.cos(self.x[2,0])

        self.line_actual, = ax.plot([0, x0_actual], [0, y0_actual], lw=1, c='k')
        self.circle_actual = ax.add_patch(plt.Circle((x0_actual, y0_actual), 0.03, fc='b', zorder=3))
        self.line_approx, = ax.plot([0, x0_approx], [0, y0_approx], lw=1, c='k', linestyle='--')
        self.circle_approx = ax.add_patch(plt.Circle((x0_approx, y0_approx), 0.03, fc='r', linestyle='--', zorder=3))
        self.time_text = ax.text(0.05, 0.9, 't=', transform=ax.transAxes)
        ax.set_xlim(-self.l * 2, self.l * 2)
        ax.set_ylim(-self.l * 2, self.l * 2)
        anim = animation.FuncAnimation(fig, self._update_animation, frames=len(self.t), repeat=True, interval=25)
        return anim
