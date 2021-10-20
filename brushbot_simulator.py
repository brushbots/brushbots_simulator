from time import time
from math import atan2
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

class Position():
    def __init__(self):
        self.x = 0.0
        self.y = 0.0

class Orientation():
    def __init__(self):
        self.z = 0.0
        self.w = 0.0

class Pose():
    def __init__(self):
        self.position = Position()
        self.orientation = Orientation()

class BrushbotSimulator:
    def __init__(self):
        self.N_AGENTS = 20
        self.ID_ROBOTS = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,18,19,20,21]
        self.robot_poses = {x: Pose() for x in self.ID_ROBOTS}
        self.W = 1.0
        self.DIFF_DRIVE_GAIN = 512
        self.DT = 0.1

        # plot variables
        self.figure = []
        self.axes = []
        self.ROBOT_RADIUS = 0.03
        self.LED_RADIUS = 0.005
        self.ENVIRONMENT_DIMENSIONS = [1.0, 0.5] # width, height (rectangular environment)
        self.V_GAIN = 0.0001
        self.OMEGA_GAIN = 0.001
        # these timeouts are hardcoded to be equal to the actual Constants.TIMEOUT_MOTORS and Constants.TIMEOUT_LEDS used in the robot firmware
        # warning: there is no consistency check between the simulator and the firmware
        self.TIMEOUT_MOTORS = 1000
        self.TIMEOUT_LEDS = 1000
        self.last_time_update_plot_millis = int(round(time() * 1000))

        self.patches_main = []
        self.patches_led_left = []
        self.patches_led_right = []
        self.last_time_received_velocity_commands = {x: -1 for x in self.ID_ROBOTS}
        self.last_time_received_led_commands = {x: -1 for x in self.ID_ROBOTS}

        # initialize poses
        for i in range(self.N_AGENTS):
            id = self.ID_ROBOTS[i]
            self.robot_poses[id].position.x = -0.5*self.ENVIRONMENT_DIMENSIONS[0]+self.ENVIRONMENT_DIMENSIONS[0]*np.random.rand()
            self.robot_poses[id].position.y = -0.5*self.ENVIRONMENT_DIMENSIONS[1]+self.ENVIRONMENT_DIMENSIONS[1]*np.random.rand()
            th_i = 2*np.pi*np.random.rand()
            self.robot_poses[id].orientation.z = np.sin(th_i/2.0)
            self.robot_poses[id].orientation.w = np.cos(th_i/2.0)

        # robot input velocities and leds
        self.motor_commands_received = {x: 2*[0] for x in self.ID_ROBOTS}
        self.led_commands_received = {x: 6*[128] for x in self.ID_ROBOTS}

        self._initialize_visualization()

    def _initialize_visualization(self):
        """
        Initialize plot with self.N_AGENTS robots
        @return:
        """
        self.figure, self.axes = plt.subplots()

        # environment patch
        p_boundary = patches.Rectangle(np.array([-0.5*self.ENVIRONMENT_DIMENSIONS[0], -0.5*self.ENVIRONMENT_DIMENSIONS[1]]), self.ENVIRONMENT_DIMENSIONS[0], self.ENVIRONMENT_DIMENSIONS[1], fill=False)

        self.axes.add_patch(p_boundary)

        # robot patches
        for i in range(self.N_AGENTS):
            id = self.ID_ROBOTS[i]
            xy_i = np.array([self.robot_poses[id].position.x, self.robot_poses[id].position.y])
            th_i = 2.0 * atan2(self.robot_poses[id].orientation.z, self.robot_poses[id].orientation.w)

            p_main = patches.Circle(xy_i, radius=self.ROBOT_RADIUS, facecolor='k')
            p_led_left = patches.Circle(xy_i+0.9*(self.ROBOT_RADIUS-self.LED_RADIUS)*np.array([np.cos(th_i),np.sin(th_i)])+2*self.LED_RADIUS*np.array([np.sin(th_i),-np.cos(th_i)]), radius=self.LED_RADIUS, facecolor='gray')
            p_led_right = patches.Circle(xy_i+0.9*(self.ROBOT_RADIUS-self.LED_RADIUS)*np.array([np.cos(th_i),np.sin(th_i)])+2*self.LED_RADIUS*np.array([-np.sin(th_i),np.cos(th_i)]), radius=self.LED_RADIUS, facecolor='gray')

            self.patches_main.append(p_main)
            self.patches_led_left.append(p_led_left)
            self.patches_led_right.append(p_led_right)

            self.axes.add_patch(p_main)
            self.axes.add_patch(p_led_left)
            self.axes.add_patch(p_led_right)

        self.axes.set_xlim(-0.6*self.ENVIRONMENT_DIMENSIONS[0], 0.6*self.ENVIRONMENT_DIMENSIONS[0])
        self.axes.set_ylim(-0.6*self.ENVIRONMENT_DIMENSIONS[1], 0.6*self.ENVIRONMENT_DIMENSIONS[1])

        self.axes.set_axis_off()
        self.axes.axis('equal')

        plt.ion()
        plt.show()

    def _update_visualization(self):
        now_millis = int(round(time() * 1000))
        if now_millis - self.last_time_update_plot_millis >= self.DT:
            for i in range(self.N_AGENTS):
                id = self.ID_ROBOTS[i]
                xy_i = np.array([self.robot_poses[id].position.x, self.robot_poses[id].position.y])
                th_i = 2.0 * atan2(self.robot_poses[id].orientation.z, self.robot_poses[id].orientation.w)

                self.patches_main[i].center = xy_i
                self.patches_led_left[i].center = xy_i+0.9*(self.ROBOT_RADIUS-self.LED_RADIUS)*np.array([np.cos(th_i),np.sin(th_i)])+2*self.LED_RADIUS*np.array([-np.sin(th_i),np.cos(th_i)])
                self.patches_led_right[i].center = xy_i+0.9*(self.ROBOT_RADIUS-self.LED_RADIUS)*np.array([np.cos(th_i),np.sin(th_i)])+2*self.LED_RADIUS*np.array([np.sin(th_i),-np.cos(th_i)])
                self.patches_led_left[i].set_facecolor(tuple([c/255.0 for c in self.led_commands_received[id][0:3]]))
                self.patches_led_right[i].set_facecolor(tuple([c/255.0 for c in self.led_commands_received[id][3:]]))

            self.figure.canvas.draw_idle()
            self.figure.canvas.flush_events()
        self.last_time_update_plot_millis = now_millis

    def get_poses(self):
        return pose2uni(self.robot_poses)

    def step(self):
        for i in range(self.N_AGENTS):
            id = self.ID_ROBOTS[i]
            th_i = 2.0 * atan2(self.robot_poses[id].orientation.z, self.robot_poses[id].orientation.w)

            v_i = self.V_GAIN * (self.motor_commands_received[id][0] + self.motor_commands_received[id][1]) / 2.0
            omega_i = self.OMEGA_GAIN * (self.motor_commands_received[id][1] - self.motor_commands_received[id][0]) / self.W

            self.robot_poses[id].position.x += v_i * np.cos(th_i) * self.DT
            self.robot_poses[id].position.y += v_i * np.sin(th_i) * self.DT
            th_i += omega_i * self.DT
            self.robot_poses[id].orientation.z = np.sin(th_i/2.0)
            self.robot_poses[id].orientation.w = np.cos(th_i/2.0)

            # boundary effect
            if self.robot_poses[id].position.x > 0.5 * self.ENVIRONMENT_DIMENSIONS[0] - self.ROBOT_RADIUS:
                self.robot_poses[id].position.x = 0.5 * self.ENVIRONMENT_DIMENSIONS[0] - self.ROBOT_RADIUS
            if self.robot_poses[id].position.x < - 0.5 * self.ENVIRONMENT_DIMENSIONS[0] + self.ROBOT_RADIUS:
                self.robot_poses[id].position.x = - 0.5 * self.ENVIRONMENT_DIMENSIONS[0] + self.ROBOT_RADIUS
            if self.robot_poses[id].position.y > 0.5 * self.ENVIRONMENT_DIMENSIONS[1] - self.ROBOT_RADIUS:
                self.robot_poses[id].position.y = 0.5 * self.ENVIRONMENT_DIMENSIONS[1] - self.ROBOT_RADIUS
            if self.robot_poses[id].position.y < - 0.5 * self.ENVIRONMENT_DIMENSIONS[1] + self.ROBOT_RADIUS:
                self.robot_poses[id].position.y = - 0.5 * self.ENVIRONMENT_DIMENSIONS[1] + self.ROBOT_RADIUS

        self._update_visualization()

    def set_velocities_and_leds(self, vel, leds):
        motor_commands_received = self.uni_to_motor_commands(vel[:, 0], vel[:, 1])

        for i, id in enumerate(self.ID_ROBOTS):
            self.motor_commands_received[id] = motor_commands_received[i,:]
            self.led_commands_received[id] = leds[id]

    def uni_to_motor_commands(self, v, omega):
        """

        @param omega: (N X 1) float array: angular velocity command
        @param v: (N X 1) float array: linear velocity command
        @return: (N X 2) float array: left and right motor commands (velocities)
        """
        N = len(v)
        one_to_N = np.array(range(N))

        mc = np.zeros((N, 2))
        mc[:, 0] = v - self.W / 2 * omega  # vl
        mc[:, 1] = v + self.W / 2 * omega  # vr

        v_positive = np.where(v > 0)
        v_negative = np.setdiff1d(one_to_N, v_positive)
        omega_positive = np.where(omega > 0)
        omega_negative = np.setdiff1d(one_to_N, omega_positive)
        vl_negative = np.where(mc[:, 0] < 0)
        vr_negative = np.where(mc[:, 1] < 0)

        v[np.intersect1d(v_positive, vl_negative)] = self.W / 2 * omega[np.intersect1d(v_positive, vl_negative)]
        v[np.intersect1d(v_positive, vr_negative)] = -self.W / 2 * omega[np.intersect1d(v_positive, vr_negative)]
        v[np.intersect1d(v_negative, omega_positive)] = self.W / 2 * omega[np.intersect1d(v_negative, omega_positive)]
        v[np.intersect1d(v_negative, omega_negative)] = -self.W / 2 * omega[np.intersect1d(v_negative, omega_negative)]

        mc[:, 0] = np.minimum(700, np.maximum(0, self.DIFF_DRIVE_GAIN * (v - self.W / 2 * omega)))  # vl
        mc[:, 1] = np.minimum(700, np.maximum(0, self.DIFF_DRIVE_GAIN * (v + self.W / 2 * omega)))  # vr

        return mc

def pose2uni(pose_array):
    """
    Converts geometry_msgs/Pose[]  (x,y,z with quaternions) into unicycle brushbot states
    """
    uni_state = np.zeros((len(pose_array), 3))
    for ii, key in enumerate(list(pose_array)):
        pose = pose_array[key]
        uni_state[ii, :2] = np.array([[pose.position.x, pose.position.y]])
        uni_state[ii, 2] = 2.0 * atan2(pose.orientation.z, pose.orientation.w)
    return uni_state
