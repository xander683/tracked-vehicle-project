#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator, AutoMinorLocator
from matplotlib import rcParams
from gazebo_msgs.srv import GetModelState

rcParams['font.size'] = 12
rcParams['axes.titlesize'] = 14
rcParams['axes.labelsize'] = 12
rcParams['legend.fontsize'] = 10
rcParams['figure.dpi'] = 150


def fine_grid(ax, x_major=None, x_minor=None, y_major=None, y_minor=None):
    if x_major is not None:
        ax.xaxis.set_major_locator(MultipleLocator(x_major))
    if x_minor is not None:
        ax.xaxis.set_minor_locator(MultipleLocator(x_minor))
    else:
        ax.xaxis.set_minor_locator(AutoMinorLocator(5))
    if y_major is not None:
        ax.yaxis.set_major_locator(MultipleLocator(y_major))
    if y_minor is not None:
        ax.yaxis.set_minor_locator(MultipleLocator(y_minor))
    else:
        ax.yaxis.set_minor_locator(AutoMinorLocator(5))
    ax.grid(True, which='major', alpha=0.4, linewidth=0.8)
    ax.grid(True, which='minor', alpha=0.15, linewidth=0.4)


class ExperimentRecorder:

    def __init__(self):
        rospy.init_node('experiment_recorder', anonymous=True)

        self.A = rospy.get_param('/pp_pid_driver/amplitude',
                    rospy.get_param('/s_trajectory_driver/amplitude', 0.6))
        self.L = rospy.get_param('/pp_pid_driver/wavelength',
                    rospy.get_param('/s_trajectory_driver/wavelength', 5.0))
        self.lead_in = rospy.get_param('/pp_pid_driver/lead_in',
                    rospy.get_param('/s_trajectory_driver/lead_in', 1.5))
        self.x_end = rospy.get_param('/pp_pid_driver/path_length',
                    rospy.get_param('/s_trajectory_driver/path_length', 11.5))
        self.speed = rospy.get_param('/pp_pid_driver/speed',
                    rospy.get_param('/s_trajectory_driver/speed', 0.3))
        self.gap = rospy.get_param('/pp_pid_driver/desired_distance',
                    rospy.get_param('/s_trajectory_driver/desired_distance', 0.55))

        self.vehicles = ['vehicle_1', 'vehicle_2', 'vehicle_3']
        self.colors = {'vehicle_1': '#D62728', 'vehicle_2': '#1F77B4',
                       'vehicle_3': '#2CA02C'}
        self.styles = {'vehicle_1': '-', 'vehicle_2': '--',
                       'vehicle_3': '-.'}
        self.labels = {'vehicle_1': 'Vehicle 1 (Leader)',
                       'vehicle_2': 'Vehicle 2 (Follower 1)',
                       'vehicle_3': 'Vehicle 3 (Follower 2)'}

        self.data = {v: {'t': [], 'x': [], 'y': [], 'yaw': []} for v in self.vehicles}

        rospy.wait_for_service('/gazebo/get_model_state', timeout=30)
        self.get_state = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState, persistent=True)

        rospy.loginfo('[Recorder] Recording data at 20 Hz...')
        self.t0 = rospy.Time.now()
        rate = rospy.Rate(20)
        stopped_count = 0

        while not rospy.is_shutdown():
            t = (rospy.Time.now() - self.t0).to_sec()
            if t < 0.5:
                rate.sleep()
                continue

            for v in self.vehicles:
                try:
                    resp = self.get_state(v, 'world')
                    if resp.success:
                        p = resp.pose.position
                        q = resp.pose.orientation
                        yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)
                        self.data[v]['t'].append(t)
                        self.data[v]['x'].append(p.x)
                        self.data[v]['y'].append(p.y)
                        self.data[v]['yaw'].append(yaw)
                except Exception:
                    pass

            if len(self.data['vehicle_1']['x']) > 40:
                x1 = self.data['vehicle_1']['x'][-1]
                x1_prev = self.data['vehicle_1']['x'][-20]
                if abs(x1 - x1_prev) < 0.005 and x1 > 2.0:
                    stopped_count += 1
                else:
                    stopped_count = 0
                if stopped_count > 60:
                    rospy.loginfo('[Recorder] Trajectory complete')
                    break

            if t > 120:
                rospy.loginfo('[Recorder] Timeout')
                break
            rate.sleep()

        n = len(self.data['vehicle_1']['t'])
        rospy.loginfo('[Recorder] Recording done, %d data points', n)
        self._plot_all()

    @staticmethod
    def _quat_to_yaw(x, y, z, w):
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny, cosy)

    def _ref_path(self, xs):
        ys = []
        for x in xs:
            if x < self.lead_in:
                ys.append(0.0)
            else:
                ys.append(self.A * math.sin(2.0 * math.pi *
                          (x - self.lead_in) / self.L))
        return ys

    def _compute_velocity(self, v):
        t = np.array(self.data[v]['t'])
        x = np.array(self.data[v]['x'])
        y = np.array(self.data[v]['y'])
        yaw = np.array(self.data[v]['yaw'])

        dt = np.diff(t)
        dt[dt < 1e-6] = 1e-6

        vlin = np.sqrt(np.diff(x)**2 + np.diff(y)**2) / dt

        dyaw = np.diff(yaw)
        dyaw = (dyaw + np.pi) % (2 * np.pi) - np.pi
        omega = dyaw / dt

        k_v = np.ones(31) / 31.0
        k_w = np.ones(11) / 11.0
        if len(vlin) > 31:
            vlin = np.convolve(vlin, k_v, mode='same')
        if len(omega) > 11:
            omega = np.convolve(omega, k_w, mode='same')

        t_mid = (t[:-1] + t[1:]) / 2.0
        return t_mid, vlin, omega

    def _nearest_path_error(self, xs, ys):
        ref_xs = np.linspace(-1.0, self.x_end + 1.0, 5000)
        ref_ys = np.array(self._ref_path(ref_xs))

        errors_mm = []
        for px, py in zip(xs, ys):
            dists = np.sqrt((ref_xs - px)**2 + (ref_ys - py)**2)
            idx = np.argmin(dists)
            min_dist = dists[idx]
            rx, ry = ref_xs[idx], ref_ys[idx]
            if idx < len(ref_xs) - 1:
                tx = ref_xs[idx + 1] - rx
                ty = ref_ys[idx + 1] - ry
            else:
                tx = rx - ref_xs[idx - 1]
                ty = ry - ref_ys[idx - 1]
            cross = tx * (py - ry) - ty * (px - rx)
            sign = 1.0 if cross >= 0 else -1.0
            errors_mm.append(sign * min_dist * 1000.0)

        return np.array(errors_mm)

    def _plot_all(self):
        out = '/home/xander/partime/ws_tracked/'
        rospy.loginfo('[Recorder] Generating plots...')

        fig, ax = plt.subplots(figsize=(11, 5))

        ref_x = np.linspace(-0.5, self.x_end + 0.5, 500)
        ref_y = self._ref_path(ref_x)
        ax.plot(ref_x, ref_y, 'k--', linewidth=1.2, alpha=0.5,
                label='Reference Path', zorder=1)

        for v in self.vehicles:
            xd = self.data[v]['x']
            yd = self.data[v]['y']
            if not xd:
                continue
            ax.plot(xd, yd, color=self.colors[v], linewidth=1.8,
                    label=self.labels[v], zorder=2)
            ax.plot(xd[0], yd[0], 'o', color=self.colors[v],
                    markersize=8, zorder=3)
            ax.plot(xd[-1], yd[-1], 's', color=self.colors[v],
                    markersize=8, zorder=3)

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Vehicle Trajectories (Circle=Start, Square=End)')
        ax.legend(loc='upper left')
        ax.set_aspect('equal')
        fine_grid(ax, x_major=1.0, x_minor=0.25, y_major=0.2, y_minor=0.05)
        fig.tight_layout()
        fig.savefig(out + 'plot_1_trajectory.png')
        plt.close(fig)
        rospy.loginfo('[Recorder]   -> plot_1_trajectory.png')

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

        for v in self.vehicles:
            t = self.data[v]['t']
            short = self.labels[v].split('(')[0].strip()
            ax1.plot(t, self.data[v]['x'], color=self.colors[v],
                     linewidth=1.5, label=short)
            ax2.plot(t, self.data[v]['y'], color=self.colors[v],
                     linewidth=1.5, label=short)

        ax1.set_ylabel('X Position (m)')
        ax1.set_title('X Coordinate vs Time')
        ax1.legend(loc='upper left')
        fine_grid(ax1, x_major=5.0, x_minor=1.0, y_major=2.0, y_minor=0.5)

        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Y Position (m)')
        ax2.set_title('Y Coordinate vs Time')
        ax2.legend(loc='lower left')
        fine_grid(ax2, x_major=5.0, x_minor=1.0, y_major=0.2, y_minor=0.05)

        fig.tight_layout()
        fig.savefig(out + 'plot_2_xy_vs_time.png')
        plt.close(fig)
        rospy.loginfo('[Recorder]   -> plot_2_xy_vs_time.png')

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

        startup = 8.0

        for v in self.vehicles:
            t_mid, vlin, _ = self._compute_velocity(v)
            mask = t_mid > startup + 2.0
            t_c = t_mid[mask]
            vlin_c = vlin[mask]
            short = self.labels[v].split('(')[0].strip()
            ls = self.styles[v]
            ax1.plot(t_c, vlin_c, color=self.colors[v], linestyle=ls,
                     linewidth=1.8, label=short)
        ax1.axhline(y=self.speed, color='k', linestyle='--', linewidth=1.0,
                     alpha=0.6, label='Target ({:.2f} m/s)'.format(self.speed))
        ax1.set_ylabel('Linear Velocity (m/s)')
        ax1.set_title('Linear Velocity vs Time (PP+PID cmd_vel)')
        ax1.legend(loc='lower right')
        fine_grid(ax1, x_major=5.0, x_minor=1.0, y_major=0.05, y_minor=0.01)

        for v in self.vehicles:
            t_mid, _, omega = self._compute_velocity(v)
            mask = t_mid > startup + 2.0
            t_c = t_mid[mask]
            omega_c = omega[mask]
            short = self.labels[v].split('(')[0].strip()
            ls = self.styles[v]
            ax2.plot(t_c, omega_c, color=self.colors[v], linestyle=ls,
                     linewidth=2.0, label=short)

        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Angular Velocity (rad/s)')
        ax2.set_title('Angular Velocity vs Time')
        ax2.legend(loc='upper right')
        fine_grid(ax2, x_major=5.0, x_minor=1.0, y_major=0.1, y_minor=0.02)

        fig.tight_layout()
        fig.savefig(out + 'plot_3_velocity.png')
        plt.close(fig)
        rospy.loginfo('[Recorder]   -> plot_3_velocity.png')

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

        pairs = [('vehicle_1', 'vehicle_2', ax1,
                  'Vehicle 2 Following Vehicle 1'),
                 ('vehicle_2', 'vehicle_3', ax2,
                  'Vehicle 3 Following Vehicle 2')]

        startup = 8.0
        for leader, follower, ax, title in pairs:
            tl = np.array(self.data[leader]['t'])
            xl = np.array(self.data[leader]['x'])
            yl = np.array(self.data[leader]['y'])
            tf = np.array(self.data[follower]['t'])
            xf = np.array(self.data[follower]['x'])
            yf = np.array(self.data[follower]['y'])

            n = min(len(tl), len(tf))
            dist = np.sqrt((xl[:n] - xf[:n])**2 + (yl[:n] - yf[:n])**2)
            err_mm = (dist - self.gap) * 1000.0

            t_arr = tl[:n]
            mask = t_arr > startup + 1.0
            ax.plot(t_arr[mask], err_mm[mask], color=self.colors[follower],
                    linewidth=1.5,
                    label='{} - {} (error)'.format(
                        follower.replace('_', ' ').title(),
                        leader.replace('_', ' ').title()))
            ax.axhline(y=0, color='red', linestyle='--', linewidth=1,
                       label='Desired ({:.2f} m)'.format(self.gap))
            ax.set_ylabel('Distance Error (mm)')
            ax.set_title(title + ' - Distance Error')
            ax.legend(loc='lower right')
            fine_grid(ax, x_major=5.0, x_minor=1.0, y_major=2.0, y_minor=0.5)

        ax2.set_xlabel('Time (s)')
        fig.tight_layout()
        fig.savefig(out + 'plot_4_distance_error.png')
        plt.close(fig)
        rospy.loginfo('[Recorder]   -> plot_4_distance_error.png')

        fig, ax = plt.subplots(figsize=(10, 5))

        for v in self.vehicles:
            t = self.data[v]['t']
            yaw_deg = [math.degrees(y) for y in self.data[v]['yaw']]
            short = self.labels[v].split('(')[0].strip()
            ax.plot(t, yaw_deg, color=self.colors[v], linewidth=1.5,
                    label=short)

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Yaw Angle (deg)')
        ax.set_title('Vehicle Yaw Angle vs Time')
        ax.legend(loc='upper left')
        fine_grid(ax, x_major=5.0, x_minor=1.0, y_major=10.0, y_minor=2.0)

        fig.tight_layout()
        fig.savefig(out + 'plot_5_yaw.png')
        plt.close(fig)
        rospy.loginfo('[Recorder]   -> plot_5_yaw.png')

        fig, ax = plt.subplots(figsize=(10, 5))

        startup = 8.0
        for v in self.vehicles:
            t = np.array(self.data[v]['t'])
            x = np.array(self.data[v]['x'])
            y = np.array(self.data[v]['y'])

            mask = (t > startup + 2.0) & (x > 0.5)
            t_c = t[mask]
            x_c = x[mask]
            y_c = y[mask]

            if len(t_c) == 0:
                continue

            err_mm = self._nearest_path_error(x_c, y_c)
            short = self.labels[v].split('(')[0].strip()
            ax.plot(t_c, err_mm, color=self.colors[v], linewidth=1.2,
                    label=short)

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Lateral Error (mm)')
        ax.set_title('Lateral Tracking Error vs Time (Nearest-Point Method)')
        ax.legend(loc='upper right')
        ax.axhline(y=0, color='k', linewidth=0.5)
        fine_grid(ax, x_major=5.0, x_minor=1.0, y_major=5.0, y_minor=1.0)

        fig.tight_layout()
        fig.savefig(out + 'plot_6_lateral_error.png')
        plt.close(fig)
        rospy.loginfo('[Recorder]   -> plot_6_lateral_error.png')

        rospy.loginfo('[Recorder] All plots saved to %s', out)


if __name__ == '__main__':
    try:
        ExperimentRecorder()
    except rospy.ROSInterruptException:
        pass
