#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import threading
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState
import tf.transformations


class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, lim=1.0):
        self.kp, self.ki, self.kd, self.lim = kp, ki, kd, lim
        self.integral = 0.0
        self.prev = 0.0
        self.first = True

    def __call__(self, err, dt=0.02):
        if self.first:
            self.prev = err
            self.first = False
        if dt <= 0:
            dt = 0.02
        self.integral += err * dt
        max_i = self.lim / max(self.ki, 1e-9)
        self.integral = max(-max_i, min(max_i, self.integral))
        d = (err - self.prev) / dt
        self.prev = err
        return max(-self.lim, min(self.lim,
                                   self.kp * err + self.ki * self.integral + self.kd * d))


class SCurvePPPIDDriver:
    def __init__(self):
        rospy.init_node('s_curve_pp_pid_driver')

        self.A = rospy.get_param('~amplitude', 0.6)
        self.L = rospy.get_param('~wavelength', 5.0)
        self.lead_in = rospy.get_param('~lead_in', 1.5)
        self.x_end = rospy.get_param('~path_length', 11.5)
        self.speed = rospy.get_param('~speed', 0.3)
        self.desired_dist = rospy.get_param('~desired_distance', 0.55)
        self.startup_delay = rospy.get_param('~startup_delay', 8.0)
        self.hz = 50

        self.leader_lookahead = 0.25
        self.follower_lookahead = 0.20

        self.ref_path = self._build_path(step=0.005)
        self.total_arc = self.ref_path[-1][3]
        rospy.loginfo("[PP+PID] Total arc length: %.2f m", self.total_arc)

        rospy.wait_for_service('/gazebo/get_model_state', timeout=30)
        self.get_model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        self.cmd_pub_v1 = rospy.Publisher('/vehicle_1/cmd_vel', Twist, queue_size=10)
        self.cmd_pub_v2 = rospy.Publisher('/vehicle_2/cmd_vel', Twist, queue_size=10)
        self.cmd_pub_v3 = rospy.Publisher('/vehicle_3/cmd_vel', Twist, queue_size=10)

        self.leader_nearest_s = 0.0

        self.v1_traj = []
        self.v1_traj_s = 0.0
        self.v1_prev = None

        self.v2_traj = []
        self.v2_traj_s = 0.0
        self.v2_prev = None

        self.pid_v2 = PID(kp=1.5, ki=0.1, kd=0.3, lim=0.5)
        self.pid_v3 = PID(kp=1.5, ki=0.1, kd=0.3, lim=0.5)

        rospy.loginfo("[PP+PID] Waiting %.0fs...", self.startup_delay)
        rospy.sleep(self.startup_delay)
        rospy.loginfo("[PP+PID] >>> Start! <<<")

        self._run()

    def _path_y(self, x):
        if x < self.lead_in:
            return 0.0
        return self.A * math.sin(2.0 * math.pi * (x - self.lead_in) / self.L)

    def _path_dydx(self, x):
        if x < self.lead_in:
            return 0.0
        k = 2.0 * math.pi / self.L
        return self.A * k * math.cos(k * (x - self.lead_in))

    def _build_path(self, step=0.005):
        pts = []
        x = -3.0
        s = 0.0
        prev_x, prev_y = x, 0.0
        while x <= self.x_end + 0.5:
            y = self._path_y(max(0, x)) if x >= 0 else 0.0
            ds = math.sqrt((x - prev_x) ** 2 + (y - prev_y) ** 2)
            s += ds
            dydx = self._path_dydx(max(0, x)) if x >= 0 else 0.0
            yaw = math.atan2(dydx, 1.0)
            pts.append((x, y, yaw, s))
            prev_x, prev_y = x, y
            x += step

        offset = 0
        for p in pts:
            if p[0] >= 0:
                offset = p[3]
                break
        pts = [(p[0], p[1], p[2], p[3] - offset) for p in pts]
        return pts

    def _interp_path(self, target_s):
        if target_s <= self.ref_path[0][3]:
            p = self.ref_path[0]
            return p[0], p[1], p[2]
        if target_s >= self.ref_path[-1][3]:
            p = self.ref_path[-1]
            return p[0], p[1], p[2]
        lo, hi = 0, len(self.ref_path) - 1
        while lo < hi - 1:
            mid = (lo + hi) // 2
            if self.ref_path[mid][3] < target_s:
                lo = mid
            else:
                hi = mid
        p1 = self.ref_path[lo]
        p2 = self.ref_path[hi]
        ds = p2[3] - p1[3]
        if ds < 1e-9:
            return p1[0], p1[1], p1[2]
        t = (target_s - p1[3]) / ds
        x = p1[0] + t * (p2[0] - p1[0])
        y = p1[1] + t * (p2[1] - p1[1])
        yaw = p1[2] + t * self._norm(p2[2] - p1[2])
        return x, y, yaw

    def _path_nearest_s(self, px, py):
        best_d2, best_i = float('inf'), 0
        step = max(1, len(self.ref_path) // 500)
        for i in range(0, len(self.ref_path), step):
            dx = self.ref_path[i][0] - px
            dy = self.ref_path[i][1] - py
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        lo = max(0, best_i - step)
        hi = min(len(self.ref_path) - 1, best_i + step)
        for i in range(lo, hi + 1):
            dx = self.ref_path[i][0] - px
            dy = self.ref_path[i][1] - py
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        return self.ref_path[best_i][3]

    @staticmethod
    def _norm(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def _record_traj(self, traj, traj_s, prev_pos, x, y, yaw):
        if prev_pos is None:
            traj.append((x, y, yaw, 0.0))
            return 0.0, (x, y)
        dx = x - prev_pos[0]
        dy = y - prev_pos[1]
        seg = math.sqrt(dx * dx + dy * dy)
        if seg >= 0.005:
            new_s = traj_s + seg
            traj.append((x, y, yaw, new_s))
            return new_s, (x, y)
        return traj_s, prev_pos

    def _traj_nearest_s(self, traj, fx, fy):
        if len(traj) < 2:
            return 0.0
        best_d2, best_i = float('inf'), 0
        step = max(1, len(traj) // 300)
        for i in range(0, len(traj), step):
            dx = traj[i][0] - fx
            dy = traj[i][1] - fy
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        lo = max(0, best_i - step)
        hi = min(len(traj) - 1, best_i + step)
        for i in range(lo, hi + 1):
            dx = traj[i][0] - fx
            dy = traj[i][1] - fy
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        return traj[best_i][3]

    def _traj_interp(self, traj, s):
        n = len(traj)
        if n < 2:
            return None
        if s <= traj[0][3]:
            return traj[0]
        if s >= traj[-1][3]:
            return traj[-1]
        lo, hi = 0, n - 1
        while lo < hi - 1:
            mid = (lo + hi) // 2
            if traj[mid][3] < s:
                lo = mid
            else:
                hi = mid
        a, b = traj[lo], traj[hi]
        ds = b[3] - a[3]
        if ds < 1e-9:
            return a
        t = (s - a[3]) / ds
        return (a[0] + t * (b[0] - a[0]),
                a[1] + t * (b[1] - a[1]),
                a[2] + t * self._norm(b[2] - a[2]),
                s)

    def _get_pose(self, model_name):
        try:
            resp = self.get_model(model_name, "world")
            if resp.success:
                q = resp.pose.orientation
                yaw = tf.transformations.euler_from_quaternion(
                    [q.x, q.y, q.z, q.w])[2]
                return resp.pose.position.x, resp.pose.position.y, yaw
        except:
            pass
        return None

    def _pure_pursuit(self, robot_x, robot_y, robot_yaw,
                      target_x, target_y, v_ref):
        dx = target_x - robot_x
        dy = target_y - robot_y
        dist = math.sqrt(dx * dx + dy * dy)
        if dist < 0.01:
            return 0.0

        alpha = self._norm(math.atan2(dy, dx) - robot_yaw)
        omega = (2.0 * v_ref * math.sin(alpha)) / max(dist, 0.05)

        return omega

    def _run(self):
        rate = rospy.Rate(self.hz)
        dt = 1.0 / self.hz
        start_time = rospy.Time.now()
        finished = False

        while not rospy.is_shutdown() and not finished:
            elapsed = (rospy.Time.now() - start_time).to_sec()

            v1_pose = self._get_pose('vehicle_1')
            v2_pose = self._get_pose('vehicle_2')
            v3_pose = self._get_pose('vehicle_3')

            if v1_pose is None:
                rate.sleep()
                continue

            v1x, v1y, v1yaw = v1_pose

            my_s = self._path_nearest_s(v1x, v1y)
            self.leader_nearest_s = my_s

            carrot_s = my_s + self.leader_lookahead
            if carrot_s >= self.total_arc:
                carrot_s = self.total_arc
                if my_s >= self.total_arc - 0.05:
                    rospy.loginfo("[PP+PID] Leader reached end!")
                    self.cmd_pub_v1.publish(Twist())
                    self.cmd_pub_v2.publish(Twist())
                    self.cmd_pub_v3.publish(Twist())
                    finished = True
                    break

            cx, cy, _ = self._interp_path(carrot_s)

            omega_leader = self._pure_pursuit(v1x, v1y, v1yaw, cx, cy, self.speed)
            omega_leader = max(-2.0, min(2.0, omega_leader))

            cmd1 = Twist()
            cmd1.linear.x = self.speed
            cmd1.angular.z = omega_leader
            self.cmd_pub_v1.publish(cmd1)

            self.v1_traj_s, self.v1_prev = self._record_traj(
                self.v1_traj, self.v1_traj_s, self.v1_prev, v1x, v1y, v1yaw)

            cmd2 = Twist()
            if v2_pose is not None and len(self.v1_traj) >= 2:
                v2x, v2y, v2yaw = v2_pose
                v2_v, v2_w = self._follower_pp_pid(
                    v2x, v2y, v2yaw,
                    self.v1_traj, self.v1_traj_s,
                    self.pid_v2, self.follower_lookahead)
                cmd2.linear.x = v2_v
                cmd2.angular.z = v2_w

                self.v2_traj_s, self.v2_prev = self._record_traj(
                    self.v2_traj, self.v2_traj_s, self.v2_prev, v2x, v2y, v2yaw)

            self.cmd_pub_v2.publish(cmd2)

            cmd3 = Twist()
            if v3_pose is not None and len(self.v2_traj) >= 2:
                v3x, v3y, v3yaw = v3_pose
                v3_v, v3_w = self._follower_pp_pid(
                    v3x, v3y, v3yaw,
                    self.v2_traj, self.v2_traj_s,
                    self.pid_v3, self.follower_lookahead)
                cmd3.linear.x = v3_v
                cmd3.angular.z = v3_w

            self.cmd_pub_v3.publish(cmd3)

            if int(elapsed * self.hz) % (self.hz * 5) == 0:
                rospy.loginfo(
                    "[PP+PID] t=%.0fs s=%.1f V1=(%.2f,%.2f) v=%.2f w=%.2f | "
                    "V2 v=%.2f w=%.2f | V3 v=%.2f w=%.2f",
                    elapsed, my_s, v1x, v1y,
                    cmd1.linear.x, cmd1.angular.z,
                    cmd2.linear.x, cmd2.angular.z,
                    cmd3.linear.x, cmd3.angular.z)

            rate.sleep()

        rospy.loginfo("[PP+PID] Done.")
        rospy.sleep(2.0)

    def _follower_pp_pid(self, fx, fy, fyaw, leader_traj, leader_traj_s,
                         pid_ctrl, lookahead):
        my_s = self._traj_nearest_s(leader_traj, fx, fy)

        target_s = leader_traj_s - self.desired_dist

        if target_s < 0:
            return 0.0, 0.0

        s_err = target_s - my_s

        v_corr = pid_ctrl(s_err, 1.0 / self.hz)
        v = self.speed + v_corr

        carrot_s = my_s + lookahead
        carrot_s = min(carrot_s, leader_traj_s)

        carrot = self._traj_interp(leader_traj, carrot_s)
        if carrot is None:
            return max(0.0, v), 0.0

        omega = self._pure_pursuit(fx, fy, fyaw, carrot[0], carrot[1], max(v, 0.05))

        dx = carrot[0] - fx
        dy = carrot[1] - fy
        alpha = abs(self._norm(math.atan2(dy, dx) - fyaw))
        if alpha > 0.8:
            v *= 0.5

        v = max(0.05, min(0.8, v))
        omega = max(-2.0, min(2.0, omega))

        return v, omega


if __name__ == '__main__':
    try:
        SCurvePPPIDDriver()
    except rospy.ROSInterruptException:
        pass
