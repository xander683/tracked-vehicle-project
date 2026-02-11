#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
基于 Pure Pursuit + PID 的车辆跟随控制器

核心策略：
- 找到后车在前车轨迹上的最近投影点
- 从投影点向前看一小段（carrot），驱向 carrot
- 用 PID 控制纵向弧长偏差（落后就加速）
- 角速度用简单比例控制，指向 carrot（低增益避免自旋）
"""

import rospy
import math
import tf.transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from collections import deque


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
        out = self.kp * err + self.ki * self.integral + self.kd * d
        return max(-self.lim, min(self.lim, out))


class PurePursuitFollower:
    def __init__(self, fid, lid, dd=0.55):
        self.fid, self.lid, self.dd = fid, lid, dd

        # 位置
        self.lx = self.ly = self.lyaw = 0.0
        self.fx = self.fy = self.fyaw = 0.0
        self.lok = self.fok = False
        self.lv = 0.0

        # 轨迹
        self.traj = deque(maxlen=60000)
        self.traj_s = 0.0
        self._plx = self._ply = None

        # Carrot 前瞻距离
        self.carrot_dist = 0.20  # 从最近投影点向前看 0.20m（很短，保证稳定）

        # PID 纵向（弧长落后时加速）
        self.pid_v = PID(kp=1.5, ki=0.08, kd=0.3, lim=0.7)

        self.v_max = 1.0
        self.w_max = 1.5    # 限制角速度，避免自旋

        self.state = "waiting"

        # ROS
        rospy.Subscriber("/vehicle_{}/ground_truth/state".format(lid),
                         Odometry, self._lcb, queue_size=1)
        rospy.Subscriber("/vehicle_{}/ground_truth/state".format(fid),
                         Odometry, self._fcb, queue_size=1)
        rospy.Subscriber("/vehicle_{}/cmd_vel".format(lid),
                         Twist, self._vcb, queue_size=1)
        self.pub = rospy.Publisher(
            "/vehicle_{}/cmd_vel".format(fid), Twist, queue_size=10)
        rospy.loginfo("PP+PID V%d→V%d d=%.2f m", fid, lid, dd)

    # --- util ---
    @staticmethod
    def _yaw(q):
        return tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])[2]

    @staticmethod
    def _na(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    # --- callbacks ---
    def _lcb(self, msg):
        p = msg.pose.pose.position
        self.lx, self.ly = p.x, p.y
        self.lyaw = self._yaw(msg.pose.pose.orientation)
        self.lok = True
        if self._plx is None:
            self._plx, self._ply = p.x, p.y
            self.traj.append((p.x, p.y, self.lyaw, 0.0))
            return
        dx, dy = p.x - self._plx, p.y - self._ply
        seg = math.sqrt(dx * dx + dy * dy)
        if seg >= 0.005:
            self.traj_s += seg
            self.traj.append((p.x, p.y, self.lyaw, self.traj_s))
            self._plx, self._ply = p.x, p.y

    def _fcb(self, msg):
        p = msg.pose.pose.position
        self.fx, self.fy = p.x, p.y
        self.fyaw = self._yaw(msg.pose.pose.orientation)
        self.fok = True

    def _vcb(self, msg):
        self.lv = msg.linear.x

    # --- 轨迹插值 ---
    def _interp(self, s):
        n = len(self.traj)
        if n < 2:
            return None
        if s <= self.traj[0][3]:
            return self.traj[0]
        if s >= self.traj[-1][3]:
            return self.traj[-1]
        lo, hi = 0, n - 1
        while lo < hi - 1:
            mid = (lo + hi) // 2
            if self.traj[mid][3] < s:
                lo = mid
            else:
                hi = mid
        a, b = self.traj[lo], self.traj[hi]
        ds = b[3] - a[3]
        if ds < 1e-9:
            return a
        t = (s - a[3]) / ds
        return (a[0] + t * (b[0] - a[0]),
                a[1] + t * (b[1] - a[1]),
                a[2] + t * self._na(b[2] - a[2]),
                s)

    # --- 找最近投影 ---
    def _nearest_s(self):
        if len(self.traj) < 2:
            return 0.0
        best_d2, best_i = float('inf'), 0
        step = max(1, len(self.traj) // 300)
        for i in range(0, len(self.traj), step):
            dx = self.traj[i][0] - self.fx
            dy = self.traj[i][1] - self.fy
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        lo = max(0, best_i - step)
        hi = min(len(self.traj) - 1, best_i + step)
        for i in range(lo, hi + 1):
            dx = self.traj[i][0] - self.fx
            dy = self.traj[i][1] - self.fy
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        return self.traj[best_i][3]

    # --- 控制 ---
    def step(self):
        cmd = Twist()
        if not (self.lok and self.fok):
            return cmd

        if self.state == "waiting":
            if self.traj_s < self.dd:
                return cmd
            self.state = "tracking"
            rospy.loginfo("[V%d] 开始跟踪! traj=%.2f m", self.fid, self.traj_s)

        target_s = self.traj_s - self.dd
        if target_s < 0:
            return cmd

        v_ref = max(0.1, abs(self.lv)) if self.lv != 0 else 0.3

        # 1) 找到后车在轨迹上的投影
        my_s = self._nearest_s()

        # 2) 纵向偏差
        s_err = target_s - my_s  # 正 = 落后

        # 3) Carrot 点：从投影点向前看
        carrot_s = my_s + self.carrot_dist
        # 确保 carrot 不超过轨迹末端
        carrot_s = min(carrot_s, self.traj_s)
        carrot = self._interp(carrot_s)
        if carrot is None:
            return cmd

        # 4) 指向 carrot 的角度
        dx = carrot[0] - self.fx
        dy = carrot[1] - self.fy
        dist_c = math.sqrt(dx * dx + dy * dy)

        if dist_c > 0.005:
            alpha = self._na(math.atan2(dy, dx) - self.fyaw)
        else:
            alpha = 0.0

        # 5) 角速度 = Pure Pursuit 公式（低增益）
        if dist_c > 0.01:
            omega = (2.0 * v_ref * math.sin(alpha)) / max(dist_c, 0.05)
        else:
            omega = 1.0 * alpha

        # 6) 线速度 = 参考 + PID 纵向修正
        v_corr = self.pid_v(s_err)
        v = v_ref + v_corr

        # 大角度时减速（但不要减太多）
        if abs(alpha) > 0.8:
            v *= 0.6
        elif abs(alpha) > 0.4:
            v *= 0.8

        # 限幅
        v = max(0.05, min(self.v_max, v))
        omega = max(-self.w_max, min(self.w_max, omega))

        cmd.linear.x = v
        cmd.angular.z = omega

        # 日志
        if not hasattr(self, '_tl') or (rospy.Time.now() - self._tl).to_sec() > 3.0:
            self._tl = rospy.Time.now()
            ad = math.sqrt((self.lx - self.fx) ** 2 + (self.ly - self.fy) ** 2)
            rospy.loginfo(
                "[V%d] d=%.3f s_err=%.3f a=%.1f° v=%.2f w=%.2f traj=%.1f",
                self.fid, ad, s_err, math.degrees(alpha), v, omega, self.traj_s)

        return cmd

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.pub.publish(self.step())
            rate.sleep()


VehicleFollower = PurePursuitFollower
TrainFollower = PurePursuitFollower

if __name__ == '__main__':
    rospy.init_node('vehicle_follower', anonymous=True)
    f = PurePursuitFollower(
        rospy.get_param('~follower_id', 2),
        rospy.get_param('~leader_id', 1),
        rospy.get_param('~desired_distance', 0.55))
    try:
        f.run()
    except rospy.ROSInterruptException:
        pass
