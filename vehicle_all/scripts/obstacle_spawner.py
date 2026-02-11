#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion


class ObstacleSpawner:

    def __init__(self):
        rospy.init_node('obstacle_spawner')

        self.A = rospy.get_param('~amplitude', 0.6)
        self.L = rospy.get_param('~wavelength', 5.0)
        self.lead_in = rospy.get_param('~lead_in', 1.5)
        self.x_end = rospy.get_param('~path_length', 11.5)

        self.offset = rospy.get_param('~offset', 0.35)
        self.seg_step = rospy.get_param('~seg_step', 0.12)
        self.wall_thick = rospy.get_param('~wall_thick', 0.06)
        self.wall_height = rospy.get_param('~wall_height', 0.40)
        self.start_x = rospy.get_param('~start_x', 0.15)
        self.end_margin = rospy.get_param('~end_margin', 0.15)

        rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=30)
        self.spawn = rospy.ServiceProxy(
            '/gazebo/spawn_sdf_model', SpawnModel, persistent=True)
        rospy.sleep(2.0)

        self._spawn_walls()

    def _path_y(self, x):
        if x < self.lead_in:
            return 0.0
        return self.A * math.sin(2.0 * math.pi * (x - self.lead_in) / self.L)

    def _path_dydx(self, x):
        if x < self.lead_in:
            return 0.0
        k = 2.0 * math.pi / self.L
        return self.A * k * math.cos(k * (x - self.lead_in))

    def _normal(self, x):
        dydx = self._path_dydx(x)
        mag = math.sqrt(1.0 + dydx * dydx)
        return (-dydx / mag, 1.0 / mag)

    def _tangent_yaw(self, x):
        dydx = self._path_dydx(x)
        return math.atan2(dydx, 1.0)

    def _build_wall_sdf(self, model_name, segments):
        parts = []
        parts.append("<?xml version='1.0'?>")
        parts.append("<sdf version='1.6'>")
        parts.append("<model name='{}'>".format(model_name))
        parts.append("  <static>true</static>")
        parts.append("  <link name='wall_link'>")

        for i, (cx, cy, cz, yaw, seg_len) in enumerate(segments):
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            pose_str = "{cx} {cy} {cz} 0 0 {yaw}".format(
                cx=cx, cy=cy, cz=cz, yaw=yaw)

            parts.append("    <visual name='v_{i}'>".format(i=i))
            parts.append("      <pose>{}</pose>".format(pose_str))
            parts.append("      <geometry>")
            parts.append("        <box><size>{l} {t} {h}</size></box>".format(
                l=seg_len, t=self.wall_thick, h=self.wall_height))
            parts.append("      </geometry>")
            parts.append("      <material>")
            parts.append("        <ambient>0.55 0.55 0.55 1</ambient>")
            parts.append("        <diffuse>0.65 0.65 0.65 1</diffuse>")
            parts.append("        <specular>0.1 0.1 0.1 1</specular>")
            parts.append("      </material>")
            parts.append("    </visual>")

            parts.append("    <collision name='c_{i}'>".format(i=i))
            parts.append("      <pose>{}</pose>".format(pose_str))
            parts.append("      <geometry>")
            parts.append("        <box><size>{l} {t} {h}</size></box>".format(
                l=seg_len, t=self.wall_thick, h=self.wall_height))
            parts.append("      </geometry>")
            parts.append("    </collision>")

        parts.append("  </link>")
        parts.append("</model>")
        parts.append("</sdf>")
        return "\n".join(parts)

    def _generate_segments(self, sign):
        segments = []
        x = self.start_x
        while x < self.x_end - self.end_margin:
            x_next = min(x + self.seg_step, self.x_end - self.end_margin)
            xc = (x + x_next) / 2.0
            seg_len = x_next - x + 0.02

            yc = self._path_y(xc)
            nx, ny = self._normal(xc)
            yaw = self._tangent_yaw(xc)

            wx = xc + sign * self.offset * nx
            wy = yc + sign * self.offset * ny
            wz = self.wall_height / 2.0

            segments.append((wx, wy, wz, yaw, seg_len))
            x = x_next

        return segments

    def _spawn_walls(self):
        for side, sign in [('left', 1), ('right', -1)]:
            segments = self._generate_segments(sign)
            model_name = 'wall_{}'.format(side)
            sdf_str = self._build_wall_sdf(model_name, segments)

            pose = Pose()
            pose.position = Point(0, 0, 0)
            pose.orientation = Quaternion(0, 0, 0, 1)

            try:
                self.spawn(model_name, sdf_str, '', pose, 'world')
                rospy.loginfo('[ObstacleSpawner] Spawned %s (%d segments)', model_name, len(segments))
            except Exception as e:
                rospy.logerr('[ObstacleSpawner] Failed %s: %s', model_name, e)

        rospy.loginfo('[ObstacleSpawner] Done! Corridor width: %.2f m, wall height: %.2f m',
                      self.offset * 2, self.wall_height)


if __name__ == '__main__':
    try:
        ObstacleSpawner()
    except rospy.ROSInterruptException:
        pass
