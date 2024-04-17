import math
import time

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from transforms3d.euler import euler2quat
from typing import List


class Pedestrian:
    OFFSETS = {
        "center": 0.0,
        "right": 3.5,
        "left": -3.5,
        "far_right": 6.0,
        "far_left": -7.7,
        "near_left_margin": -2.2,
        "near_left_bike": -1.0,
        "far_left_margin": -5.7,
        "near_right_margin": 8.8,
        "far_right_margin": 11.0,
    }

    def __init__(self, bp=0, dir=0, lane="center", pos_in_lane=0) -> None:
        self.bp = bp
        self.yaw = math.radians(dir)
        self.lane = lane

        self.has_run = False

        self.calculate_x_offset(pos_in_lane)
        self.calculate_y_offset(pos_in_lane)

    def calculate_x_offset(self, pos_in_lane):
        mod_3 = pos_in_lane % 3
        divided_by_3 = pos_in_lane // 3

        self.x_offset = -2.5 * divided_by_3

        if mod_3 != 0:
            self.x_offset -= 1

    def calculate_y_offset(self, pos_in_lane):
        mod_3 = pos_in_lane % 3
        self.y_offset = self.OFFSETS[self.lane]

        if mod_3 == 1:
            self.y_offset -= 1.0
        elif mod_3 == 2:
            self.y_offset += 1.0

    def get_spawn(self, ref_x, ref_y, ref_theta):
        a, b, c, d = euler2quat(self.yaw - ref_theta, math.pi, 0)
        return Pose(
            position=Point(x=ref_x + self.x_offset, y=ref_y + self.y_offset, z=1.0),
            orientation=Quaternion(x=a, y=b, z=c, w=d),
        )


class PedestrianAction:
    def __init__(self, spawn_distance: int, action):
        self.spawn_distance = spawn_distance

        self.speed = action.get("speed", 0.0)

        yaw = math.radians(action.get("yaw", 0.0))
        self.x_dir, self.y_dir = math.cos(yaw), math.sin(yaw)

        self.mdelay = action.get("mdelay", 0.0)
        self.tdelay = action.get("tdelay", 0.0)

    def set_previous_time(self, prev_time):
        self.start_time = prev_time

    def set_dist(self, waypoints: List[PoseStamped]):
        if not self.mdelay:
            return

        begin_loc = waypoints[self.spawn_distance - int(self.mdelay)].pose.position
        next_loc = waypoints[self.spawn_distance - int(self.mdelay) + 1].pose.position
        self.begin_x, self.begin_y = begin_loc.x, begin_loc.y
        self.next_x, self.next_y = next_loc.x, next_loc.y

    def should_run(self, v_loc):
        return self.is_in_triggering_distance(v_loc) or self.is_in_triggering_time()

    def is_in_triggering_distance(self, v_loc) -> bool:
        if not self.mdelay:
            return False

        dist_to_trigger = distance(*v_loc, self.begin_x, self.begin_y)
        dist_after_trigger = distance(*v_loc, self.next_x, self.next_y)

        return dist_to_trigger > dist_after_trigger

    def is_in_triggering_time(self) -> bool:
        if not self.tdelay:
            return False

        time_passed = time.time() - self.start_time
        return time_passed > self.tdelay


def distance(x0, y0, x1, y1):
    return math.sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2)
