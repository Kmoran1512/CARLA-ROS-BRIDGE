import math
import time

from typing import List
from geometry_msgs.msg import PoseStamped


SPAWN_DISTANCE = 70

class PedestrianAction:
    def __init__(self, action):
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

        begin_loc = waypoints[SPAWN_DISTANCE - int(self.mdelay)].pose.position
        next_loc = waypoints[SPAWN_DISTANCE - int(self.mdelay) + 1].pose.position
        self.begin_x, self.begin_y = begin_loc.x, begin_loc.y
        self.next_x, self.next_y = next_loc.x, next_loc.y

    def should_run(self, v_loc):
        return (
            self.is_in_triggering_distance(v_loc)
            or self.is_in_triggering_time()
        )

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

