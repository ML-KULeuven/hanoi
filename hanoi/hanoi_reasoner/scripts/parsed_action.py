import re

from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion
from hanoi_actuator.srv import ActionRequest
import rospy

# https://regex101.com/r/bvIxIn/1
ACTION_REGEX = r"action\(move\((?P<disk>\d+),pos\((?P<disk_x>-?\d+\.\d+),(?P<disk_y>-?\d+\.\d+),(?P<disk_z>-?\d+\.\d+)\),(?P<tower>\d+),pos\((?P<tower_x>-?\d+\.\d+),(?P<tower_y>-?\d+\.\d+),(?P<tower_z>-?\d+\.\d+)\)\)\)"


class ParsedAction:
    @staticmethod
    def match(action):
        return re.compile(ACTION_REGEX).match(action)

    def __init__(self, action, obs_msg):
        action = action.replace(' ', '')
        re_match = ParsedAction.match(action)
        re_groups = re_match.groupdict()
        self.disk = int(re_groups['disk'])
        self.disk_pos = (float(re_groups['disk_x']), float(
            re_groups['disk_y']), float(re_groups['disk_z']))
        self.disk_orientation = self.get_orientation(self.disk, obs_msg)
        self.tower = int(re_groups['tower'])
        self.tower_orientation = self.get_orientation(self.tower, obs_msg)
        self.tower_pos = (float(re_groups['tower_x']), float(
            re_groups['tower_y']), float(re_groups['tower_z']))

    def get_orientation(self, obj_id, obs_msg):
        if obs_msg is None:
            return Quaternion()
        for o in obs_msg.observations:
            if o.id == obj_id:
                return o.pose.pose.orientation
        return Quaternion()

    def from_point(self):
        from_pose = PoseStamped()
        from_pose.header.frame_id = "ar_marker_0"
        from_pose.header.stamp = rospy.Time.now()
        position = Point()
        position.x = self.disk_pos[0]
        position.y = self.disk_pos[1]
        position.z = self.disk_pos[2]
        from_pose.pose.position = position
        from_pose.pose.orientation = self.disk_orientation
        return from_pose

    def to_point(self):
        to_pose = PoseStamped()
        to_pose.header.frame_id = "ar_marker_0"
        to_pose.header.stamp = rospy.Time.now()
        position = Point()
        position.x = self.tower_pos[0]
        position.y = self.tower_pos[1]
        position.z = self.tower_pos[2]
        to_pose.pose.position = position
        to_pose.pose.orientation = self.disk_orientation
        return to_pose

    def create_request(self):
        req = ActionRequest()

        req.disk_id = self.disk
        req.tower_id = self.tower
        req.from_pose = self.from_point()
        req.to_pose = self.to_point()

        return req

    def __str__(self):
        return "action(move({},pos{},{},pos{}))".format(self.disk, self.disk_pos, self.tower, self.tower_pos)
