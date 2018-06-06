#! /usr/bin/python

import rospkg
import rospy
import time
import pickle
import tf2_ros
import tf2_geometry_msgs
from dc_pyutils import DCPlannerUtil
from hanoi_actuator.srv import Action, ActionResponse

from robot_control_modules import *
from math import pi, cos, sin


class Actuator:
    def __init__(self, prefix, *args, **kwargs):
        rospy.loginfo("Initializing actuator")
        self.quat_vertical = tf.transformations.quaternion_from_euler(
            180*3.1415/180, 0*3.1415/180, 0*3.1415/180, 'rxyz')
        self.prefix = prefix
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, queue_size=1)
        self.init_position()

    def init_position(self):
        homeRobot(self.prefix)
        self.go_home()
        self.open_gripper()

    def safe_position(self, position):
        # For safety: make sure the robot arm does not hit stuff
        x = max(min(position[0], 0.50), -.50) - 0.02
        if x < -0.10:
            x = -0.14
        if x > 0.08:
            x -= 0.01
        y = min(max(position[1], -1), 0.0) + 0.07
        z = max(min(position[2], 0.33), 0.02) + 0.05
        return (x, y, z)

    def as_tuple(self, position):
        return (position.x, position.y, position.z)

    def go_home(self):
        rospy.loginfo('Going home!')
        self.go_to_aligned((0.0, -.2, .4))

    def pickup(self, position):
        rospy.loginfo("Picking up item at position\n{}".format(position))
        safe_pos = self.safe_position(position)
        # y = safe_pos[1]
        y = -0.54
        self.open_gripper()
        # Go to 10 cm in front of the block
        self.go_to_aligned((safe_pos[0], y + 0.1, safe_pos[2]))
        # Go to the block
        self.go_to_aligned((safe_pos[0], y, safe_pos[2]))
        self.grasp()
        # Go to above position
        self.go_to_aligned((safe_pos[0], y, 0.4))

    def drop(self, position):
        rospy.loginfo("Dropping item at position\n{}".format(position))
        safe_pos = self.safe_position(position)
        # y = safe_pos[1]
        y = -0.54
        # Go to above position
        self.go_to_aligned((safe_pos[0], y, 0.4))
        # Go to the drop position
        self.go_to_aligned((safe_pos[0], y, safe_pos[2]))
        self.open_gripper()
        self.go_to_aligned((safe_pos[0], y, 0.4))

    def go_to_aligned(self, position):
        p, q = self.generate_gripper_align_pose(
            position, 0.03, math.pi/2.0, 0.0, 0.0)
        return cartesian_pose_client(p, q, self.prefix)

    def open_gripper(self):
        rospy.loginfo('Opening gripper')
        return gripper_client([0, 0, 0], self.prefix)

    def close_gripper(self):
        rospy.loginfo('Closing gripper')
        return gripper_client([6400, 6400, 0], self.prefix)

    def grasp(self):
        rospy.loginfo('Grasping object')
        return gripper_client([2600, 2600, 0], self.prefix)

    # /**
    # Source (in C++): https://github.com/Kinovarobotics/kinova-ros/blob/master/kinova_moveit/kinova_arm_moveit_demo/src/pick_place.cpp
    #  * @brief PickPlace::generate_gripper_align_pose
    #  * @param targetpose_msg pick/place pose (object location): where gripper close/open the fingers (grasp/release the object). Only position information is used.
    #  * @param dist distance of returned pose to targetpose
    #  * @param azimuth an angle measured from the x-axis in the xy-plane in spherical coordinates, denoted theta (0<= theta < 2pi ).
    #  * @param polar also named zenith, colatitude, denoted phi (0<=phi<=pi). It is the angle from the positive z-axis to the vector.  phi= pi/2 - delta where delta is the latitude.
    #  * @param rot_gripper_z rotation along the z axis of the gripper reference frame (last joint rotation)
    #  * @return a pose defined in a spherical coordinates where origin is located at the target pose. Normally it is a pre_grasp/post_realease pose, where gripper axis (last joint axis) is pointing to the object (target_pose).
    #  */
    def generate_gripper_align_pose(self, target_position, dist, azimuth, polar, rot_gripper_z):
        # computer pregrasp position w.r.t. location of grasp_pose in spherical coordinate. Orientation is w.r.t. fixed world (root) reference frame.
        delta_x = -dist * cos(azimuth) * sin(polar)
        delta_y = -dist * sin(azimuth) * sin(polar)
        delta_z = -dist * cos(polar)

        # compute the orientation of gripper w.r.t. fixed world (root) reference frame. The gripper (z axis) should point(open) to the grasp_pose.
        q = tf.transformations.quaternion_from_euler(
            azimuth, polar, rot_gripper_z, 'rxyz')

        px = target_position[0] + delta_x
        py = target_position[1] + delta_y
        pz = target_position[2] + delta_z

        return ((px, py, pz), q)

    def handle_action(self, req):
        transformed_from = self.as_tuple(self.transform(req.from_pose))
        self.pickup(transformed_from)
        transformed_to = self.as_tuple(self.transform(req.to_pose))
        self.drop(transformed_to)
        self.go_home()

        resp = ActionResponse(True, 'Msg for ActionResponse')
        resp.success = True
        resp.message = "Action succesfully executed!"
        return resp

    def transform(self, pose):
        transform = self.tfBuffer.lookup_transform("{}link_base".format(
            self.prefix), pose.header.frame_id, rospy.Time(0))
        res = tf2_geometry_msgs.do_transform_pose(pose, transform)
        return res.pose.position

    def handle_action_mock(self, req):
        print(req)
        raw_input("Press Enter to continue...")
        resp = ActionResponse(True, 'Msg for ActionResponse')
        resp.success = True
        resp.message = "Action succesfully executed!"
        return resp

    def __str__(self):
        return "Actuator for robot {}".format(self.prefix)


def main():
    rospy.init_node('hanoi_actuator')
    a = Actuator('m1n6s200_')
    print(a)
    rospy.Service('hanoi_actuator_srv', Action, a.handle_action)
    rospy.loginfo("Ready to spin")
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
