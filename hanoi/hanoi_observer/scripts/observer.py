#! /usr/bin/python

import geometry_msgs.msg
import rospy
import tf
import tf2_geometry_msgs
import tf2_ros
from ar_track_alvar_msgs.msg import AlvarMarkers
from hanoi_observer.msg import Observation, Observations
from std_msgs.msg import String
from visualization_msgs.msg import Marker

import matplotlib.pyplot as plt

REFERENCE_TAG_ID = 0
ROBOT_TAG_ID = 1
ORIGIN_FRAME = "origin"
ROBOT_BASE_FRAME = "m1n6s200_link_base"


class SimpleObserver:
    """
    Used ROS params:
    * with_visualization: enable sending visualization markers and plotting the transformed coordinates in 2D (required matplotlib)
    """
    def __init__(self, **args):
        self.with_visualization = rospy.get_param('with_visualization', False)
        if self.with_visualization:
            self.marker_publisher = rospy.Publisher('hanoi_marker', Marker)
            plt.ion()
            plt.pause(0.01)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.br_static = tf2_ros.StaticTransformBroadcaster()
        self.origin_marker_frame = args.get(
            'origin_marker_frame') or "ar_marker_{}".format(REFERENCE_TAG_ID)
        self.camera_frame = "kinect2_link"


        self.publisher = rospy.Publisher('hanoi_observer', Observations, queue_size=1)
        rospy.loginfo("Initialized {}".format(self))

    def get_frame(self, marker_id):
        """
        Helper method to return the tf frame name for the given AR tag.
        """
        return "ar_marker_{}".format(marker_id)

    def callback(self, msg):
        """
        Callback for the ar_pose_marker topic.
        This will transform the markers
        and publish them on /hanoi_observer topic.
        """
        rospy.loginfo("Callback {}".format(self))
        new_msg = Observations()
        new_msg.header = msg.header
        self.transform = self.tf_buffer.lookup_transform(
            self.origin_marker_frame, self.camera_frame, rospy.Time(0), rospy.Duration(5))
        for marker in msg.markers:
            if marker.id in [ROBOT_TAG_ID]:
                # Skip the robot tag because we do not need to provide it to the reasoner as an Observation
                continue
            try:
                transformed_pose = tf2_geometry_msgs.do_transform_pose(
                    marker.pose, self.transform)
                observation = Observation()
                observation.id = marker.id
                observation.pose = transformed_pose
                rospy.loginfo("marker {} transformed pose {}".format(
                    marker.id, self.prettify(transformed_pose)))
                if self.with_visualization:
                    self.send_marker(observation)
                new_msg.observations.append(observation)
            except Exception, e:
                rospy.logerr(e)
        if self.with_visualization:
            plt.clf()
            plt.axis([-.5, 1, -.5, 1])
            self.plot(new_msg, color='green')
        self.publisher.publish(new_msg)
        rospy.sleep(rospy.Duration(1))

    def prettify(self, pose):
        """
        Creates a simple string to print the position in the given pose.
        """
        return "({}, {}, {})".format(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)

    def send_marker(self, observation):
        """
        Publishes a visualization marker of the transformed observation to use in rviz for example.
        """
        marker = Marker()
        marker.ns = 'hanoi_marker'
        marker.id = observation.id
        marker.action = 0  # add/modify
        marker.text = "Tag {}".format(observation.id)
        marker.header.frame_id = ORIGIN_FRAME
        marker.header.stamp = rospy.Time.now()
        marker.lifetime = rospy.Duration(1)
        marker.type = 1  # cube
        marker.pose = observation.pose.pose
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_publisher.publish(marker)

    def plot(self, msg, color='black'):
        """
        Plots the observations in a 2D plane using matplotlib.
        """
        coords = map(lambda m: (float(m.pose.pose.position.x),
                                float(m.pose.pose.position.y)), msg.observations)
        plt.scatter(*zip(*coords), c=color)
        for o in msg.observations:
            plt.annotate(
                o.id, (o.pose.pose.position.x, o.pose.pose.position.y))
        plt.pause(0.01)

    def __str__(self):
        return "SimpleObserver with origin {}".format(self.origin_marker_frame)


def main():
    rospy.init_node('hanoi_observer')
    t = SimpleObserver()

    rospy.Subscriber("ar_pose_marker", AlvarMarkers, t.callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
