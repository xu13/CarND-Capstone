#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
import numpy as np
from scipy.spatial import KDTree

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50  # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose_stamped = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoints_kdtree = None

        self.loop()

    def loop(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            rospy.loginfo(self.pose_stamped)
            if self.pose_stamped and self.waypoints_kdtree: # wait for kdtree to be initialized
                # Get closest waypoint index
                closest_waypoint_index = self.get_closest_waypoint_index()
                self.publish_waypoints(closest_waypoint_index)
            rate.sleep()

    def get_closest_waypoint_index(self):
        x = self.pose_stamped.pose.position.x
        y = self.pose_stamped.pose.position.y
        closest_index = self.waypoints_kdtree.query([x, y], 1)[1]

        # Check if the closest waypoint is ahead of vehicle or behind vehicle
        closest_vec = np.array(self.waypoints_2d[closest_index])
        previous_vec = np.array(self.waypoints_2d[closest_index - 1])
        pose_vec = np.array([x, y])

        val = np.dot(closest_vec - previous_vec, pose_vec - closest_vec)
        if val > 0:
            # Closest waypoint is behind the vehicle
            return closest_index + 1
        else:
            return closest_index

    def publish_waypoints(self, index):
        lane = Lane()
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = self.base_waypoints.waypoints[index:index + LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose_stamped = msg

    def waypoints_cb(self, msg):
        # TODO: Implement
        self.base_waypoints = msg
        if not self.waypoints_2d:
            self.waypoints_2d = [[wp.pose.pose.position.x, wp.pose.pose.position.y] for wp in msg.waypoints]
            self.waypoints_kdtree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
