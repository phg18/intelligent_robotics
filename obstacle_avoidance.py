#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from ackermann_msgs.msg import AckermannDrive
import numpy as np
import copy

class ObstacleAvoidance:
    def __init__(self):
        # Initialise the ROS node
        rospy.init_node('obstacle_avoidance')

        # Subscriber for the point cloud of obstacles captured by lidar
        rospy.Subscriber("/obstacles", PointCloud2, self.obstacle_callback)

        # Subscriber for Ackermann control commands
        rospy.Subscriber("/blue/preorder_ackermann_cmd", AckermannDrive, self.ackermann_callback)

        # TODO Publisher for modified Ackermann commands
        self.cmd_pub = rospy.Publisher("/blue/ackermann_cmd", AckermannDrive, queue_size=10)
        # Store the last Ackermann message received
        self.last_ackermann_cmd = AckermannDrive()

    def obstacle_callback(self, msg):
        pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        obstacles_in_front = []
        for point in pc_data:
            x, y, z = point
            if 0.0 < x < 1.5 and abs(y) < 0.5:
                obstacles_in_front.append((x, y, z))
        #Send ackermann's message 
        cmd = self.last_ackermann_cmd

        # TODO Modify ackermann's message if necessary
        if len(obstacles_in_front) > 0:
            print("Obstacle detected -- stopping BLUE")
            if cmd.speed >= 0.0:
                cmd = self.modify_ackermann_command()

        self.cmd_pub.publish(cmd)
        
        return

    def ackermann_callback(self, msg):
        # Stores the last command received
        self.last_ackermann_cmd = msg

    def modify_ackermann_command(self):
        cmd = copy.deepcopy(self.last_ackermann_cmd)
        cmd.speed = 0.0  
        cmd.steering_angle = 0.0
        return cmd

if __name__ == '__main__':
    oa = ObstacleAvoidance()
    rospy.spin()
