import rospy
import numpy as np
import pandas as pd
import os

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, PoseArray
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class PathGenerator:

    def __init__(self, source_topic, target_topic, plan_topic, good_file_name, bad_file_name, start_file_name):

        self.plan = []   #initialize the plan as an empty list
        self.next_wpt = 0

        self.wpts = self.read_csv(good_file_name)  # Load good waypoints
        self.bad_wpts = self.read_csv(bad_file_name)  # Load bad waypoints
        self.start_wpt = self.read_csv(start_file_name)[0]  # Load start point'

        self.source_pub = rospy.Publisher(source_topic, PoseWithCovarianceStamped, queue_size=10) # Create a publisher to source_topic
        self.target_pub = rospy.Publisher(target_topic, PoseStamped, queue_size=10) # Create a publisher to target_topic
        self.plan_sub = rospy.Subscriber(plan_topic, PoseArray, self.plan_cb) # Create a subscriber to plan_topic, with callback 'self.plan_cb'
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    

    def read_csv(file_name):
        csv_folder_path = os.path.join(os.path.dirname(__file__), '..', 'csv_files')
        file_path = os.path.join(csv_folder_path, file_name)
        try:
            data = pd.read_csv(file_path)
            return list(zip(data['x'], data['y']))
        except FileNotFoundError:
            print("File not found.")
            return None
    

    def plan_cb(self, plan):
        self.plan.append(plan)
        if self.next_wpt < len(self.wpts):
            pose = Pose()
            pose.position.x, pose.position.y = self.wpts[self.next_wpt]
            pose.position.z = 0.0
            pose.orientation = Utils.angle_to_quaternion(0.0)
            self.source_pub.publish(pose)

            self.next_wpt += 1
            if self.next_wpt < len(self.wpts):
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x, pose.pose.position.y = self.wpts[self.next_wpt]
                pose.pose.position.z = 0.0
                pose.pose.orientation = Utils.angle_to_quaternion(0.0)
                self.target_pub.publish(pose)


    def publish_markers(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.2  # Marker size

        # Good waypoints (blue)
        marker.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)  # Blue
        for point in self.wpts:
            p = Pose()
            p.position.x, p.position.y = point
            marker.points.append(p.position)

        # Bad waypoints (red)
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red
        for point in self.bad_wpts:
            p = Pose()
            p.position.x, p.position.y = point
            marker.points.append(p.position)

        # Start point (green)
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Green
        p = Pose()
        p.position.x, p.position.y = self.start_wpt
        marker.points.append(p.position)

        self.marker_pub.publish(marker)
    
    #start_point = self.read_csv('start-1.csv')[0] # (x,y)
    #good_waypoints = self.read_csv('good_waypoints-1.csv') # [(x0,y0), ..., (xn,yn)]
    #bad_waypoints = self.read_csv('bad_waypoints-1.csv') # [(x0,y0), ..., (xn,yn)]

    if __name__ == '__main__':
        rospy.init_node('line_follower', anonymous=True) # Initialize the node

        plan_topic = rospy.get_param("~plan_topic", '/planner_node/car_plan')
        source_topic = rospy.get_param("~source_topic", '/initialpose')
        target_topic = rospy.get_param("~target_topic", '/move_base_simple/goal')
        good_file_name = rospy.get_param("~good_file_name", 'good_waypoints-1.csv')
        bad_file_name = rospy.get_param("~bad_file_name", 'bad_waypoints-1.csv')
        start_file_name = rospy.get_param("~start_file_name", 'start-1.csv')

        Path_Generator = PathGenerator(source_topic, target_topic, plan_topic, good_file_name, bad_file_name, start_file_name)
        
         # Publish markers for visualization
        rate = rospy.Rate(1)  # Publish markers every 1 second
        while not rospy.is_shutdown():
            Path_Generator.publish_markers()
            rate.sleep()
        
        rospy.spin()