import rospy
import numpy as np
import pandas as pd
import os

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, PoseArray

class PathGenerator:

    def __init__(source_topic, target_topic, plan_topic, file_name):

        self.plan = None
        self.next_wpt = 0

        self.wpts = self.read_csv(file_name) # 'good_waypoints-1.csv'

        self.source_pub = rospy.Publisher(source_topic, PoseWithCovarianceStamped, queue_size=10) # Create a publisher to source_topic
        self.target_pub = rospy.Publisher(target_topic, PoseStamped, queue_size=10) # Create a publisher to target_topic
        self.plan_sub = rospy.Subscriber(plan_topic, PoseArray, self.plan_cb) # Create a subscriber to plan_topic, with callback 'self.plan_cb'

    
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
        if self.next_wpt < len(self.wpts)
            pose = Pose()
            pose.position.x, pose.position.y = wpts[next_wpt]
            pose.position.z = 0.0
            pose.orientation = Utils.angle_to_quaternion(0.0)
            source_pub.publish(pose)

            self.next_wpt += 1
            pose.position.x, pose.position.y = wpts[next_wpt]
            target_pub.publish(pose)

    
    #start_point = self.read_csv('start-1.csv')[0] # (x,y)
    #good_waypoints = self.read_csv('good_waypoints-1.csv') # [(x0,y0), ..., (xn,yn)]
    #bad_waypoints = self.read_csv('bad_waypoints-1.csv') # [(x0,y0), ..., (xn,yn)]

    if __name__ == '__main__':
        rospy.init_node('line_follower', anonymous=True) # Initialize the node

        plan_topic = rospy.get_param("~plan_topic", '/planner_node/car_plan')
        source_topic = rospy.get_param("~source_topic", '/initialpose')
        target_topic = rospy.get_param("~target_topic", '/move_base_simple/goal')
        file_name = rospy.get_param("~file_name", 'good_waypoints-1.csv')

        Path_Generator = PathGenerator(source_topic, target_topic, plan_topic, file_name)
    