from numpy.core.numeric import full
import rclpy
from rclpy.node import Node
import ros2pkg
from geometry_msgs.msg import Twist,PoseStamped,Pose,TransformStamped
from ssafy_msgs.msg import TurtlebotStatus, EnviromentStatus
from sensor_msgs.msg import Imu,LaserScan
from std_msgs.msg import Float32
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path,OccupancyGrid,MapMetaData
from math import pi,cos,sin,sqrt
import tf2_ros
import os
import hanbi.utils as utils
import numpy as np
import cv2
import time
import socketio



info = {
    "robot" : {
        "pos" : [],
        "velocity" : 0,
        "battery" : 100,
        "mode" : 0
    },
    "environment" : {
        "temperature" : 30,
        "weather" : "Cloudy"
    }
}

params_map = {
    "MAP_RESOLUTION": 0.1,
    "OCCUPANCY_UP": 0.02,
    "OCCUPANCY_DOWN": 0.01,
    "MAP_CENTER": (-50, -50),
    "MAP_SIZE": (50, 50),
    "MAP_FILENAME": 'test.png',
    "MAPVIS_RESIZE_SCALE": 2.0,
    "origin":[]
}

params_map["origin"] = [params_map["MAP_CENTER"][0]-(params_map["MAP_SIZE"][0]/2), params_map["MAP_CENTER"][1]-(params_map["MAP_SIZE"][1]/2)]
def pose2grid(pose):
    [x, y] = pose
    return  [int((x - params_map["origin"][0])/params_map["MAP_RESOLUTION"]),int((y - params_map["origin"][1])/params_map["MAP_RESOLUTION"])]

    pass
def grid2pose(grid):
    pass
class SocketClass(Node):

    def __init__(self):
        super().__init__('Mapper')
        
        self.map_sub = self.create_subscription(OccupancyGrid,'map',self.map_callback,1)
        self.odom_sub = self.create_subscription(Odometry,'odom',self.odom_callback,1)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose',1)
        self.env_sub = self.create_subscription(EnviromentStatus,'/envir_status',self.env_callback,1)        
        self.turtle_sub = self.create_subscription(TurtlebotStatus,
        '/turtlebot_status',self.turtlebot_status_callback,1)
        # self.cmd_sub = self.create_subscription(Twist,'cmd_vel',self.cmd_callback,10)
        self.timer = self.create_timer(0.2, self.timer_callback)

        #?????? ????????? ??????
        self.sio = socketio.Client()
        @self.sio.event
        def connect():
            print('connection established')
            
        @self.sio.event
        def disconnect():
            print('disconnected from server')

        @self.sio.on("Click2Ros")
        def listen_node(data):
            print("clicked : ", data['x'], data['y'])
            goal_pose_msg = PoseStamped()
            goal_pose_msg.header.frame_id = "map"
            goal_pose_msg.pose.position.x = round(float(data['x']), 2)
            goal_pose_msg.pose.position.y = round(float(data['y']), 2)
            self.goal_pub.publish(goal_pose_msg)

        #?????? ??????
        ip_server = 'http://localhost:3000/'

        self.sio.connect(ip_server)
        

    def turtlebot_status_callback(self, msg):
        x = msg.twist.angular.x
        y = msg.twist.angular.y
        ret = pose2grid([x, y])
        # print(ret)
        info["robot"]["pos"] =ret
        info["robot"]["velocity"] = msg.twist.linear.x

    def map_callback(self, msg):
        pass
    def odom_callback(self, msg):
        pass
    def goal_callback(self, msg):
        pass

    def env_callback(self, msg):
        info["environment"]["weather"] = msg.weather
        info["environment"]["temperature"] = msg.temperature
        pass

    def timer_callback(self):
        self.sio.emit("SocketNode2Server", info)

    # def cmd_callback(self, data):
    #     linearVelocity = data.linear.x
    #     angularVelocity = data.angular.z
    #     info["robot"]["velocity"] = linearVelocity
        

def main(args=None):    
    rclpy.init(args=args)
    
    
    run_mapping = SocketClass()
    rclpy.spin(run_mapping)
    run_mapping.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()