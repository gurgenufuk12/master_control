#!/usr/bin/env python3

import rospy
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path, OccupancyGrid
import json
import numpy as np
from pymongo import MongoClient
from mqtt_client import MQTTClient

class RobotControl:
    def __init__(self, mqtt_client, robot_name):
        self.mqtt_client = mqtt_client
        self.robot_name = robot_name
        self.mqtt_client.client.on_message = self.on_mqtt_message
        self.mqtt_client.client.on_connect = self.on_mqtt_connect

        self.mqtt_client.client.subscribe("deneme")
        self.total_points = 0
        self.robot_x, self.robot_y = (0, 0)
        self.path_list = []

        self.listener = tf.TransformListener()
        self.mqtt_client.start()
        rospy.on_shutdown(self.cleanup)

    def on_mqtt_message(self, client, userdata, msg):
        rospy.loginfo(f"MQTT message received on topic {msg.topic}: {msg.payload}")
        data = json.loads(msg.payload.decode())
        if 'task' in data and data['robot'] == self.robot_name:
            self.execute_task(data['task'])

    def on_mqtt_connect(self, client, userdata, flags, rc):
        rospy.loginfo(f"Connected with result code {rc}")
        if rc == 0:
            self.mqtt_client.client.subscribe(self.mqtt_client.topic)
        else:
            rospy.logerr(f"Failed to connect, return code {rc}")
            
    def cleanup(self):
        rospy.loginfo("Shutting down fleet manager node")
        self.mqtt_client.stop()

    def execute_task(self, task):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        target_list = task['Targets']
        for i, target in enumerate(target_list):
            if not target.get('targetExecuted', False):
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = float(target['Position']['x'])
                goal.target_pose.pose.position.y = float(target['Position']['y'])
                goal.target_pose.pose.position.z = float(target['Position']['z'])
                goal.target_pose.pose.orientation.x = float(target['Orientation']['x'])
                goal.target_pose.pose.orientation.y = float(target['Orientation']['y'])
                goal.target_pose.pose.orientation.z = float(target['Orientation']['z'])
                goal.target_pose.pose.orientation.w = float(target['Orientation']['w'])

                client.send_goal(goal)
                client.wait_for_result()

                target['targetExecuted'] = True
                progress = (i + 1) / len(target_list) * 100

                update_data = {
                    "robot": self.robot_name,
                    "task_percentage": progress,
                    "path_points": self.path_list,
                    "targets": target_list
                }
                self.mqtt_client.publish("robot_updates", json.dumps({"update": update_data}))

        self.update_robot_status("Idle")

    def update_robot_status(self, status):
        update_data = {
            "robot": self.robot_name,
            "status": status
        }
        self.mqtt_client.publish("robot_updates", json.dumps({"update": update_data}))

    def callback_odom(self, msg):
        posx = str(round(msg.pose.pose.position.x, 12))
        posy = str(round(msg.pose.pose.position.y, 12))
        posz = str.round(msg.pose.pose.position.z, 12)
        orix = str.round(msg.pose.pose.orientation.x, 12)
        oriy = str.round(msg.pose.pose.orientation.y, 12)
        oriz = str.round(msg.pose.pose.orientation.z, 12)
        oriw = str.round(msg.pose.pose.orientation.w, 12)
        odom_data = {
            "Pose.Position.x": posx,
            "Pose.Position.y": posy,
            "Pose.Position.z": posz,
            "Pose.Orientation.x": orix,
            "Pose.Orientation.y": oriy,
            "Pose.Orientation.z": oriz,
            "Pose.Orientation.w": oriw
        }
        update_data = {
            "robot": self.robot_name,
            "odom": odom_data
        }
        self.mqtt_client.publish("robot_updates", json.dumps({"update": update_data}))

        self.robot_x, self.robot_y = float(posx), float(posy)

    def callback_vel(self, msg):
        v_lin = str.round(msg.linear.x, 2)
        v_ang = str.round(msg.angular.z, 2)
        vel_msg = {
            "velocity": {
                "robotVelocity.linearVelocity": v_lin,
                "robotVelocity.angularVelocity": v_ang
            }
        }
        self.mqtt_client.publish("robot_updates", json.dumps({"update": {"robot": self.robot_name, **vel_msg}}))

    def callback_costmap(self, msg):
        length = int(np.sqrt(len(msg.data)))
        msg.data = np.array(msg.data).reshape((length, length))
        costmap_border = 15
        divider = length / costmap_border
        hundreds = []
        unknowns = []
        for i in range(0, msg.data.shape[0]):
            for j in range(0, msg.data.shape[1]):
                if msg.data[i][j] == 100:
                    hundreds.append((self.robot_x + (round((((j / divider) - (costmap_border / 2))), 3)), 
                                     self.robot_y + round(((i / divider) - (costmap_border / 2)), 3)))
                elif msg.data[i][j] == -1:
                    unknowns.append((i, j))
        costmap_data = {"createdCostmap": hundreds}
        update_data = {
            "robot": self.robot_name,
            "costmap": costmap_data
        }
        self.mqtt_client.publish("robot_updates", json.dumps({"update": update_data}))

    def callback_path(self, msg):
        path_points = msg.poses
        
        try:
            self.listener.waitForTransform('/map', '/odom_combined', rospy.Time(), rospy.Duration(4.0))
            (trans, rot) = self.listener.lookupTransform('/map', '/odom_combined', rospy.Time(0))

            path_points = [(k.pose.position.x + trans[0], k.pose.position.y + trans[1]) for k in path_points]
            self.path_list = path_points

            update_data = {
                "robot": self.robot_name,
                "path_points": path_points
            }
            self.mqtt_client.publish("robot_updates", json.dumps({"update": update_data}))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("TF Exception occurred")

    def start_node(self):
        rospy.loginfo(f"Starting robot control node")

        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callback_odom)
        rospy.Subscriber("/cmd_vel", Twist, self.callback_vel)
        rospy.Subscriber('/move_base/DWAPlannerROS/global_plan', Path, self.callback_path)
        rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.callback_costmap)

        rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('robot_control_node')  # Initialize ROS node once
        robot_name = rospy.get_param('~robot_name', 'default_robot')
        mqtt_client = MQTTClient(robot_name, "localhost", 1883, "deneme")
        robot_control = RobotControl(mqtt_client, robot_name)
        robot_control.start_node()
    except rospy.ROSInterruptException:
        pass
