#!/usr/bin/env python3

import threading
import rospy
import json
import websocket
from pymongo import MongoClient
from mqtt_client import MQTTClient
from std_msgs.msg import String

class FleetManager:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('fleet_manager_node', anonymous=True)

        self.client = MongoClient("mongodb://localhost:27017/")
        self.db = self.client['altinay_amr_fleet']
        self.robot_collection = self.db['robots'] 
        self.task_collection = self.db['tasks']

        # Initialize MQTT client
        self.mqtt_client = MQTTClient("fleet_manager", "localhost", 1883, "deneme")
        self.mqtt_client.client.on_message = self.on_mqtt_message
        self.mqtt_client.client.on_connect = self.on_mqtt_connect

        self.ws = websocket.WebSocketApp("ws://localhost:8000", on_message=self.on_ws_message)
        self.ws_thread = threading.Thread(target=self.ws.run_forever)
        self.mqtt_client.start()
        self.ws_thread.start()

        
        rospy.on_shutdown(self.cleanup)

    def on_mqtt_connect(self, client, userdata, flags, rc):
        rospy.loginfo(f"Connected with result code {rc}")
        if rc == 0:
            self.mqtt_client.client.subscribe(self.mqtt_client.topic)
        else:
            rospy.logerr(f"Failed to connect, return code {rc}")

    def cleanup(self):
        rospy.loginfo("Shutting down fleet manager node")
        self.mqtt_client.stop()

    def on_ws_message(self, ws, message):
        data = json.loads(message)
        if 'type' in data and data['type'] == 'new_task':
            task_data = data['data']
            rospy.loginfo("Received new task: websocket message")
            self.assign_task_to_robot(task_data)

    def on_mqtt_message(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())
        rospy.loginfo(f"Received data: true")

    def assign_task_to_robot(self, task_data):
        idle_robots = list(self.robot_collection.find({"robotStatus": "Idle"}))
        for robot in idle_robots:
            rospy.loginfo(f"Robot Name: {robot['robotName']} Robot Status: {robot['robotStatus']}")

        if idle_robots:
            chosen_robot = idle_robots[0]
            rospy.loginfo(f"Assigning task to robot: {chosen_robot['robotName']}")
            updated_robot = {
            "Task": {
                "taskCode": task_data['Task']['taskCode'],
                "taskName": task_data['Task']['taskName'],
                "taskId": task_data['Task']['taskId'],
                "taskPriority": task_data['Task']['taskPriority'],
                "taskPercentage": "0",
                "pathPoints": [], 
            },
            "Targets": task_data['Targets'],
            "robotStatus": "Task In Progress",
            }

            updated_Task ={
                "robotName": chosen_robot['robotName'],
            }

            rospy.loginfo(f"Task to be updated: {task_data['Task']['taskId']}")
            task_id = task_data['Task']['taskId']

            robot_update_result= self.robot_collection.update_one(
                {"robotName": chosen_robot['robotName']},
                {"$set": updated_robot}
            )
            if robot_update_result.matched_count > 0:
                rospy.loginfo(f"Successfully updated robot {chosen_robot['robotName']} with new task")
            else:
                rospy.logerr(f"Failed to update robot {chosen_robot['robotName']} with new task")

            task_update_result = self.task_collection.update_one(
                {"Task.taskId": task_id},
                {"$set": updated_Task}
            )
            if task_update_result.matched_count > 0:
                rospy.loginfo(f"Successfully updated task with taskId: {task_id}")
            else:
                rospy.logerr(f"Failed to update task with taskId: {task_id}. Task not found.")

            task_message = {
                "task": task_data,
                "robot": chosen_robot['robotName']
            }
            rospy.loginfo(f"Task message to be published: {task_message}")
            self.mqtt_client.publish(topic="deneme", payload=json.dumps(task_message))
        else:
            rospy.loginfo("No idle robots available to assign the task")

    def start_node(self):
        rospy.loginfo("Starting fleet manager node")
        rospy.spin()

if __name__ == '__main__':
    try:
        fleet_manager = FleetManager()
        fleet_manager.start_node()
    except rospy.ROSInterruptException:
        pass
