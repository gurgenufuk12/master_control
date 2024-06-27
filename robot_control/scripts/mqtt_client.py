import paho.mqtt.client as mqtt
import rospy
import json
import ssl

class MQTTClient:
    def __init__(self, client_id, broker_address, port, topic):
        self.client = mqtt.Client(client_id)
        self.broker_address = broker_address
        self.port = port
        self.topic = topic
        
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def start(self):
        rospy.loginfo(f"Connecting to MQTT broker at {self.broker_address}:{self.port}")
        self.client.connect(self.broker_address, self.port)
        self.client.loop_start()

    def stop(self):
        self.client.loop_stop()
        self.client.disconnect()

    def on_connect(self, client, userdata, flags, rc):
        rospy.loginfo(f"Connected with result code {rc}")
        if rc == 0:
            client.subscribe(self.topic)
            rospy.loginfo(f"Subscribed to topic: {self.topic}")
        else:
            rospy.logerr(f"Failed to connect, return code {rc}")

    def on_message(self, client, userdata, msg):
        rospy.loginfo(f"Received message: {msg.topic} {str(msg.payload)}")

    def publish(self, topic, payload):
        rospy.loginfo(f"Publishing to topic {topic}: {payload}")
        try:
            result = self.client.publish(topic, payload)
            rospy.loginfo(f"Publish result: {result.rc}")
        except Exception as e:
            rospy.logerr(f"Error publishing to topic {topic}: {e}")

    def subscribe(self, topic):
        self.client.subscribe(topic)
        rospy.loginfo(f"Subscribed to topic: {topic}")
