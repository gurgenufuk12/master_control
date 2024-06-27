import paho.mqtt.client as mqtt
import rospy
import json
import ssl

class MQTTClient:
    def __init__(self, client_id, broker_address, port, topic):
        self.client = mqtt.Client(client_id, protocol=mqtt.MQTTv311, userdata=None, transport="tcp", reconnect_on_failure=True)
        self.broker_address = broker_address
        self.port = port
        self.topic = topic
        
        # Set TLS configuration
        #self.client.tls_set(tls_version=ssl.PROTOCOL_TLS)
        
        # Set username and password for authentication
        #self.client.username_pw_set("ufuk2", "Atadogu07!")
        
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def start(self):
        rospy.loginfo(f"Connecting to MQTT broker at {self.broker_address}:{self.port}")
        
        # Connect to the broker
        self.client.connect(self.broker_address, self.port)
        
        # Start the loop
        self.client.loop_start()

        if self.client.is_connected():
            rospy.loginfo("Connected to MQTT broker")
        else:
            rospy.logerr("Failed to connect to MQTT broker")

    def stop(self):
        self.client.loop_stop()
        self.client.disconnect()

    def on_connect(self, client, userdata, flags, rc):
        rospy.loginfo(f"Connected with result code {rc}")
        if rc == 0:
            client.subscribe(self.topic)
        else:
            rospy.logerr(f"Failed to connect, return code {rc}")


    def on_message(self, client, userdata, msg):
        rospy.loginfo(f"Received message: {msg.topic} {str(msg.payload)}")
        data = json.loads(msg.payload.decode())
        rospy.loginfo(f"Received data: {data}")

    def publish(self, topic,payload):
        rospy.loginfo(f"Attempting to publish to topic {topic}: {payload}")
        try:
            result = self.client.publish(topic, payload)
            rospy.loginfo(f"Publish result: {result.rc}")
        except Exception as e:
            rospy.logerr(f"Error publishing to topic {topic}: {e}")
