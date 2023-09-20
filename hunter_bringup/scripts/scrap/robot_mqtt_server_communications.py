#!/usr/bin/env python3

import paho.mqtt.client as mqtt
import os, json, msgpack

import rospy
import actionlib
from rospy_message_converter import message_converter

from std_msgs.msg import String
import topological_navigation_msgs.msg
from gofar_navigation.msg import NewAgentConfig
from topological_navigation_msgs.msg import ExecutePolicyModeGoal

class MqttPsuedoBridge(object):

    def __init__(self):
        # Define all the details for the MQTT broker
        self.robot_name = os.getenv('ROBOT_NAME', "gofar_001")
        self.mqtt_ip = os.getenv('MQTT_BROKER_IP', '10.8.0.99')
        self.mqtt_port = int(os.getenv('MQTT_BROKER_PORT', 8883))
        self.mqtt_encoding = os.getenv('MQTT_ENCODING', 'msgpack')
        mqtt_client = None
        #self.mqtt_encoding = "json"
        # Specify the loading and dumping functions
        self.dumps = msgpack.dumps if self.mqtt_encoding == 'msgpack' else json.dumps
        self.loads = msgpack.loads if self.mqtt_encoding == 'msgpack' else json.loads

        # Define topics to connect with (TODO: move this to read from yaml config file)
        self.topics = {
            'topological_map_2': {
                'method':'subscribe',
                'namespace_robot': '/',
                'namespace_mqtt': '',
                'namespace_server': '/',
                'type': String
            },
            'rasberry_coordination/dynamic_fleet/add_agent': {
                'method':'publish',
                'namespace_robot': '/',
                'namespace_mqtt': '',
                'namespace_server': '/',
                'type': NewAgentConfig
            },
            f'execute_policy_mode/goal': {
                'method':'subscribe',
                'namespace_robot': '/',
                'namespace_mqtt': f'{self.robot_name}/',
                'namespace_server': f'/{self.robot_name}/',
                'type': ExecutePolicyModeGoal,
            },
            f'current_node': {
                'method':'publish',
                'namespace_robot': '/',
                'namespace_mqtt': f'{self.robot_name}/',
                'namespace_server': f'/{self.robot_name}/',
                'type': String
            },
            f'closest_node': {
                'method':'publish',
                'namespace_robot': '/',
                'namespace_mqtt': f'{self.robot_name}/',
                'namespace_server': f'/{self.robot_name}/',
                'type': String
            }
        }
        self.mqtt_topics = dict()

        # Initiate connections to ROS and MQTT
        self.connect_to_mqtt()
        self.connect_to_ros()

    # MQTT management functions
    def connect_to_mqtt(self):
        self.mqtt_client = mqtt.Client("robot_" + self.robot_name)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect(self.mqtt_ip, self.mqtt_port)
        self.mqtt_client.loop_start()

    # When MQTT connects, subscribe to all relevant MQTT topics
    def on_connect(self, client, userdata, flags, rc):
        print(f"MQTT | Connected")
        for topic, topic_details in self.topics.items():
            if topic_details['method'] == 'subscribe':
                mqtt_topic = topic_details['namespace_mqtt']+topic
                print(f"MQTT | subscribing to: {topic} on {mqtt_topic}")
                self.mqtt_topics[mqtt_topic] = topic
                self.mqtt_client.subscribe(mqtt_topic)

    def on_message(self, client, userdata, msg):
        # Identify topic
        print(f"MQTT | Message received [{msg.topic}]")
        if msg.topic != 'topological_map_2': print(f"MQTT |     payload: {msg.payload}")

        # Get details of topic
        topic = self.mqtt_topics[msg.topic]
        topic_info = self.topics[topic]
        msg_type = topic_info['type']

        #Convert bytearray to msg
        data = self.loads(msg.payload)
        rosmsg_data = msg_type(str(data))
        self.ros_topics[topic].publish(rosmsg_data)

    def connect_to_ros(self):
        # Define publishers and subscribers to ROS
        self.ros_topics = dict()
        for topic, topic_details in self.topics.items():
            mqtt_topic = topic_details['namespace_mqtt'] + topic
            ros_topic = topic_details['namespace_robot'] + topic
            if topic_details['method'] == 'subscribe':
                print(f" ROS | publishing to: {ros_topic}")
                self.ros_topics[topic] = rospy.Publisher(ros_topic, topic_details['type'], queue_size=10)
            if topic_details['method'] == 'publish':
                print(f" ROS | subscribing to: {ros_topic}")
                self.ros_topics[topic] = rospy.Subscriber(ros_topic, topic_details['type'], self.ros_cb, callback_args=mqtt_topic)

    # Our subscribers to get data from the local ROS and into the MQTT broker
    def ros_cb(self, msg, callback_args):
        print(f"MQTT | publishing on {callback_args}")
        data = bytearray(self.dumps(message_converter.convert_ros_message_to_dictionary(msg)))
        self.mqtt_client.publish(callback_args, data)



if __name__ == '__main__':
    rospy.init_node('robot_mqtt_server_communications')
    mpb = MqttPsuedoBridge()
    rospy.spin()

