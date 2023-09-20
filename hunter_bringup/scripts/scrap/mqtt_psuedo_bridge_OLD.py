#!/usr/bin/env python3

import os
import paho.mqtt.client as mqtt
import json, msgpack, yaml

import rospy
from rospy_message_converter import message_converter

from std_msgs.msg import String
import topological_navigation_msgs.msg
from topological_navigation_msgs.msg import ExecutePolicyModeGoal

try:
    from rasberry_coordination.msg import NewAgentConfig
except:
    from gofar_navigation.msg import NewAgentConfig


class MqttPsuedoBridge(object):

    def __init__(self):
        # Define all the details for the MQTT broker
        self.robot_name = os.getenv('ROBOT_NAME', "robbie")
        self.mqtt_ip = os.getenv('MQTT_BROKER_IP', '10.8.0.99')
        self.mqtt_port = int(os.getenv('MQTT_BROKER_PORT', 8883))
        self.mqtt_encoding = os.getenv('MQTT_ENCODING', 'msgpack')
        self.local_namespace = 'namespace_robot' if self.robot_name else 'namespace_server'
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
        self.ros_topics = dict()

        # Initiate connections to ROS and MQTT
        self.connect_to_mqtt()
        self.connect_to_ros()


    def connect_to_mqtt(self):
        # MQTT management functions
        self.mqtt_client = mqtt.Client("robot_" + self.robot_name)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect(self.mqtt_ip, self.mqtt_port)
        self.mqtt_client.loop_start()


    def on_connect(self, client, userdata, flags, rc):
        # When MQTT connects, subscribe to all relevant MQTT topics
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
        if self.local_namespace == 'namespace_server':
            # On the server, everything not agent-specific is already being published, so we only need to subscribe to the agent namespace topics
            self.agent_sub = rospy.Subscriber('/rasberry_coordination/dynamic_fleet/add_agent', NewAgentConfig, self.agent_cb)
            return

        elif self.local_namespace == 'namespace_robot':
            # On the robot, everything must be subscribed/published to
            self.ros_topics = dict()
            for topic, topic_details in self.topics.items():

                # Define relevent topics
                mqtt_topic = topic_details['namespace_mqtt'] + topic
                ros_topic = topic_details['namespace_robot'] + topic

                # Subscribing to MQTT messages (to publish to MQTT later)
                if topic_details['method'] == 'subscribe':
                    print(f" ROS | publishing to: {ros_topic}")
                    self.ros_topics[topic] = rospy.Publisher(ros_topic, topic_details['type'], queue_size=10)

                # Subscribing to ROS messages (to publish to MQTT later)
                if topic_details['method'] == 'publish':
                    print(f" ROS | subscribing to: {ros_topic}")
                    self.ros_topics[topic] = rospy.Subscriber(ros_topic, topic_details['type'], self.ros_cb, callback_args=mqtt_topic)


    def agent_cb(self, msg):
        # Skip if agent exists
        if msg.agent_id in self.agents: return

        # Otherwise subscribe
        for topic, topic_details in self.topics:
            mqtt_topic = topic_details['namespace_mqtt'] + topic
            ros_topic = topic_details['namespace_server'] + topic

            # Subscribing to MQTT messages (to publish to MQTT later)
            if topic_details['method'] == 'subscribe':
                print(f" ROS | publishing to: {ros_topic}")
                self.ros_topics[topic] = rospy.Publisher(ros_topic, topic_details['type'], queue_size=10)
                self.mqtt_client.subscribe(mqtt_topic)

            # Subscribing to ROS messages (to publish to MQTT later)
            if topic_details['method'] == 'publish':
                print(f" ROS | subscribing to: {ros_topic}")
                self.ros_topics[topic] = rospy.Subscriber(ros_topic, topic_details['type'], self.ros_cb, callback_args=mqtt_topic)



    # Our subscribers to get data from the local ROS and into the MQTT broker
    def ros_cb(self, msg, callback_args):
        print(f"MQTT | publishing on {callback_args}")
        data = bytearray(self.dumps(message_converter.convert_ros_message_to_dictionary(msg)))
        self.mqtt_client.publish(callback_args, data)






if __name__ == '__main__':
    rospy.init_node('mqtt_psuedo_bridge')
    mpb = MqttPsuedoBridge()
    rospy.spin()
