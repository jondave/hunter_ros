#!/usr/bin/env python

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
        self.mqtt_ip = os.getenv('MQTT_BROKER_IP', '10.8.0.99')
        self.mqtt_port = int(os.getenv('MQTT_BROKER_PORT', 8883))
        self.mqtt_encoding = os.getenv('MQTT_ENCODING', 'msgpack')
        mqtt_client = None

        # Specify the loading and dumping functions
        self.dumps = msgpack.dumps if self.mqtt_encoding == 'msgpack' else json.dumps
        self.loads = msgpack.loads if self.mqtt_encoding == 'msgpack' else json.loads

        # Define source information
        self.robot_name = os.getenv('ROBOT_NAME', '')
        self.source = 'robot' if self.robot_name else 'server'
        self.local_namespace = 'namespace_'+self.source

        # Define topics to connect with (TODO: move this to read from yaml config file)
        self.topics = {
            'topological_map_2': {
                'source':'server',
                'namespace_robot': '/',
                'namespace_mqtt': '',
                'namespace_server': '/',
                'type': String
            },
            'rasberry_coordination/dynamic_fleet/add_agent': {
                'source':'robot',
                'namespace_robot': '/',
                'namespace_mqtt': '',
                'namespace_server': '/',
                'type': NewAgentConfig
            },
            'execute_policy_mode/goal': {
                'source':'server',
                'namespace_robot': '/',
                'namespace_mqtt': self.robot_name+'<<rn>>/',
                'namespace_server': '/'+self.robot_name+'<<rn>>/',
                'type': ExecutePolicyModeGoal,
            },
            'current_node': {
                'source':'robot',
                'namespace_robot': '/',
                'namespace_mqtt': self.robot_name+'<<rn>>/',
                'namespace_server': '/'+self.robot_name+'<<rn>>/',
                'type': String
            },
            'closest_node': {
                'source':'robot',
                'namespace_robot': '/',
                'namespace_mqtt': self.robot_name+'<<rn>>/',
                'namespace_server': '/'+self.robot_name+'<<rn>>/',
                'type': String
            }
        }
        self.mqtt_topics = dict()
        self.ros_topics = dict()
        self.agents = []

        # Initiate connections to ROS and MQTT
        self.connect_to_mqtt()
        self.connect_to_ros()


    def connect_to_mqtt(self):
        # MQTT management functions
        self.mqtt_client = mqtt.Client(self.source + "_" + self.robot_name)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect(self.mqtt_ip, self.mqtt_port)
        self.mqtt_client.loop_start()


    def on_connect(self, client, userdata, flags, rc):
        # When MQTT connects, subscribe to all relevant MQTT topics
        print(" MQTT ->     | Connected")
        for topic, topic_details in self.topics.items():

            # We dont want to subscribe if the message it local
            if topic_details['source'] == self.source: continue

            # Skip if topic uses agent namespacing
            if topic_details['namespace_server'] == '/<<rn>>/': continue

            #Subscribe mqtt stream to this topic
            mqtt_topic = topic_details['namespace_mqtt'].replace('<<rn>>/', '/') + topic
            print(" MQTT ->     | subscribing to " + mqtt_topic)
            self.mqtt_topics[mqtt_topic] = topic
            self.mqtt_client.subscribe(mqtt_topic)
            #TODO: we could skip this and only connect on agent load? for the agent-specific topics only?

    def on_message(self, client, userdata, msg):
        # Identify topic
        print(" MQTT        | Message received ["+msg.topic+"]")
        if msg.topic != 'topological_map_2': print(" MQTT      |     payload: "+msg.payload)

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
        if self.source == 'server':
            # On the server, everything not agent-specific is already being published/subscribed
            # so we only need to subscribe to the agent namespace topics, but need the name to do so
            self.agent_sub = rospy.Subscriber('/rasberry_coordination/dynamic_fleet/add_agent', NewAgentConfig, self.agent_cb)
            return

        elif self.source == 'robot':
            # On the robot, everything must be subscribed/published to
            for topic, topic_details in self.topics.items():
                self.make_topic_connection(topic, topic_details, replace_with="/")

    def agent_cb(self, msg):
        # Skip if agent exists
        if msg.agent_id in self.agents: return
        self.agents += [msg.agent_id]
        print('       AGENT | New agent detected: ' + msg.agent_id)

        # Otherwise subscribe
        for topic, topic_details in self.topics.items():

            # Skip if topic doesnt use agent namespacing
            if topic_details['namespace_server'] == '/': continue

            # Construct topics
            self.make_topic_connection(topic, topic_details, replace_with=msg.agent_id+'/', sub_to_mqtt=True)

    # Our subscribers to get data from the local ROS and into the MQTT broker
    def ros_cb(self, msg, callback_args):
        print(" MQTT        | publishing on "+callback_args)
        data = bytearray(self.dumps(message_converter.convert_ros_message_to_dictionary(msg)))
        self.mqtt_client.publish(callback_args, data)




    def make_topic_connection(self, topic, topic_details, replace_with, sub_to_mqtt=False):
        # Define relevent topics
        mqtt_topic = topic_details['namespace_mqtt'].replace('<<rn>>/', replace_with) + topic
        ros_topic = topic_details[self.local_namespace].replace('<<rn>>/', replace_with) + topic

        # Subscribing to MQTT messages (to publish to MQTT later)
        # On the robot, if the source is the server, we sub to MQTT and pub through to ROS
        # On the server, if the source is the robot, we sub to MQTT and pub through to ROS
        if topic_details['source'] != self.source:
            print(" MQTT -> ROS | publishing from " + mqtt_topic + " to " + ros_topic)
            self.ros_topics[topic] = rospy.Publisher(ros_topic, topic_details['type'], queue_size=10)
            if sub_to_mqtt:
                self.mqtt_client.subscribe(mqtt_topic)

        # Subscribing to ROS messages (to publish to MQTT later)
        # On the server, if the source is the server, we sub to ROS and pub through to MQTT
        # On the  robot, if the source is the  robot, we sub to ROS and pub through to MQTT
        if topic_details['source'] == self.source:
            print(" ROS -> MQTT | publishing from " + ros_topic + " to " + mqtt_topic)
            self.ros_topics[topic] = rospy.Subscriber(ros_topic, topic_details['type'], self.ros_cb, callback_args=mqtt_topic)







if __name__ == '__main__':
    rospy.init_node('mqtt_psuedo_bridge')
    mpb = MqttPsuedoBridge()
    rospy.spin()
