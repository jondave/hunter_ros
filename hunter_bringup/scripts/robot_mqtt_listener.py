#!/usr/bin/env python3

import paho.mqtt.client as mqtt # pip3 install paho-mqtt
import os, json
import rospy
from std_msgs.msg import String
import actionlib
import topological_navigation_msgs.msg

#=========================================
# Receives MQTT messages containing the location of the human that requires the robot-runner
#   and sets those locations as goals for the robot.
class MqttCommandRetriever:

    ROBOT_ID = os.getenv('ROBOT_ID', "gofar") # ENVIRONMENT VARIABLE
    MQTT_BROKER_IP = 'mqtt.lcas.group'
    #MQTT_BROKER_IP = '10.101.8.31'
    #MQTT_BROKER_IP = 'tuf1'
    MQTT_BROKER_PORT = 1883

    mqtt_client = None
    goal_pub = None

    def __init__(self):  #setup_connections(self)
        self.connect_to_ros()
        self.connect_to_mqtt()


    #------

    def connect_to_mqtt(self):
        self.mqtt_client = mqtt.Client("robot_" + self.ROBOT_ID) # client ID "mqtt-test"
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        #client.username_pw_set("myusername", "aeNg8aibai0oiloo7xiad1iaju1uch")
        self.mqtt_client.connect(self.MQTT_BROKER_IP, self.MQTT_BROKER_PORT)
        self.mqtt_client.loop_start()  # Start networking daemon. OR USE: .loop_start()

    #------

    def connect_to_ros(self):
        #self.goal_pub = rospy.Publisher('/topological_navigation/goal', String, queue_size=10)
        self.top_nav_client = actionlib.SimpleActionClient("topological_navigation", topological_navigation_msgs.msg.GotoNodeAction)
        rospy.init_node('mqtt_command_listener', anonymous=True)

    #------

    ####
    # When this client connects to mqtt, subscribe to the robot's topic
    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected with result code {rc}")
        # subscribe:
        self.mqtt_client.subscribe("/uol/cofruit/" + self.ROBOT_ID + "/task")

    #------

    def on_message(self, client, userdata, msg):
        print(f"Message received [{msg.topic}]: {msg.payload}")
        msg_content = json.loads(msg.payload)
        node_id = msg_content["picker_node_location"]
        self.navigate_to_node(node_id)

    #------

    def navigate_to_node(self, node_id):
        # convert to Type: topological_navigation_msgs/GotoNodeActionGoal
        '''
        #goal definition
        string target
        bool   no_orientation	#Do not care about the final orientation
        ---
        #result definition
        bool success
        ---
        #feedback
        string route
        '''
        navgoal = topological_navigation_msgs.msg.GotoNodeGoal()
        navgoal.target = node_id
        print("Publish Goal: " + str(node_id))
        #self.goal_pub.publish(navgoal)
        self.top_nav_client.send_goal(navgoal)
        self.top_nav_client.wait_for_result()
        result = self.top_nav_client.get_result()
        print("RESULT",result)
        if result.success:
            #print("SUCCESS")
            # success got to node
            # todo send back weight data from scales
            data = {}
            data["picker_node_location"] = node_id
            data["status"] = 'reached_picker_node'
            print("Reached Goal")
            json_data = json.dumps(data)
            self.mqtt_client.publish("/uol/cofruit/" + self.ROBOT_ID + "/task_status", json_data)

#=========================================
if __name__ == '__main__':
    try:
        mqttCommandRetriever = MqttCommandRetriever()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#=========================================

