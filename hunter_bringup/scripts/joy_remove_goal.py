#!/usr/bin/env python
### press button on controller to remove move base goal
###
import rospy
from math import sqrt
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped


class JoyRemoveGoal:
    """
    Recieves twist messages and prints to screen (send the movement messages to the vechicle controller via TCP).
    (Recieves odometry messages from vechicle controller via TCP and publishes them to odom topic.)
    """

    def __init__(self):
        """
        On construction of the object, create a Subscriber to listen to twist messages.
        """
        self.Subscriber_Joy = rospy.Subscriber("joy_teleop/joy", Joy, self.callback_joy)

        self.cancel_pub = rospy.Publisher("/move_base/cancel", actionlib.GoalID, queue_size=1)

        # For movebase
        # https://github.com/AutonomousRobotics/ROSNutshell/wiki/Try-Move-Base-with-Navigation-Stack
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        #self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        # Waits until the action server has started up and started listening for goals.
        #self.client.wait_for_server()

    # https://github.com/AutonomousRobotics/ROSNutshell/wiki/Try-Move-Base-with-Navigation-Stack
    # https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
    # https://answers.ros.org/question/330776/how-to-send-a-new-goal-movebasegoal-without-interrupting-the-node-python/
    def set_movebaseGoal(self,target_list):
        """
        Input x, y, z, of goal point, returns in move_base format.
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = target_list[0]
        goal.target_pose.pose.position.y = target_list[1]
        goal.target_pose.pose.position.z = target_list[2]

        goal.target_pose.pose.orientation.w = 1.0
        return goal

    def callback_joy(self, data):
        """
        Callback called any time new joy mesasages becomes available.
        For Logitech 710 Axes and Buttons are:
        axes[0-7]:
            [0] = Dpad +left / -right
            [1] = Dpad +up / -down
            [2] = LT +1 to -1, -1 is pressed down
            [3] = right stick +left / -right
            [4] = right stick +up / -down
            [5] = RT +1 to -1, -1 is pressed down
            [6] = left stick +left / -right
            [7] = left stick +left / -right

        buttons[0-10]: (0 = not pressed / 1 = pressed)
            [0] = A
            [1] = B
            [2] = X
            [3] = Y
            [4] = LB
            [5] = RB
            [6] = BACK
            [7] = START
            [8] = LOGITECH Button
            [9] = left stick button
            [10] = right stick button

        8BitDo Pro2 Axes and Buttons:
            axes[0-7]:
            [0] = left stick +left / -right
            [1] = left stick +up / -down
            [2] = right stick +left / -right
            [3] = right stick +up / -down
            [4] = R2
            [5] = L2
            [6] = Dpad +left / -right
            [7] = Dpad +up / -down

        buttons[0-15]: (0 = not pressed / 1 = pressed)
            [0] = A
            [1] = B
            [2] = ???
            [3] = X
            [4] = Y
            [5] = ???
            [6] = L
            [7] = R
            [8] = ???
            [9] = ???
            [10] = Select
            [11] = Start
            [12] = Turbo (checker board button)
            [13] = left stick button
            [14] = right stick button
        """
        # remove current move_base goal point
        # https://answers.ros.org/question/340635/movebaseaction-cancel_goal-does-not-work-from-python-script/
        if data.buttons[1] == 1: # press B to remove current move_base goal point
            print("Removing current move_base goal point")
            cancel_msg = actionlib.GoalID()
            self.cancel_pub.publish(cancel_msg)

if __name__ == '__main__':
    rospy.init_node('joy_remove_goal')
    JoyRemoveGoal()
    rospy.spin()

#### set move_base goal x,y,z;
#movebaseGoal = self.set_movebaseGoal((2,2,0))
#self.client.send_goal(movebaseGoal)
## Waits for the server to finish performing the action.
#wait = self.client.wait_for_result()
## If the result doesn't arrive, assume the Server is not available
#if not wait:
#    rospy.loger("Action server not available!")
#    rospy.signal_shutdown("Action server not available!")
#else:
#    # Result of executing the action
#    print("Goal Reached")