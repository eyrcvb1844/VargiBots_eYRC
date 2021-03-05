#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
import rosservice

from std_msgs.msg import String
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from importlib import import_module
from pkg_vb_sim.msg import LogicalCameraImage
from pkg_vb_sim.msg import Model

cnt=0

class convey:
    '''
        This is the class for controlling the conveyor belt.

        It starts the conveyor when there is no package under the logical_camera_2.
        When a package comes under the camera, it stops the conveyor and sends a message on the topic 'check_box'

        :param pub: Publisher handle of the topic 'check_box'
    '''

    def __init__(self):
        '''
            This is the contructor for convey class.
            It initializes the node 'convey' and the publisher of the topic 'check_box'.
        '''

        rospy.init_node('convey', anonymous=True)
        self.pub = rospy.Publisher('check_box', String, queue_size=1)

    def fast_conveyor(self):
        '''
            This function runs the conveyor belt at full speed, 100.
            
            :return: convey(100): Sets the conveyor speed to 100 and returns True.
            :rtype: boolean
        '''

        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        convey = rospy.ServiceProxy(
            '/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
        return convey(100)

    def stop_conveyor(self):
        '''
            This function stops the conveyor belt.
        
            :return: convey(0): Sets the conveyor speed to 0 and returns True.
            :rtype: boolean
        '''

        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        convey = rospy.ServiceProxy(
            '/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
        return convey(0)

    def callback_logical_cam(self, message):
        '''
            This is the callback function of the subscriber of logical_camera_2.
            It detects if the package comes in the middle of the camera's range (i.e. y coordinate is around 0) and stops the conveyor.
            After the conveyor is stopped it sends a message in the form "<box no.> + <True>" to the topic 'check_box'.

            :param message: The LogicalCameraImage sent by logical_camera_2
            :type message: LogicalCameraImage
        '''

        global flag2,cnt
        try:
            if(message.models[-1].type != "ur5"):
                y = message.models[-1].pose.position.y
                if(y > 0.5):
                    flag2 = True

                if(y < 0.05 and y > -0.05 and flag2):
                    string= str(cnt)+"True"
                    self.stop_conveyor()
                    self.pub.publish(string)
                    rospy.sleep(2.5)
                    self.fast_conveyor()
                    flag2 = False
                    cnt+=1
                    pass

                if((not flag2) and message.models[0] == "x"):
                    pass

        except Exception:
            pass

    def __del__(self):
        moveit_commander.roscpp_shutdown()


def main():
    '''
        It is the main function which creates an object of convey class, starts the conveyor and keeps the node running.
        It initializes the subscriber of logical_camera_2 which receives the Logical image from the camera.
    '''
    conveyor = convey()
    rospy.sleep(3.0)
    conveyor.fast_conveyor()
    rospy.Subscriber('/eyrc/vb/logical_camera_2',
                     LogicalCameraImage, conveyor.callback_logical_cam, queue_size=1, buff_size=2**24)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
