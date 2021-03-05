#! /usr/bin/env python

from actionlib.action_client import GoalManager
import rospy
import os
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
import rosservice
import rospkg
import yaml
import time
import cv2

from std_msgs.msg import String
from sensor_msgs.msg import Image
from importlib import import_module
from cv_bridge import CvBridge, CvBridgeError
from pkg_vb_sim.srv import vacuumGripper
from pyzbar.pyzbar import decode

boxes = list()
colors = list()
cnt = 0


class Actuator2:
    """
        This class recieves whether a package is under the end effector of the ur5_2 arm from the rostopic "check_box", and then the package's colour through the rostopic "/order_shipping".
        It then makes the ur5_2 arm pick up the package place the package in the appropriate bin, and then publishes a shipped message through the rostopic "order_status".


        :param _robot_ns: Name of the ur5 arm
        :param _planning_group: Name of the planning group by the ur5 arm
        :param _commander: initialized moveit_commander object
        :param _robot: moveit_commander.RobotCommander object to get the group names and the current state of the ur5 arm
        :param _scene: moveit_commander.PlanningSceneInterface object to get the Rviz scene of the arm
        :param _group: moveit_commander.MoveGroupCommander object to move the ur5 arm
        :param _display_trajectory_publisher: Publisher handle to publish to /ur5_2/move_group/display_planned_path
        :param _exectute_trajectory_client: Simple Action Client object
        :param _planning_frame: gets the planning frame of the ur5 arm
        :param _eef_link: gets the end effector link of the ur5 arm
        :param _group_names: gets the group names of the ur5 arm
        :param _box_name: gets the name of the box to be added or removed from the Rviz scene
        :param order_status: Publisher handle to publish Dispatch in rostopic "/order_status" once a package is placed on the conveyor belt
        :param order_shipping: Publisher handle to publish the colour and id of the package placed on the conveyor belt
        :param _computed_plan: Stores computed trajectory by planner
        :param _pkg_path: Stores the path of pkg_task5
        :param _file_path: Stores the path of the folder which contains the saved yaml trajectory files
        :param _curr_state: Gets the current state of the robot        
    """
    # Constructor

    def __init__(self, arg_robot_name):
        """
            Constructor for the Actuator2 class.
            It initializes all the class variables.
            This subscribes and publishes the to following topics 
            - "/ur5_2/move_group/display_planned_path"
            - "/ur5_2/execute_trajectory"
            - "/arm_commander"
            - "/order_status"
            - "/order_shipping"
            - "check_box"

        :param arg_robot_name: Takes the name of the ur5 arm to be controlled i.e. ur5_1 or ur5_2
        """

        rospy.init_node('ur5_2_controller', anonymous=True)

        self._robot_ns = '/' + arg_robot_name
        self._planning_group = "manipulator"

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(
            robot_description=self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(
            ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(
            self._planning_group, robot_description=self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._group.set_planning_time(seconds=200)
        self._display_trajectory_publisher = rospy.Publisher(
            self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        # Attribute to store computed trajectory by the planner
        self._computed_plan = ''
        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task5')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.Subscriber('/order_shipping', String, self.arm_command)
        rospy.Subscriber('check_box', String, self.pickup)
        self.order_status = rospy.Publisher(
            '/order_status', String, queue_size=12)

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

    def set_joint_angles(self, arg_list_joint_angles):
        """
        Sets the joint angles of the ur5 arm to the given set of joint angles.

        :param arg_list_joint_angles: Gets the set of joint angles

        :return: flag_plan: Indicates the success at setting the given joint angles
        :rtype: boolean
        """

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        """
        Checks to see if Rviz has updated its scene in 4 different ways according to Arguments- \n
        1) If the Box was added to the scene
        2) If the End Effector has gripped the box
        3) If the End Effector has released the box
        4) If the Box was removed from the scene

        :param box_is_known: True if there is supposed to be a box in the scene, False if there is supposed to be no box in the scene
        :param box_is_attached: True if the End Effector is supposed to attach to the box, False if the End Effector is supposed to detach the box
        :param timeout: No. Of seconds given to check if the Rviz scene has updated or not

        :return: Value indicating the success of Rviz updating its scene according to the following parameters
        :rtype: boolean

        """
        box_name = self._box_name
        scene = self._scene
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            is_known = box_name in scene.get_known_object_names()
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()

        return False

    def add_box(self, x, y, z, type, timeout=4):
        """
        Adds a box at x, y, z coordinate in the Rviz Scene

        :param x: The X-coordinate where the box is to be added
        :param y: The Y-coordinate where the box is to be added
        :param z: The Z-coordinate where the box is to be added
        :param type: The Name the box is given in the Rviz scene
        :param timeout: No. Of seconds given to check if the Rviz scene has updated or not

        :return: Value indicating the success of the box being added to the Rviz Scene
        :rtype: boolean

        """
        box_name = self._box_name
        scene = self._scene

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.position.x = x
        box_pose.pose.position.y = y
        box_pose.pose.position.z = z
        box_name = type
        scene.add_box(box_name, box_pose, size=(0.15, 0.15, 0.15))

        self._box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        """
        Makes the End Effector grab the box

        :param timeout: No. Of seconds given to check if the Rviz scene has updated or not
        :type timeout: int, optional

        :return: Value indicating the success of the End Effector grabbing the box
        :rtype: boolean
        """
        box_name = self._box_name
        robot = self._robot
        scene = self._scene
        eef_link = self._eef_link
        group_names = self._group_names

        grasping_group = 'manipulator'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)

        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):
        """
        Makes the End Effector detach the box

        :param timeout: No. Of seconds given to check if the Rviz scene has updated or not
        :type timeout: int, optional

        :return: Value indicating the success of the End Effector detaching the box
        :rtype: boolean

        """
        box_name = self._box_name
        scene = self._scene
        eef_link = self._eef_link

        scene.remove_attached_object(eef_link, name=box_name)
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):
        """
        Deletes the box from the Rviz scene
        :param timeout: No. Of seconds given to check if the Rviz scene has updated or not
        :type timeout: int, optional

        :return: Value indicating the success of the deletion of the box from the scene
        :rtype: boolean

        """
        box_name = self._box_name
        scene = self._scene
        scene.remove_world_object(box_name)

        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

    def attachgripper(self, r_name):
        """
        Turns on i.e Activates the End Effector(Vacuum Gripper) by accessing it's Service
        
        :param r_name: The name of the robot whose vacuum gripper service is accessed.
        :type r_name: string

        :return: Value indicating that the Vacuum Gripper has been activated
        :rtype: boolean

        """
        rospy.wait_for_service(
            '/eyrc/vb/ur5/activate_vacuum_gripper/' + r_name)
        grip = rospy.ServiceProxy(
            '/eyrc/vb/ur5/activate_vacuum_gripper/' + r_name, vacuumGripper)
        return grip(True)

    def dettachgripper(self, r_name):
        """
        Turns off i.e deactivates the End Effector(Vacuum Gripper) by accessing its Service

        :param r_name: The name of the robot whose vacuum gripper service is accessed.
        :type r_name: string

        :return: Value indicating that the Vacuum Gripper has been activated
        :rtype: boolean
        """

        rospy.wait_for_service(
            '/eyrc/vb/ur5/activate_vacuum_gripper/' + r_name)
        grip = rospy.ServiceProxy(
            '/eyrc/vb/ur5/activate_vacuum_gripper/' + r_name, vacuumGripper)
        return grip(False)

    def arm_command(self, message):
        """
        Callback function to get the colour of the package currently placed on the conveyor and adding it to a global list colors.
        This is used to accordingly place the packages in the right coloured bins.
        
        :param message: The color of the package currently placed on the conveyor belt.

        """
        global colors
        col, id = message.data.split(';')
        colors.insert(0, (col, id))

    def pickup(self, message):
        """
        Callback function which makes the ur5_2 arm pick up the package from the conveyor belt when the box is placed just under the ur5_2 end effector, and takes the package over to the appropriate coloured bin and releases the package.
        
        :param message: Indicates if there is a box positioned just under the ur5_2 end effector (by a boolean value) and the index of the package (to synchronize with the index of the package colour). If True, then the ur5_2 arm picks up the package and places it in the right coloured bin.

        """
        global cnt, colors
        z = 0.995
        if(message.data == str(cnt)+"True"):
            typ = colors[-1][0]
            pose_values = self._group.get_current_pose().pose
            self.add_box(float(pose_values.position.x), float(
                pose_values.position.y), float(z), typ)
            while(1):
                flag_gripper = False
                check_gripper = rospy.ServiceProxy(
                    '/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
                flag_gripper = check_gripper(True).result
                self.attachgripper("ur5_2")
                if(flag_gripper):
                    break
            self.attach_box()
            self.drop(typ)
            self.order_status.publish(colors[-1][1]+';'+"shipped")
            colors.pop()
            cnt += 1

    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        """Plays the requested yaml saved trajectory.
        
        :param arg_file_path: Gets the directory in which all the yaml saved trajectories are saved
        :param arg_file_name: Gets the specific name of the yaml saved trajectory to be played
    
        :return: Value which indicates the success of playing the requested yaml trajectory
        :rtype: boolean

        """
        file_path = arg_file_path + arg_file_name

        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)

        ret = self._group.execute(loaded_plan)
        # rospy.logerr(ret)
        return ret

    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        """
        Plays the requested yaml saved trajectory by calling def moveit_play_planned_path_from_file and attempts it "arg_max_attempts" no. of times before aborting.
        
        :param arg_file_path: Gets the directory in which all the yaml saved trajectories are saved
        :param arg_file_name: Gets the specific name of the yaml saved trajectory to be played
        :param arg_max_attempts: Gets the number of times function should attempt at playing the yaml trajectory before aborting

        :return: True after the process is over
        :rtype: boolean

        """
        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and (flag_success is False)):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(
                arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts))
            # # self.clear_octomap()

        return True

    def drop(self, type):
        """
        Makes the ur5_2 arm move from the conveyor belt to the right coloured bin, drop the package, and then come back to its home position.
        
        :param type: Colour of the package and thus the bin where the package is supposed to end up in.
        :type type: string

        """

        flag1 = False
        flag2 = False
        if(type == "red"):
            while(True):
                flag1 = self.moveit_hard_play_planned_path_from_file(
                    self._file_path, 'home_to_red.yaml', 5)
                if(flag1 == True):
                    break
            self.drop_in_bin()
            while(True):
                flag2 = self.moveit_hard_play_planned_path_from_file(
                    self._file_path, 'red_to_home.yaml', 5)
                if(flag2 == True):
                    break

        if(type == "yellow"):
            while(True):
                flag1 = self.moveit_hard_play_planned_path_from_file(
                    self._file_path, 'home_to_yellow.yaml', 5)
                if(flag1 == True):
                    break
            self.drop_in_bin()
            while(True):
                flag2 = self.moveit_hard_play_planned_path_from_file(
                    self._file_path, 'yellow_to_home.yaml', 5)
                if(flag2 == True):
                    break

        if(type == "green"):
            while(True):
                flag1 = self.moveit_hard_play_planned_path_from_file(
                    self._file_path, 'home_to_green.yaml', 5)
                if(flag1 == True):
                    break
            self.drop_in_bin()
            while(True):
                flag2 = self.moveit_hard_play_planned_path_from_file(
                    self._file_path, 'green_to_home.yaml', 5)
                if(flag2 == True):
                    break


    def drop_in_bin(self):
        """
        Makes the ur5_2 arm drop the package and remove the package drom the ur5_2 Rviz Scene.

        """
        self.dettachgripper("ur5_2")
        self.detach_box()
        self.remove_box()
        rospy.sleep(0.5)

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Actuator Deleted." + '\033[0m')


def main():
    """
    Instantiates an Actuator2 object of the ur5_2 arm and sets the arm to its home position
    """
    ur5 = Actuator2("ur5_2")

    home = [math.radians(8.6),
            math.radians(-123),
            math.radians(-88),
            math.radians(-59),
            math.radians(90),
            math.radians(0)]
    ur5.set_joint_angles(home)
    rospy.spin()

    del ur5


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
