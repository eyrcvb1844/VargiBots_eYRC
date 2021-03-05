#! /usr/bin/env python

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

from pkg_vb_sim.srv import vacuumGripper
from std_srvs.srv import Empty
from std_msgs.msg import String
from importlib import import_module
from pkg_vb_sim.msg import LogicalCameraImage
from pkg_vb_sim.msg import Model

dict_order = {"red": [], "yellow": [], "green": []}
tup_order = list()


class Actuator:
    """
        This class recieves the orders from the ur5_action_client node through "/arm_commander" and stores them in a tuple sorted according to their priority. 
        It then makes the ur5_1 arm pick up the packages according to the recieved orders in the sorted order and places them on the conveyor belt using the saved yaml trajectories,
        and then publishes the dispatch message in rostopic "/order_status" and the colour and id of the package dispatched in rostopic "/order_shipping".

        
        :param _robot_ns: Name of the ur5 arm
        :param _planning_group: Name of the planning group by the ur5 arm
        :param _commander: initialized moveit_commander object
        :param _robot: moveit_commander.RobotCommander object to get the group names and the current state of the ur5 arm
        :param _scene: moveit_commander.PlanningSceneInterface object to get the Rviz scene of the arm
        :param _group: moveit_commander.MoveGroupCommander object to move the ur5 arm
        :param _display_trajectory_publisher: Publisher handle to publish to /ur5_1/move_group/display_planned_path
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
        Constructor for the Actuator class.
        It initializes all the class variables.
        This subscribes and publishes the to following topics 
        - "/ur5_1/move_group/display_planned_path"
        - "/ur5_1/execute_trajectory"
        - "/arm_commander"
        - "/order_status"
        - "/order_shipping"
        
        :param arg_robot_name: Takes the name of the ur5 arm to be controlled i.e. ur5_1 or ur5_2
        
        """
        rospy.init_node('ur5_1_controller', anonymous=True)

        self._robot_ns = '/' + arg_robot_name
        self._planning_group = "manipulator"

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(
            robot_description=self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(
            ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(
            self._planning_group, robot_description=self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._group.set_planning_time(seconds=150)
        self._display_trajectory_publisher = rospy.Publisher(
            self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        rospy.Subscriber('/arm_commander', String, self.arm_command)
        self.order_status = rospy.Publisher(
            '/order_status', String, queue_size=12)
        self.order_shipping = rospy.Publisher(
            '/order_shipping', String, queue_size=12)

        # Attribute to store computed trajectory by the planner
        self._computed_plan = ''
        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task5')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        """
        Plays the requested yaml saved trajectory.
        
        :param arg_file_path: Gets the directory in which all the yaml saved trajectories are saved
        :param arg_file_name: Gets the specific name of the yaml saved trajectory to be played
        
        :return: A value which indicates the success of playing the requested yaml trajectory
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
        
        :return: A Boolean True value
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

    def go_to_predefined_pose(self, arg_pose_name):
        """
        Takes the ur5 arm to a predefined pose saved.
        
        :param arg_pose_name: Gets the name of the pose
        
        """
        rospy.loginfo(
            '\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo(
            '\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    def set_joint_angles(self, arg_list_joint_angles):
        """
        Sets the joint angles of the ur5 arm to the given set of joint angles.
        
        :param arg_list_joint_angles: Gets the set of joint angles
        
        :return flag_plan: A Boolean value which indicates the success at setting the given joint angles
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
        Checks to see if Rviz has updated its scene in 4 different ways according to Arguments - \n
        1) If the Box was added to the scene,
        2) If the End Effector has gripped the box
        3) If the End Effector has released the box
        4) If the Box was removed from the scene
        
        :param box_is_known: True if there is supposed to be a box in the scene, False if there is supposed to be no box in the scene
        :param box_is_attached: True if the End Effector is supposed to attach to the box, False if the End Effector is supposed to detach the box
        :param timeout: No. Of seconds given to check if the Rviz scene has updated or not
        
        :return: A Boolean value indicating the success of Rviz updating its scene according to the following parameters
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
        
        :return: A Boolean value indicating the success of the box being added to the Rviz Scene
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
        
        :return: A Boolean value indicating the success of the End Effector grabbing the box
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
        
        :return: A Boolean value indicating the success of the End Effector detaching the box
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
        
        :return: A Boolean value indicating the success of the deletion of the box from the scene
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
        
        :return: A Boolean value indicating that the Vacuum Gripper has been activated
        :rtype: boolean
        """
        rospy.wait_for_service(
            '/eyrc/vb/ur5/activate_vacuum_gripper/' + r_name)
        grip = rospy.ServiceProxy(
            '/eyrc/vb/ur5/activate_vacuum_gripper/' + r_name, vacuumGripper)
        return grip(True)

    def dettachgripper(self, r_name):
        """
        Turns off i.e deactivates the End Effector(Vacuum Gripper) by accessing it's Service
        
        :param r_name: The name of the robot whose vacuum gripper service is accessed.
        
        :return: A Boolean value indicating that the Vacuum Gripper has been deactivated
        :rtype: boolean
        """
        rospy.wait_for_service(
            '/eyrc/vb/ur5/activate_vacuum_gripper/' + r_name)
        grip = rospy.ServiceProxy(
            '/eyrc/vb/ur5/activate_vacuum_gripper/' + r_name, vacuumGripper)
        return grip(False)

    def arm_command(self, data):
        """
        Creates a tuple which contains all the orders and sorts them according to the given high priority to low priority
        
        :param r_name: The name of the robot whose vacuum gripper service is accessed.
        
        :return: A Boolean value indicating that the Vacuum Gripper has been deactivated
        :rtype: boolean
        
        """
        global tup_order
        order = data.data
        color_dict = {'red': 1, 'yellow': 2, 'green': 3, 'empty': 4}
        lis_color = order.split(';')
        lis_color[0] = color_dict[lis_color[0]]
        tup_order.append(tuple(lis_color))
        tup_order = sorted(tup_order)
        print("\n\n\n{}\n\n\n".format(tup_order))

    # Destructor
    def __del__(self):
        """
        Shuts down the moveit_commander and thus deletes the Actuator Object
        
        """
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Actuator Deleted." + '\033[0m')

        rospy.spin()


def main():
    """
    Instantiates and Actuator object of the ur5_1 arm and Controls the Ur5_1 arm. It picks up the packages from the shelf and places packages on the conveyor belt according to the order recieved.
        
    """
    global tup_order
    ur5_1 = Actuator("ur5_1")

    # input = raw_input()
    box_poses = [[[0.28, -0.42, 1.89],
                  [0., -0.42, 1.89],
                  [-0.28, -0.42, 1.89]],
                 [[0.28, -0.42, 1.62],
                     [0., -0.42, 1.62],
                     [-0.28, -0.42, 1.62]],
                 [[0.28, -0.42, 1.39],
                     [0., -0.42, 1.39],
                     [-0.28, -0.42, 1.39]],
                 [[0.28, -0.42, 1.16],
                     [0., -0.42, 1.16],
                     [-0.28, -0.42, 1.16]]]

    home = [math.radians(0),
            math.radians(-116.83),
            math.radians(-81),
            math.radians(-72),
            math.radians(90),
            math.radians(0)]

    decode = {1: 'red', 2: 'yellow', 3: 'green'}
    rospy.sleep(5.0)
    ur5_1.set_joint_angles(home)
    while not rospy.is_shutdown():
        if(tup_order):

            order = tup_order.pop(0)
            print("\n\n\n{}\n\n".format(order))
            x=int(order[1])
            y=int(order[2])

            ur5_1.order_shipping.publish(decode[order[0]]+';'+str(order[3]))

            ur5_1.add_box(box_poses[x][y][0], box_poses[x][y][1], box_poses[x][y][2], "packagen")
            if(x == 0 and (y == 0 or y == 2)):
                ur5_1.go_to_predefined_pose("straightUp")

            ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, 'zero_to_pack{0}{1}.yaml'.format(x,y), 5)
            ur5_1.attachgripper("ur5_1")
            ur5_1.attach_box()
            ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, 'pack{0}{1}_to_pack{0}{1}_1.yaml'.format(x,y), 5)

            if(x == 3 and (y == 0 or y == 1)):
                ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, 'pack{0}{1}_1_to_pack{0}{1}_2.yaml'.format(x,y), 5)
                ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, 'pack{0}{1}_2_to_home.yaml'.format(x,y), 5)
            else:
                ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, 'pack{0}{1}_1_to_home.yaml'.format(x,y), 5)

            ur5_1.detach_box()
            ur5_1.dettachgripper("ur5_1")
            ur5_1.remove_box()
            status=str(order[-1])+";"+"dispatched"
            ur5_1.order_status.publish(status)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
