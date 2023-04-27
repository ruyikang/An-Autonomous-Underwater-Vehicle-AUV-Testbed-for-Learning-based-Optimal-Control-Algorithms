#!/usr/bin/env python3

import gym
from gym import wrappers
import rospy
import rospkg
import random
import numpy as np
from collections import deque
from deepleng_gym.task_envs.deepleng import deepleng_docking

from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from auv_controller import AUVController


class TaskEnv:
    def __init__(self):
        """
        Initialize Gym and reset the state to initial state before any actions are performed.
        """
        # Create the Gym environment
        self.env = gym.make("DeeplengDocking-v2")
        rospy.loginfo("Gym environment done")
        state = self.env.reset()
        rospy.loginfo("Gym environment reset")

    def send_goal(goal):
        """
        Publish the goal to the / goal topic.
        This is a blocking function that will wait for 10Hz to be received and publish the goal every 10 seconds
        NOT RECOMMENDED TO USE THIS FUNCTION. USE THE generate_goal() FUNCTION.
        Args:
            goal: The goal to be published to the /goal topic.
        """
        # Publish the goal to the '/goal' topic
        pub_goal = rospy.Publisher("/goal", Point, queue_size=10)
        rate = rospy.Rate(10)  # Publish rate of 10Hz

        # This method will block until rospy is shutdown.
        while not rospy.is_shutdown():
            pub_goal.publish(goal)
            rate.sleep()


class DeepLeng:
    def __init__(self, task_env):
        """
        Initialize the task. This is called before __call__ is called.
        You can override this method in order to do something with the task environment before the task starts.

        Args:
            task_env: The environment of the Docking task.
        """
        self.task_env = task_env
        pass

    def set_action(self, action):
        """
        Set thruster RPMs. This is a wrapper around task_env.
        Set_thruster_rpm to allow us to set the action at run time.

        Args:
            action: Action to set at run time.
        """
        """
        Note:
        x_thruster_rpm.data = thruster_rpms[0]
        y_thruster_rear_rpm.data = thruster_rpms[1]
        y_thruster_front_rpm.data = thruster_rpms[2]
        """
        self.task_env.env.set_thruster_rpm(action, time_sleep=0.5)

    def get_auv_pose(self):
        """
        Get AUV pose from deeppleng. This is used to determine the position of the camera.
        DONT USE THIS FUNCTION. USE THE get_auv_pose() FUNCTION IN THE AUV_CONTROLLER CLASS.

        Returns:
                a tuple of 3 floats ( x y z w ) or None if there is no pose to be found.
        """
        """
                geometry_msgs/Pose pose
                    geometry_msgs/Point position
                    float64 x
                    float64 y
                    float64 z
                    geometry_msgs/Quaternion orientation
                    float64 x
                    float64 y
                    float64 z
                    float64 w
        """
        model_state_list = self.env.auv_data
        # Auv_pose is the AUV Pose of the model state list.
        for i in range(len(model_state_list.name)):
            # The AUV Pose of the AV.
            if model_state_list.name[i] == "deepleng":
                auv_pose = model_state_list.pose[i]
                return auv_pose
        return None


def generate_goal():
    """
    Generates a goal for the simulation. It is used to test the generation of the goal.


    Returns:
        The point of the goals. Point() is a ROS message type.
    """
    # Set the goal position
    goal1 = Point()
    goal1.x = 5.0  # x coordinate of the goal position
    goal1.y = 5.0  # y coordinate of the goal position
    goal1.z = 0.0  # z coordinate of the goal position

    # Set the goal position
    goal2 = Point()
    goal2.x = -5.0  # x coordinate of the goal position
    goal2.y = 5.0  # y coordinate of the goal position
    goal2.z = 0.0  # z coordinate of the goal position

    # Set the goal position
    goal3 = Point()
    goal3.x = -5.0  # x coordinate of the goal position
    goal3.y = -5.0  # y coordinate of the goal position
    goal3.z = 0.0  # z coordinate of the goal position

    # Set the goal position
    goal4 = Point()
    goal4.x = 5.0  # x coordinate of the goal position
    goal4.y = -5.0  # y coordinate of the goal position
    goal4.z = 0.0  # z coordinate of the goal position

    goal_list = [goal1, goal2, goal3, goal4]
    return goal_list


def main():
    """
    Create and publish a goal. This is the main function of the AUV task.
    """
    rospy.init_node("PID_deepleng", anonymous=True)
    task_env = TaskEnv()
    rospy.loginfo("Task environment initialized.")
    auv = DeepLeng(task_env)

    goal_list = generate_goal()

    # Initialize the AUV controller and publish the goal position
    controller = AUVController(auv, goal_list)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting down")


# This function is used to handle the main function.
if __name__ == "__main__":
    main()
