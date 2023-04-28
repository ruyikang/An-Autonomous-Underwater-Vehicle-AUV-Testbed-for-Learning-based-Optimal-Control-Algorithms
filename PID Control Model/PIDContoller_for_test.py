#!/usr/bin/env python3

import rospy
import rospkg
import random
import numpy as np
from collections import deque
from pid_controller import PIDController
import rospy
import numpy as np
import math



class PIDController:
        def __init__(self):
            """
            Initialize the AUV controller. This is called by __init__ and should not be called directly.

            Args:
                auv: avv object to be used for the controller to control.
                goal: goal position to be reached.
            """

            print("auv controller init")
            # init yaw and distance
            self.des_yaw = 0.0
            self.des_distance = 0.0
            self.last_yaw = 0
            self.last_des_yaw = 0

            # Initialize the PID controllers for yaw and distance
            self.controller_yaw = PIDController(
                Kp=40.0, Ki=0.0, Kd=0.0, setpoint=self.des_yaw
            )
            self.controller_distance = PIDController(
                Kp=8.0, Ki=0.0, Kd=1.0, setpoint=self.des_distance
            )

        def get_position(self, x, y, yaw):

            return x, y, yaw

        def controller(self, x, y, yaw):
            """
            Called when the robot is ready to receive data.
            This is where the controller is supposed to update the robot's state based on the state of the robot in the simulation.

            Args:
                model_state: DONT USE ModelStates! Use Gazebo service instead.
                Actually obtained in get_auv_pose(): rospy.ServiceProxy
            """

            goal1 = np.array([5.0, 5.0, 0.0])
            goal2 = np.array([-5.0, 5.0, 0.0])
            goal3 = np.array([-5.0, -5.0, 0.0])
            goal4 = np.array([5.0, -5.0, 0.0])
            goal = [goal1, goal2, goal3, goal4]
            i = 0

            # orientation_reached = False
            while True:
                 # x, y, yaw = self.get_position(x, y, yaw)

                print("x: ", x)
                print("y: ", y)
                print("goal x: ", goal[i][0])
                print("goal y: ", goal[i][1])
                # if arrive nearby the target point, then go to next point
                # or already pass away a target point, auv will ignore it.
                if self.check_goal(x, y, goal[0], goal[1]) or self.check_pos_x(x, y, goal[0], goal[1]):
                    print("------ Next goal, break the loop --------")
                    i += 1
                    break

                distance = math.sqrt(
                    (x - goal[i][0]) * (x - goal[i][0]) + (y - goal[i][1]) * (y - goal[i][1])
                )
                self.des_yaw = math.atan2(goal[1] - y, goal[0] - x)
                print("des_yaw: ", self.des_yaw)
                print("distance: ", distance)
                print("x: ", x)
                print("y: ", y)
                print("yaw: ", yaw)

                self.controller_yaw.setGoal(self.des_yaw)
                self.controller_distance.setGoal(self.des_distance)  # des_distance = 0.0
                output1 = self.controller_yaw.calculate(yaw)
                output2 = self.controller_distance.calculate(distance)
                    #
                    # # Reorienting step into orientation changing
                    # if not orientation_reached:
                    #     print("Step into orientation changing")
                    #     # Reorienting finished. Set output2 to 0.0 if orientation is reached.
                    #     if abs(self.des_yaw - yaw) > 0.2:
                    #         print("Set output2 to 0.0")
                    #         output2 = 0.0
                    #     else:
                    #         print("------- Reorienting finished!----------")
                    #         orientation_reached = True

                    # Check if goal is reached. If yes, break the loop.
                if distance < 1.0:
                    output1 = 0.0
                    output2 = 0.0
                    break

                print("output1: ", output1)
                print("output2: ", output2)
                self.action(output1, output2)

        def check_pos_x(self, x1, y1, x2, y2):
            """
            Check if the AUV already passes away the nearnest goalpoint, only considering left region

            Args:
                x1 y1: x and y coordinate of the AUV body
                x2 y2: x and y coordinate of the goal

            Returns:
                True if the goal is dropped.
            """
            if (np.sign(x1) == np.sign(x2) == 1.0 and np.sign(y1) == np.sign(y2) == 1.0) or (
                    np.sign(x1) == np.sign(x2) == -1.0 and np.sign(y1) == np.sign(y2) == 1.0):
                delta = 2.0
                return x2 - x1 > delta

        def check_goal(self, x1, y1, x2, y2):
            """
            Check if the AUV is close enough to the goal.

            Args:
                x1 y1: x and y coordinate of the AUV body
                x2 y2: x and y coordinate of the goal

            Returns:
                True if the goal is dropped.
            """
            delta = 1.0
            return abs(x2 - x1) < delta and abs(y2 - y1) < delta



        def action(self, output1, output2):
            """
            Set the action to be taken. This is a shortcut for set_action ( np. zeros ( 3 )).

            Args:
                output2: the output of the main thruster. Move the AUV forward.
                output1: the output of two side thrusters. Rotate the AUV.
            """
            action = np.zeros(3)
            action[1] = -output1
            action[0] = -output2
            action[2] = output1
            button = self.map_action_to_button(action)
            print("button: ", button)

        def map_action_to_button(self, action):
            """
            Maps an action to a button.
            This is a function to be used in order to determine the button to be pressed based on the action.

            Args:
                action: nparray of shape ( 3 ).

            Returns:
                int corresponding to the action as a button.
            """
            # nparray stores signs of action
            nparray = np.zeros(3)
            nparray[0] = np.sign(action[0])  # Main thruster
            nparray[1] = np.sign(action[1])  # Right thruster
            nparray[2] = np.sign(action[2])  # Left thruster
            # if signs equal to the set array, then map tp corresponding button
            if np.array_equal(nparray, np.array([0., 1., -1.])):
                return 0  # right
            elif np.array_equal(nparray, np.array([-1., 0., 0.])):
                return 1  # back
            elif np.array_equal(nparray, np.array([1., 0., 0.])):
                return 2  # forward
            elif np.array_equal(nparray, np.array([0., -1., 1.])):
                return 3  # left
            elif np.array_equal(nparray, np.array([1., 1., -1.])):
                return 6  # forward+right
            elif np.array_equal(nparray, np.array([1., -1., 1.])):
                return 7  # forward+left
            elif np.array_equal(nparray, np.array([-1., 1., -1.])):
                return 8  # back+right
            elif np.array_equal(nparray, np.array([-1., -1., 1.])):
                return 9  # back+left
            elif np.array_equal(nparray, np.array([0., 0., 0.])):
                return 10  # stop
            else:
                return None





def main():
    """
    Create and publish a goal. This is the main function of the AUV task.
    """

    goal_list = generate_goal()

    # Initialize the AUV controller and publish the goal position
    controller = PIDController(goal_list)



# This function is used to handle the main function.
if __name__ == "__main__":
    main()
