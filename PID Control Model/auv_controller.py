#!/usr/bin/env python3

from pid_controller import PIDController
import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import numpy as np
import math


class AUVController:
    def __init__(self, auv, goal):
        """
        Initialize the AUV controller. This is called by __init__ and should not be called directly.
        
        Args:
            auv: avv object to be used for the controller to control.
            goal: goal position to be reached.
        """
        print("auv controller init")
        self.auv = auv
        self.goal = goal
        # init yaw and distance
        self.des_yaw = 0.0
        self.des_distance = 0.0
        self.last_yaw = 0
        self.last_des_yaw = 0
        self.count = 0
        self.count_gazebo = 0

        # Initialize the PID controllers for yaw and distance
        self.controller_yaw = PIDController(
            Kp=40.0, Ki=0.0, Kd=0.0, setpoint=self.des_yaw
        )
        self.controller_distance = PIDController(
            Kp=8.0, Ki=0.0, Kd=1.0, setpoint=self.des_distance
        )

        # # Subscribe to the topics for the AUV's position
        rate = rospy.Rate(1)
        # Only work as callback function DONT USE ModelStates!
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)

    def callback(self, model_state):
        """
        Called when the robot is ready to receive data. 
        This is where the controller is supposed to update the robot's state based on the state of the robot in the simulation.
        
        Args:
            model_state: DONT USE ModelStates! Use Gazebo service instead.
            Actually obtained in get_auv_pose(): rospy.ServiceProxy
        """
        for goal in self.goal:
            rospy.loginfo(goal)
            orientation_reached = False
            while True:
                x, y, yaw = self.get_position()
                
                print("x: ", x)
                print("y: ", y)
                print("goal x: ", goal.x)
                print("goal y: ", goal.y)
               # if arrive nearby the target point, then go to next point
                # or already pass away a target point, auv will ignore it.
                if self.check_goal(x, y, goal.x, goal.y) or self.check_pos_x(x, y, goal.x, goal.y):
                    print("------ Next goal, break the loop --------")
                    break
                
                yaw = self.calculate_gazebo_yaw(yaw)
                # Calculate the output for the left and right thrusters in the x direction
                distance = math.sqrt(
                    (x - goal.x) * (x - goal.x) + (y - goal.y) * (y - goal.y)
                )
                current_yaw = math.atan2(goal.y - y, goal.x - x)
                self.des_yaw = self.calculate_yaw(current_yaw)
                print("des_yaw: ", self.des_yaw)
                print("distance: ", distance)
                print("x: ", x)
                print("y: ", y)
                print("yaw: ", yaw)

                self.controller_yaw.setGoal(self.des_yaw)
                self.controller_distance.setGoal(self.des_distance)    # des_distance = 0.0
                output1 = self.controller_yaw.calculate(yaw)
                output2 = self.controller_distance.calculate(distance)

                # Reorienting step into orientation changing
                if not orientation_reached:
                    print("Step into orientation changing")
                    # Reorienting finished. Set output2 to 0.0 if orientation is reached.
                    if abs(self.des_yaw - yaw) > 0.2:
                        print("Set output2 to 0.0")
                        output2 = 0.0
                    else:
                        print("------- Reorienting finished!----------")
                        orientation_reached = True

                # Check if goal is reached. If yes, break the loop.
                if distance < 1.0:
                    output1 = 0.0
                    output2 = 0.0
                    rospy.loginfo("Goal reached!")
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
        if (np.sign(x1) == np.sign(x2) == 1.0 and np.sign(y1) == np.sign(y2) == 1.0 ) or (np.sign(x1) == np.sign(x2) == -1.0 and np.sign(y1) == np.sign(y2) == 1.0 ):
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

    def calculate_gazebo_yaw(self, current_yaw):
        """
        Calculates gazebo yaw based on current yaw. In case the yaw goes over.
        
        Args:
            current_yaw: The current yaw in radians from the Gazebo service.
        
        Returns: 
            The new yaw in radians after fixing the gazebo yaw.
        """
        delta_yaw = current_yaw - self.last_yaw
        self.last_yaw = current_yaw
        if delta_yaw > 5.0:
            self.count_gazebo -= 1
        elif delta_yaw < -5.0:
            self.count_gazebo += 1
        current_yaw = current_yaw + self.count_gazebo * 2 * math.pi
        return current_yaw

    def calculate_yaw(self, current_yaw):
        """
        Calculates desired yaw based on current desired yaw.
        
        Args:
            current_yaw: Current yaw in radians.
        
        Returns: 
            The new yaw in radians after fixing the desired yaw.
        """
        delta_yaw = current_yaw - self.last_des_yaw
        self.last_des_yaw = current_yaw
        if delta_yaw > 5.0:
            self.count -= 1
        elif delta_yaw < -5.0:
            self.count += 1
        current_yaw = current_yaw + self.count * 2 * math.pi
        return current_yaw

    def get_auv_pose(self):
        """
        Get AUV pose from Gazebo. 
        Call Gazebo service to get the AUV pose.
    
        Returns: 
            The pose of the AUV in world coordinates.
        """
        get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        get_model_state.wait_for_service()
        model_name = "deepleng"
        request = GetModelStateRequest()
        request.model_name = model_name
        response = get_model_state(request)
        auv_pose = response.pose
        return auv_pose

    def get_position(self):
        """
        Get the position of the camera in world coordinates. 
        
        
        Returns: 
            ( x y yaw ) of the AUV body position in world coordinates and yaw in radians.
        """
        auv_pose = self.get_auv_pose()
        x = auv_pose.position.x
        y = auv_pose.position.y
        yaw = self.quat2yaw(
            auv_pose.orientation.w,
            auv_pose.orientation.x,
            auv_pose.orientation.y,
            auv_pose.orientation.z,
        )
        return x, y, yaw

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
        self.auv.set_action(action)
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
        nparray[0] = np.sign(action[0]) # Main thruster
        nparray[1] = np.sign(action[1]) # Right thruster
        nparray[2] = np.sign(action[2]) # Left thruster
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
        
    def quat2yaw(self, w, x, y, z):
        """
        Convert quaternion to yaw angle. This is used to convert a 3D rotation matrix to a yaw angle
        
        Args:
            w: W component of quaternion in radians
            x: X component of quaternion in radians. Must be nonnegative.
            y: Y component of quaternion in radians. Must be nonnegative.
            z: Z component of quaternion in radians. Must be nonnegative.
        
        Returns: 
            Yaw angle in radians.
        """
        # roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        # pitch = math.asin(2 * (w * y - x * z))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
        return yaw
