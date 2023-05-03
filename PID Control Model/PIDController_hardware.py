#!/usr/bin/env python3

import numpy as np
import math

def PID_Controller(x,y,yaw,i,last_error_dis,last_integral_dis,last_error_yaw,last_integral_yaw,last_current_yaw):


    goal1 = np.array([0.2, 0.3])
    goal2 = np.array([0.3, 0.3])
    goal3 = np.array([0.4, 0.3])
    goal4 = np.array([0.5, 0.2])
    goal5 = np.array([100,100])   # no sense, for stopping at goal4
    goal = [goal1, goal2, goal3, goal4, goal5]

    real_yaw, last_current_yaw = calculate_current_yaw(yaw,last_current_yaw)

    if check_dist_goal(x,y,goal[i][0],goal[i][1]) and check_angle_goal(x,y,goal[i][0],goal[i][1],real_yaw):
        i=i+1

    des_yaw = math.atan2(goal[i][1] - y, goal[i][0] - x)
    des_distance = 0.0

    distance = math.sqrt(
            (x - goal[i][0]) * (x - goal[i][0]) + (y - goal[i][1]) * (y - goal[i][1])
        )



    print("des_yaw: ", des_yaw)
    print("distance: ", distance)
    print("x: ", x)
    print("y: ", y)
    print("yaw: ", real_yaw)

    #button, i, error_dis, error_yaw,intergal_dis, intergal_yaw = PID_Controller(x,y,yaw,i,last_error_dis,last_integral_dis,last_error_yaw,last_integral_yaw):
    #output, error,integral = PIDcontroller(Kp, Ki, Kd, setpoint, process_value,last_integral,last_error):


    #output1
    controller_yaw, error_yaw,integral_yaw= PIDcontroller(
        Kp=40.0, Ki=0.0, Kd=0.0, setpoint=des_yaw, process_value = real_yaw,last_integral=last_integral_yaw,last_error=last_error_yaw)
    #output2
    controller_distance, error_dis,integral_dis = PIDcontroller(
        Kp=8.0, Ki=0.0, Kd=1.0, setpoint=des_distance, process_value = distance,last_integral=last_integral_dis,last_error=last_error_dis)
    #print("x: ", x)
    #print("y: ", y)
    print("goal x: ", goal[i][0])
    print("goal y: ", goal[i][1])

    if i == 4:
        controller_yaw  = 0.0
        controller_distance = 0.0


#  output2: controller_distance, the output of the main thruster. Move the AUV forward.
#  output1: controller_yaw ,the output of two side thrusters. Rotate the AUV.


    print("the difference of yaw: ", controller_yaw )
    print("the difference of distance: ", controller_distance)
    button = action(controller_yaw, controller_distance)

    return button, i, error_dis, error_yaw, integral_dis,integral_yaw, last_current_yaw

def calculate_current_yaw(current_yaw, last_current_yaw):
    count_current = 0
    delta_current_yaw = current_yaw - last_current_yaw
    last_current_yaw = current_yaw
    if delta_current_yaw > 5.0:
        count_current = -1
    elif delta_current_yaw < -5.0:
        count_current = 1
    current_yaw = current_yaw + count_current * 2 * math.pi
    return current_yaw,last_current_yaw





def PIDcontroller(Kp, Ki, Kd, setpoint, process_value,last_integral,last_error):
    # Calculate the error, integral and derivative terms
    error = setpoint - process_value
    integral = last_integral + error
    derivative = error - last_error

    # Calculate the PID output
    output = Kp * error + Ki * integral + Kd * derivative

    # Update the variables
    last_error = error

    return output, error, integral

def check_dist_goal(x1, y1, x2, y2):
    """
    Check if the AUV is close enough to the goal.

    Args:
    x1 y1: x and y coordinate of the AUV body
    x2 y2: x and y coordinate of the goal

    Returns:
        True ifthe goal is dropped.
    """
    delta = 0.05
    if abs(x2 - x1) < delta and abs(y2 - y1) < delta - 0.015:
        return True

def check_angle_goal(x1,y1,x2,y2,yaw):
    """
    Check if the AUV is close enough to the goal.

    Args:
    x1 y1: x and y coordinate of the AUV body
    x2 y2: x and y coordinate of the goal

    Returns:
        True if the goal is dropped.
    """

    delta = 0.7
    target_angle  = math.atan2(y2 - y1, x2 - x1)
    if abs(target_angle - yaw) < delta:
        return True

def action(output1, output2):
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
    button = map_action_to_button(action)
    print("action:", action)
    print("button: ", button)

    return button

def map_action_to_button(action):
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
        return 7  # forward+left
    elif np.array_equal(nparray, np.array([1., -1., 1.])):
        return 6  # forward+right
    elif np.array_equal(nparray, np.array([-1., 1., -1.])):
        return 8  # back+right
    elif np.array_equal(nparray, np.array([-1., -1., 1.])):
        return 9  # back+left
    elif np.array_equal(nparray, np.array([0., 0., 0.])):
        return 10  # stop
    else:
        return None


