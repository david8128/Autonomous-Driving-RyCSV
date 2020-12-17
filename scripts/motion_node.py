#!/usr/bin/env python

import rospy
import time
from controlador import controller
from dynamic_reconfigure.server import Server
from auto_drive_rycsv.cfg import controllerConfig
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
from rospy.numpy_msg import numpy_msg 
import numpy as np

#Callback function from dynamic reconfigure
def callback(config, level):
    global dyn_flag
    dyn_flag = 1
    print('Param change requested...')
    return config

#Callback error for vision error
def error_callback(data):
    global error_vision
    error_vision = data.data
      
def lidar_callback(lidar_scan):
    global linear_x
    global angular_z
    global state_description
    ranges    = lidar_scan.ranges.copy()
    regions = {
        'right':  min(min(ranges[210:314]), 10),
        'fright': min(min(ranges[315:344]), 10),
        'front':  min(min(np.concatenate([ranges[0:15],ranges[359:345]])), 10),
        'fleft':  min(min(ranges[16:46]), 10),
        'left':   min(min(ranges[47:150]), 10),
    }
    if regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] > 1:
        if (np.abs(np.mean(ranges[30:150])-np.mean(ranges[210:330]))<0.6): 
            state_description = 'case 1 - nothing'
            linear_x = 0.6
            angular_z = 0
        else:
            state_description = 'case 1.2 - straight'
            linear_x = 0.6
            angular_z = 0
    elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] > 1:
        if(regions['fleft']<regions['fright']):
            state_description = 'case 2.1- front'
            linear_x = 0
            angular_z = -1
        else:
            state_description = 'case 2.2- front'
            linear_x = 0
            angular_z = 1
    elif regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] < 1:
        state_description = 'case 3 - fright'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] > 1:
        state_description = 'case 4 - fleft'
        linear_x = 0
        angular_z = -0.3
    elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] < 1:
        state_description = 'case 5 - front and fright'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] > 1:
        state_description = 'case 6 - front and fleft'
        linear_x = 0
        angular_z = -0.3
    elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] < 1:
        state_description = 'case 7 - front and fleft and fright'
        linear_x = 0
        angular_z = -0.3
    elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] < 1:
        state_description = 'case 8 - fleft and fright'
        linear_x = 0
        angular_z = -0.3
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)
    print(state_description)


if __name__ == "__main__":

    print("MOTION WAITING FOR GAZEBO")
    rospy.wait_for_service('/gazebo/spawn_urdf_model') #Wait for model spawn
    time.sleep(2)

    #Node initialization
    rospy.init_node("motion_node", anonymous = False)
    rate = rospy.Rate(50) # 50 Hz ROS

    #Controller init
    kobuki_controller = controller()

    #Dynamic reconfigure flag
    dyn_flag = 0

    #Dynamic reconfigure server initialization
    srv = Server(controllerConfig, callback)

    #Wheel speed publisher
    nameSpeedTopic = "/cmd_vel"
    kobuki_speed_pub = rospy.Publisher(nameSpeedTopic, Twist, queue_size=10)
    command = Twist()

    #Error suscriber
    error_vision = 0
    vision_error_sus = rospy.Subscriber('vision/error', Int32, error_callback)
    list_error_vision = [error_vision] * 20

    #Lidar suscriber
    nameTopicSub = "/scan"

    # Subscribers
    linear_x = 0
    angular_z = 0
    state_description = ''
    rospy.Subscriber(nameTopicSub, numpy_msg(LaserScan), lidar_callback, queue_size=10 )

    #Node Loop
    while(not rospy.is_shutdown()):
        
        #Set goal 
        k_vision = rospy.get_param('/motion_node/k_vision')
        k_th = rospy.get_param('/motion_node/k_th')
        #k_vision = 0.01
        x = 0.8
        y = np.mean(list_error_vision)*(-1)*k_vision
        th = np.mean(list_error_vision)*(-1)*k_th
        goal = [x,y,th]
        kobuki_controller.set_goal(float(goal[0]),float(goal[1]),float(goal[2]))

        #Broadcast goal TF
        goal_time = rospy.Time.now()
        kobuki_controller.broadcast_goal(goal_time)

        #Check and change controllers params if requested 
        if dyn_flag == 1:
            kobuki_controller.set_controller_params()
            dyn_flag = 0

        # print("Moving to "+ str(goal) + " (X,Y,TH)")
        # print("--")

        """ print("Wait 3 sec ...")
        time.sleep(1)
        print("Wait 2 sec ..")
        time.sleep(1)
        print("Wait 1 sec .")
        time.sleep(1) """

        #Control Loop
        while (kobuki_controller.done == False):
            """ #Set goal 
            k_vision = rospy.get_param('/motion_node/k_vision')
            #k_vision = 0.01
            x = 1
            y = error_vision*(-1)*k_vision
            th = 0
            goal = [x,y,th]
            kobuki_controller.set_goal(float(goal[0]),float(goal[1]),float(goal[2]))
 """
            list_error_vision.append(error_vision)
            list_error_vision = list_error_vision[1:]
            # print(list_error_vision)
            # print("Este es el Goal: ")
            # print(goal)

            #Get "now" time to syncronize target tf and error tf 
            now = rospy.Time.now()

            #Control methods
            kobuki_controller.compute_error(now,goal_time)
            kobuki_controller.transform_error()
            kobuki_controller.control_speed()

            #Publish Speed
            if(state_description == 'case 1 - nothing'):
                command.linear.x =  kobuki_controller.v_out
                command.angular.z =  kobuki_controller.w_out
            else:
                command.linear.x =  linear_x
                command.angular.z =  angular_z

            kobuki_speed_pub.publish(command)

            #Check if goal has been reached
            kobuki_controller.check_goal_reached()

            #Check and change controllers params if requested 
            if dyn_flag == 1:
                kobuki_controller.set_controller_params()
                dyn_flag = 0
            
            rate.sleep() #Wait for ROS node cycle
            

        #Stop
        command.linear.x =  0
        command.angular.z =  0
        kobuki_speed_pub.publish(command)
        kobuki_controller.done = False
        print("Goal has been reached...")
        print("--")
        rate.sleep() #Wait for ROS node cycle


