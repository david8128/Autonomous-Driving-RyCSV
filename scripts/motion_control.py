#!/usr/bin/env python

import rospy
import time
from controlador import controller
from dynamic_reconfigure.server import Server
from auto_drive_rycsv.cfg import controllerConfig
from geometry_msgs.msg import Twist

#Callback function from dynamic reconfigure
def callback(config, level):
    global dyn_flag
    dyn_flag = 1
    print('Param change requested...')
    return config
    

if __name__ == "__main__":

    #Node initialization
    rospy.init_node("motion_control", anonymous = False)
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

    #Node Loop
    while(not rospy.is_shutdown()):
        
        #Get goal from user input
        input_raw = raw_input("Enter goal coordinates [X,Y,TH]: ")
        goal = input_raw.split(",")

        #Set goal 
        kobuki_controller.set_goal(float(goal[0]),float(goal[1]),float(goal[2]))

        #Check and change controllers params if requested 
        if dyn_flag == 1:
            kobuki_controller.set_controller_params()
            dyn_flag = 0

        print("Moving to "+ str(goal) + " (X,Y,TH)")
        print("--")

        print("Wait 3 sec ...")
        time.sleep(1)
        print("Wait 2 sec ..")
        time.sleep(1)
        print("Wait 1 sec .")
        time.sleep(1)


        #Control Loop
        while (kobuki_controller.done == False):

            #Get "now" time to syncronize target tf and error tf 
            now = rospy.Time.now()

            #Broadcast goal TF
            kobuki_controller.broadcast_goal(now)

            #Control methods
            kobuki_controller.compute_error(now)
            kobuki_controller.transform_error()
            kobuki_controller.control_speed()

            #Publish Speed
            command.linear.x =  kobuki_controller.v_out
            command.angular.z =  kobuki_controller.w_out
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


