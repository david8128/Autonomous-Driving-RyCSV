#!/usr/bin/env python

import rospy
import time
from controlador import controller
from dynamic_reconfigure.server import Server
from auto_drive_rycsv.cfg import controllerConfig
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

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


    #Node Loop
    while(not rospy.is_shutdown()):
        
        #Set goal 
        k_vision = rospy.get_param('/motion_node/k_vision')
        #k_vision = 0.01
        x = 1
        y = error_vision*(-1)*k_vision
        th = 0
        goal = [x,y,th]
        kobuki_controller.set_goal(float(goal[0]),float(goal[1]),float(goal[2]))

        #Broadcast goal TF
        now = rospy.Time.now()
        kobuki_controller.broadcast_goal(now)

        #Check and change controllers params if requested 
        if dyn_flag == 1:
            kobuki_controller.set_controller_params()
            dyn_flag = 0

        print("Moving to "+ str(goal) + " (X,Y,TH)")
        print("--")

        """ print("Wait 3 sec ...")
        time.sleep(1)
        print("Wait 2 sec ..")
        time.sleep(1)
        print("Wait 1 sec .")
        time.sleep(1) """

        #Control Loop
        while (kobuki_controller.done == False):
            #Set goal 
            k_vision = rospy.get_param('/motion_node/k_vision')
            #k_vision = 0.01
            x = 1
            y = error_vision*(-1)*k_vision
            th = 0
            goal = [x,y,th]
            kobuki_controller.set_goal(float(goal[0]),float(goal[1]),float(goal[2]))

            print("Este es el Goal: ")
            print(goal)

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


