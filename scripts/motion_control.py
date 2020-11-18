#!/usr/bin/env python

#Lib imports
import rospy
import roslib
import tf_conversions
import tf2_ros
import math
import numpy as np
import time
from control import Motion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist


    
if __name__ == '__main__':

    #Trajectory node init
    rospy.init_node('motion_controller', anonymous=True)
    rospy.loginfo("Motion controller node init")

    #Wheel speed publisher
    nameSpeedTopic = "/cmd_vel"
    kobuki_speed_pub = rospy.Publisher(nameSpeedTopic, Twist, queue_size=10)
    command = Twist()

    #Controller object
    controlador = Motion()

    #Square trajectory for testing
    traj = np.array([[0,0,0],
                       [2,0,0],
                       [2,0,90],
                       [2,2,90],
                       [2,2,180],
                       [0,2,180],
                       [0,2,270],
                       [0,0,270]])

    #Paralelogram trajectory for testing
    #traj = np.array([[0,0,0],
    #                   [2,1,90],
    #                   [3,2,0],
    #                   [4,1,-90]])


    #Trajectory limits
    goal_id = 1
    dot_count, coord = traj.shape

    rate = rospy.Rate(50) # 50 Hz ROS

    print("WAITING FOR GAZEBO")
    rospy.wait_for_service('/gazebo/spawn_urdf_model') #Wait for model spawn
    time.sleep(2)
    
    #Initial point
    controlador.set_goal(traj[goal_id][0],traj[goal_id][1],traj[goal_id][2])
    print("--")
    print("GOAL")
    print("X: "+str(controlador.x_goal))
    print("Y: "+str(controlador.y_goal))
    print("Z: "+str(controlador.th_goal))
    print("--")

    print('Trayectoria a seguir')
    print(traj)
    
    time.sleep(15)

    while (not rospy.is_shutdown()):

        #Get "now" time to syncronize target tf and error tf 
        now = rospy.Time.now()
        controlador.broadcast_goal(now) 
        controlador.compute_error(now)
        controlador.transform_error()
        controlador.control_speed() #Compute wheel speed (out)

        command.linear.x =  controlador.v_out
        command.angular.z =  controlador.w_out

        kobuki_speed_pub.publish(command)

        if controlador.arrived2goal():  
            goal_id = goal_id+1      #Change point when arrived to goal

            if goal_id == dot_count:
                #goal_id = goal_id-1         #Wait at last point
                goal_id = 0                 #Go back to initial point

            controlador.set_goal(traj[goal_id][0],traj[goal_id][1],traj[goal_id][2])
            now = rospy.Time.now()
            controlador.broadcast_goal(now) 
            controlador.compute_error(now)

        print("--")
        print("GOAL")
        print("X: "+str(controlador.x_goal))
        print("Y: "+str(controlador.y_goal))
        print("TH: "+str(controlador.th_goal))
        print("--")
        rate.sleep()