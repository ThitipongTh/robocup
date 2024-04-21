#!/usr/bin/env python3
import rospy
import mqtt_communication
from threading import Thread
import numpy as np
import robot

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String


# Setup variables
_hostname = robot._hostname
_robotID = robot._robotID


# Global variable
robot_pose = [0]
game_state = [0]
kill_thread = [0]


# Handle functions
def handle_robot_pose(msg):
    global robot_pose, _robotID
    
    pos = msg.pose.pose.position
    orn = msg.pose.pose.orientation
    yaw = np.arctan2(2*(orn.w * orn.z + orn.x * orn.y), 1 - 2*(np.power(orn.y, 2) + np.power(orn.z, 2)))
     
    robot_pose[0] = {
        'name': _robotID,
        'x': f'{pos.x:.1f}',
        'y': f'{pos.y:.1f}',
        'yaw': int(np.degrees(yaw))
    }


# Main
if __name__ == '__main__':
    rospy.init_node('handle_refbox')
    
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, handle_robot_pose)
    
    game_state_publisher = rospy.Publisher('/game_state', String, queue_size= 10)
    
    handle_refbox_game_state_thread = Thread(target= mqtt_communication.handle_refbox_game_state, args=(_hostname, game_state_publisher, kill_thread))
    handle_refbox_game_state_thread.start()
    
    
    print(f'Initiate robot - {_robotID}, connect to host {_hostname}')

    while not rospy.is_shutdown():
        
        mqtt_communication.push_current_position(_hostname, _robotID, robot_pose)
        print(robot_pose[0])
        rospy.sleep(1)
    
    kill_thread[0] = None
    handle_refbox_game_state_thread.join()
        