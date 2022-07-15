#!/usr/bin/env python3 

import rospy
import tf2_msgs.msg
import tf_conversions
import tf
import tf2_ros
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
from controller_utils import *
import geometry_msgs.msg

'''
This node is in charge of read the joint_state sensor of the arms, 
apply the Resolver-rate motion control algorithm and compute 
the command velocities to move the robot towards a position goal in the 
3D space
'''
# Class to handle the Resolved-rate motion controller 
class MotionController:
    def __init__(self):

        '''Robot attributes'''
        self.joints       = np.array([0,0])
        self.robot        = Manipulator(self.joints)
        self.v_max        = 0.1


        '''Task-Priority parameters'''
        self.P       = np.eye(self.robot.getDOF())
        self.uv      = np.zeros((2,1))
        self.ur      = np.zeros((2,1))
        self.weights = np.array([[1,1]])
        
        
        '''Subscribers'''
        self.sub_center       = rospy.Subscriber("/modem_center", Marker, self.get_center, queue_size=10)

        '''Publishers'''
        self.pub_command_velocity = rospy.Publisher("/alpha2/joint_velocity_controller/command",Float64MultiArray,queue_size=10)

        '''Timer for the controller'''
        self.pub_timer = rospy.Timer(rospy.Duration(0.1),self.resolved_rate)


        '''Control parameters'''
        self.modem_center = np.array([320, 240])

        '''Tasks definition'''
        self.tasks   = [
                        BasePosition('Base Orientation',np.array([[np.deg2rad(0)]]),gain=1)
                        ]

        print("\nResolved-rate motion controller running...")


    def get_center(self, msg):
        self.modem_center = np.array([msg.pose.position.x, msg.pose.position.y])
    

    def resolved_rate(self,event=None):
        position = Float64MultiArray()
        self.robot.update(self.joints)

        self.dq     = np.zeros((self.robot.getDOF(),1))
        self.uv     = np.zeros((2,1))
        self.ur     = np.zeros((2,1))
        self.P      = np.eye(self.robot.getDOF())

    
        for index,task in enumerate(self.tasks):
            print("---------- modem center", self.modem_center)
            task.update(self.robot, self.modem_center)

            if task.isActive():
                Jbar = task.getJacobian() @ self.P
                x_dot = task.getFfv() + task.getGainMatrix() @ task.getError()
                self.ur   =  self.ur + DLS_Weights(Jbar,0.05,self.weights) @ (x_dot-(task.getJacobian()@self.ur))
                self.P    =  self.P - np.linalg.pinv(Jbar)@Jbar
            
            self.robot.update(self.joints)     

        print('----- ERROR', (self.tasks[0].getError()), self.ur)
        self.uv =  self.ur[0]/500
        self.publish_commands()


    def publish_commands(self):
        velocity = Float64MultiArray()
        x_vel = np.clip(self.uv[0], -self.v_max, self.v_max)
        y_vel = np.clip(self.uv[1], -self.v_max, self.v_max)
        velocity.data = np.array([x_vel, -1 * y_vel])
        self.pub_command_velocity.publish(velocity)
        print(f"---uv=({self.uv[0]},{self.uv[1]})")

if __name__ == '__main__':
    rospy.init_node('Resolved_rate_controller')
    node = MotionController()
    rospy.spin()
