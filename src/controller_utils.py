#!/usr/bin/python3
'''
This file contains functions to describe the forward kinematics,
the Jacobians of the swiftpro manipulator and the Manipulator class definition of the 
'''
import numpy as np 

'''Function that implements the DLS method over a matrix A'''

def DLS_Weights(A, damping, weights):

    W = np.eye(weights.shape[1])*weights.T

    return np.linalg.inv(W)@np.transpose(A)@np.linalg.inv(A@np.transpose(A) + (damping**2)*np.eye(len(A))) 

class Manipulator:
    def __init__(self,joints):
        self.joints = joints
        self.dof    = len(self.joints)
    
    def update(self,joints):
        self.joints = joints

    def getDOF(self):
        return self.dof

class Task: 
    '''
    Constructor

    Arguments:
    name (string): name of the taks
    desired (Numpy array): desired goal
    ffv (Numpy array): feedforward component
    K (Numpy array): gain values for the diagonal matrix  
    '''

    def __init__(self,name,desired,ffv,K):
        self.name = name
        self.sigma_d = desired
        self.ffv = ffv
        self.K = K

    def update(self,robot):
        pass

    # def isActive(self):
    #     return self.activation_value

    def setDesired(self,value):
        self.sigma_d = value

    def getDesired(self):
        return self.sigma_d

    def getFfv(self):
        return self.ffv

    def getJacobian(self):
        return self.J

    def getError(self):
        return self.err

    def getGainMatrix(self):
        return self.K
    
    def isActive(self):
        pass

    def Active(self,active):
        self.active = active

class BasePosition(Task):
    def __init__(self,name,desired,gain):
        ffv = np.zeros((1,1))
        K = np.diag([gain, gain])
        super().__init__(name,desired,ffv,K)
        self.active = 1
    
    def update(self,robot, modemCenter):
        yaw = modemCenter
        self.J = np.array([1,1]).reshape(1,2)
        self.err = np.array([320, 240]) - yaw

    def isActive(self):
        return self.active

   
        