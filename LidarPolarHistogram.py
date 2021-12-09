# -*- coding: utf-8 -*-
"""
Created on Mon Nov 15 15:43:07 2021

@author: Cavender Holt
"""

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time
import numpy as np
from matplotlib import cm
import matplotlib.pyplot as plt
import random


import copy

#Connect to coppeliasim
print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID != -1:
    print ('Connected to remote API server')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res,objs=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking)
    if res==sim.simx_return_ok:
        print ('Number of objects in the scene: ',len(objs))
    else:
        print ('Remote API function call returned with error code: ',res)

    
#Variable defines    
t = time.time()

fig = plt.figure()
ax = fig.add_subplot(projection='polar')
flag = 1

errL,left_motor = sim.simxGetObjectHandle(clientID,"Pioneer_p3dx_leftMotor",sim.simx_opmode_blocking)
errR,right_motor = sim.simxGetObjectHandle(clientID,"Pioneer_p3dx_rightMotor",sim.simx_opmode_blocking)

lVel = 0.0
rVel = 0.0
sign = 1

print(errL,errR)

plt.ion()
plt.show()

while time.time() - t < 1000:
        outlist = []
        
        returnCode, signalValue = sim.simxGetStringSignal(clientID, 'CurrentMeasureData', sim.simx_opmode_streaming)
        
        #unpack the lidar data from coppelliasim
        pointData = sim.simxUnpackFloats(signalValue)
        pointData = np.array(pointData)
        
        if len(pointData) == 0:
            continue
        
        #transform the point data since it was transformed into a 1d array when sent from copelliasim
        listLen = int(len(pointData)/3)
        TransformedPointData = np.reshape(pointData,(listLen,3))
        PolarPointData = np.zeros((listLen,2))
        
        #Transform to polar coordinates 
        #This gives us a view of what is around the robot in 360 degrees and the distance away is given by a radius
        PolarPointData[:,0] = np.sqrt(np.power(TransformedPointData[:,0],2) + np.power(TransformedPointData[:,2],2)) 
        PolarPointData[:,1] = np.arctan2(TransformedPointData[:,0],TransformedPointData[:,2])

        #this function is used to set the threshold for when an object is close enough to be considered to update the position of the robot
        for i in PolarPointData:
            if i[0] >= 1:
                i[0] = 10


        #transform the polar angles from radians to degrees
        anglesDegrees = PolarPointData[:,1] * 180 / np.pi

        degreeRanges = np.array([20 * i for i in range(-9,10,1)])
        radianRanges = np.radians(degreeRanges)

        print(radianRanges)

        confidenceCoefficient = 5 ** 2
        #Weights that determine how important distance is to the calculation
        a = 1/10
        b = 1/100
            
        #Calculate obstacle vectors for across discrete lidar point grid - This equation comes from the chapter in the vector filed histogram section
        ObstacleVectors = np.array(confidenceCoefficient * (a - b * PolarPointData[:,0]))
        
        
        #Calculate the density for each sector of the polar histogram - This equation also comes from the vector filed histogram section
        counter = 0
        SectorSums = np.zeros(18,dtype=float)
        for i in anglesDegrees:
            sectorIndex = int((i + 180) / 20)    
            SectorSums[sectorIndex] = SectorSums[sectorIndex] + ObstacleVectors[counter]
            counter = counter + 1
                

        
        
        #Use the polar histogram data to determine if the robot should go straight or turn
        flag = 1
        
        if SectorSums[8] > 7000 or SectorSums[9] > 7000 or SectorSums[10] > 7000 or SectorSums[7] > 7000:
            lVel = sign * 2.0
            rVel = sign * -2.0
            
        else:
            lVel = 5.0
            rVel = 5.0
            sign = 1 if random.random() < 0.5 else -1
        
                

        
        #Send updates to the velocity of the mobile robot wheels        
        rCode1 = sim.simxSetJointTargetVelocity(clientID,left_motor,lVel,sim.simx_opmode_streaming)
        rCode2 = sim.simxSetJointTargetVelocity(clientID,right_motor,rVel,sim.simx_opmode_streaming)
        
        #Draw the histogram
        ax.clear()
        ax.plot(radianRanges[:-1], SectorSums)
        fig.canvas.draw()
        plt.pause(0.001)
        
        
