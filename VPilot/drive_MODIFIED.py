#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from deepgtav.messages import Start, Stop, Scenario, Dataset, Commands, frame2numpy, GoToLocation, TeleportToLocation, SetCameraPositionAndRotation
from deepgtav.client import Client

import argparse
import time
import cv2

import matplotlib.pyplot as plt

class Model:
    def run(self,frame):
        return [1.0, 0.0, 0.0] # throttle, brake, steering

# Controls the DeepGTAV vehicle
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=None)
    parser.add_argument('-l', '--host', default='localhost', help='The IP where DeepGTAV is running')
    parser.add_argument('-p', '--port', default=8000, help='The port where DeepGTAV is running')
    # args = parser.parse_args()

    # TODO for running in VSCode
    args = parser.parse_args('')
    

    # Creates a new connection to DeepGTAV using the specified ip and port. 
    # If desired, a dataset path and compression level can be set to store in memory all the data received in a gziped pickle file.
    # We don't want to save a dataset in this case
    client = Client(ip=args.host, port=args.port)
    



    # We set the scenario to be in manual driving, and everything else random (time, weather and location). 
    # See deepgtav/messages.py to see what options are supported
    # scenario = Scenario(drivingMode=-1) #manual driving (Works, but is too slow to be used)
    # scenario = Scenario(drivingMode=786603) #automatic driving
    # scenario = Scenario(drivingMode=786603, vehicle="buzzard") #automatic driving
    # scenario = Scenario(drivingMode=786603, vehicle="buzzard", location=[-424.991, -522.049, 50]) #automatic driving
    scenario = Scenario(drivingMode=786603, vehicle="buzzard", location=[-2573.13916015625, 3292.256103515625, 13.241103172302246]) #automatic driving
    # scenario = Scenario(drivingMode=-1, vehicle="buzzard") #automatic driving

    dataset=Dataset(showBoxes=True, location=True)
    # dataset = Dataset(recordScenario=True)
    # dataset = Dataset(stationaryScene=True)
   


    # Send the Start request to DeepGTAV. Dataset is set as default, we only receive frames at 10Hz (320, 160)
    client.sendMessage(Start(scenario=scenario, dataset=dataset))
    

    # Adjustments for recording from UAV perspective
    client.sendMessage(SetCameraPositionAndRotation(z = -3, rot_x = -90))



    # Manual Control
    # client.sendMessage(Commands(throttle=1.))

    # Dummy agent
    model = Model()

    # Start listening for messages coming from DeepGTAV. We do it for 80 hours

    stoptime = time.time() + 80 * 3600
    while time.time() < stoptime:
        try:
            # We receive a message as a Python dictionary
            message = client.recvMessage()  
            print("vehicles: ", message["vehicles"])
            print("peds: ", message["peds"])
            print("location: ", message["location"])
            print("index: ", message["index"])
            print("focalLen: ", message["focalLen"])
            print("curPosition: ", message["curPosition"])
            print("seriesIndex: ", message["seriesIndex"])
            
            print("image_2: ", message["image_2"])
            frame = message["frame"]
            image = frame2numpy(frame, (1920,1080))
            cv2.imshow('img',image)
            cv2.waitKey(1) # wait 50ms
            
            # imgplot = plt.imshow(image)
            # plt.show()

            # client.sendMessage(Commands(throttle=1.))

            # The frame is a numpy array that can we pass through a CNN for example     
            # image = frame2numpy(message['frame'], (320,160))
            # commands = model.run(image)
            # We send the commands predicted by the agent back to DeepGTAV to control the vehicle
            # client.sendMessage(Commands(commands[0], commands[1], commands[2]))
        except KeyboardInterrupt:
            break
            
    # We tell DeepGTAV to stop
    client.sendMessage(Stop())
    client.close()
