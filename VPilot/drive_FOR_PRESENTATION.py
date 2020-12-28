#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from utils.Constants import IMG_WIDTH, IMG_HEIGHT


from deepgtav.messages import Start, Stop, Scenario, Dataset, Commands, frame2numpy, GoToLocation, TeleportToLocation, SetCameraPositionAndRotation
from deepgtav.messages import StartRecording, StopRecording, SetClockTime, SetWeather
from deepgtav.client import Client

from utils.BoundingBoxes import add_bboxes, parseBBox2d, convertBBoxesDeepGTAToYolo, parseBBox_YoloFormat_to_Image
from utils.utils import save_image_and_bbox, save_meta_data, getRunCount, generateNewTargetLocation

import argparse
import time
import cv2
import matplotlib.pyplot as plt
from PIL import Image
from random import uniform
from math import sqrt
import numpy as np
import os



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=None)
    parser.add_argument('-l', '--host', default='localhost', help='The IP where DeepGTAV is running')
    parser.add_argument('-p', '--port', default=8000, help='The port where DeepGTAV is running')
    parser.add_argument('-s', '--save_dir', default='E:\\Bachelorarbeit\\DataGeneration_DeepGTAV-PreSIL\\EXPORTDIR\\ExportPresentation', help='The directory the generated data is saved to')
    # args = parser.parse_args()

    # TODO for running in VSCode
    args = parser.parse_args('')
    
    args.save_dir = os.path.normpath(args.save_dir)

    client = Client(ip=args.host, port=args.port)
    
    scenario = Scenario(drivingMode=786603, vehicle="buzzard", location=[245.23306274414062, -998.244140625, 29.205352783203125]) #automatic driving
    dataset=Dataset(location=True, time=True)
    client.sendMessage(Start(scenario=scenario, dataset=dataset))
    

    # Adjustments for recording from UAV perspective
    client.sendMessage(SetCameraPositionAndRotation(z = -20, rot_x = -40))

    count = 0
    bbox2d_old = ""
    errors = []


    # SETTINGS

    currentTravelHeight = 40
    x_start, y_start = -388, 0
    x_target, y_target = 1165, -553


    if not os.path.exists(os.path.join(args.save_dir, 'images')):
        os.makedirs(os.path.join(args.save_dir, 'images'))
    if not os.path.exists(os.path.join(args.save_dir, 'labels')):
        os.makedirs(os.path.join(args.save_dir, 'labels'))
    if not os.path.exists(os.path.join(args.save_dir, 'meta_data')):
        os.makedirs(os.path.join(args.save_dir, 'meta_data'))
        

    run_count = getRunCount(args.save_dir)


    while True:
        try:
            count += 1
            print("count: ", count)

            # Only record every 10th frame
            if count > 50 and count % 10 == 0:
                client.sendMessage(StartRecording())
            if count > 50 and count % 10 == 1:
                client.sendMessage(StopRecording())


            if count == 2:
                client.sendMessage(TeleportToLocation(-388, 0, 200))
                client.sendMessage(GoToLocation(1165, -553, 40))

            if count == 4:
                client.sendMessage(SetClockTime(12))

            if count == 150:
                client.sendMessage(SetClockTime(0))

            if count == 200:
                client.sendMessage(SetClockTime(19))


            if count == 250:
                currentTravelHeight = 25

            if count == 300:
                currentTravelHeight = 100

            if count == 380:
                currentTravelHeight = 40


            if count == 400:
                client.sendMessage(SetCameraPositionAndRotation(z = -20, rot_x = -90))

            if count == 450:
                client.sendMessage(SetCameraPositionAndRotation(z = -20, rot_x = -20))



            message = client.recvMessage()  
            
            # None message from utf-8 decode error
            if message == None:
                continue


            # keep the currentTravelHeight under the wanted one
            # Move a little bit in the desired direction but primarily correct the height
            estimated_ground_height = message["location"][2] - message["HeightAboveGround"]
            if message["HeightAboveGround"] > currentTravelHeight + 3 or message["HeightAboveGround"] < currentTravelHeight - 3:
                direction = np.array([x_target - message["location"][0], y_target - message["location"][1]])
                direction = direction / np.linalg.norm(direction)
                direction = direction * 50
                x_temporary = message["location"][0] + direction[0]
                y_temporary = message["location"][1] + direction[1]
                client.sendMessage(GoToLocation(x_temporary, y_temporary, estimated_ground_height + currentTravelHeight))
                print("Correcting height")
            else:
                client.sendMessage(GoToLocation(x_target, y_target, estimated_ground_height + currentTravelHeight))

            if message["bbox2d"] != None:
                try:
                    bboxes = convertBBoxesDeepGTAToYolo(message["bbox2d"])
                except Exception as e:
                    errors.append(e)
                if bboxes != bbox2d_old:
                    bbox2d_old = bboxes
                    try: # Sometimes there are errors with the message, i catch those here

                        # save Data
                        filename = f'{run_count:04}' + '_' + f'{count:010}'
                        if bboxes != "":
                            save_image_and_bbox(args.save_dir, filename, frame2numpy(message['frame'], (IMG_WIDTH,IMG_HEIGHT)), bboxes)
                            save_meta_data(args.save_dir, filename, message["location"], message["HeightAboveGround"], message["CameraPosition"], message["CameraAngle"], message["time"], "CLEAR")
                            
                        # Show image with bbox                    
                        img = add_bboxes(frame2numpy(message['frame'], (IMG_WIDTH,IMG_HEIGHT)), parseBBox_YoloFormat_to_Image(bboxes))
                        cv2.imshow("test", img)
                        cv2.waitKey(1) 
                        
                    except Exception as e:
                        print(e)
                        errors.append(e)

            
        except KeyboardInterrupt:
            break
            
    # We tell DeepGTAV to stop
    client.sendMessage(Stop())
    client.close()

