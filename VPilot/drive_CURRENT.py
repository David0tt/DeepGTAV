#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from deepgtav.messages import Start, Stop, Scenario, Dataset, Commands, frame2numpy, GoToLocation, TeleportToLocation, SetCameraPositionAndRotation
from deepgtav.messages import StartRecording, StopRecording
from deepgtav.client import Client

from utils.BoundingBoxes import add_bboxes, parseBBox2d
# import utils.BoundingBoxes 

import argparse
import time
import cv2

import matplotlib.pyplot as plt

from PIL import Image

from random import uniform
from math import sqrt
import numpy as np

import os



# saves image in np format and bboxes in string format
def save_image_and_bbox(save_dir, filename, image, bboxes):
    # convert image BGR -> RGB
    image = image[...,::-1]

    img = Image.fromarray(image)
    img.save(os.path.join(save_dir, 'images', filename + ".jpg"))
    # bboxes = convertBBoxesYolo_relative(convertBBoxYolo(bboxes), 1920, 1080)
    # label_texts = ["{:d} {:1.6f} {:1.6f} {:1.6f} {:1.6f}".format(*bbox) for bbox in bboxes]
    # label_texts = "\n".join(label_texts)

    # TODO this is temporary, thill i correctly parse the bbox texts
    label_texts = bboxes
    with open(os.path.join(save_dir, 'labels', filename + ".txt"), 'w') as file:
        file.write(label_texts)



# returns the highest run in the directory + 1
def getRunCount(save_dir):
    files = os.listdir(os.path.join(save_dir, 'images'))
    files = [int(f[:4]) for f in files]
    return max(files) + 1




# Go to some random location in area 
# x in [-1960, 1900]
# y in [-3360, 2000]
# This is the metropolitan area and some outskirts
# locations can be found here https://www.gtagmodding.com/maps/gta5/

def generateNewTargetLocation(x_min=-1960, x_max=1900, y_min=-3360, y_max=2000):
    x_target = uniform(x_min, x_max)
    y_target = uniform(y_min, y_max)
    return x_target, y_target


# Controls the DeepGTAV vehicle
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=None)
    parser.add_argument('-l', '--host', default='localhost', help='The IP where DeepGTAV is running')
    parser.add_argument('-p', '--port', default=8000, help='The port where DeepGTAV is running')
    # parser.add_argument('-s', '--save_dir', default='E:\\Bachelorarbeit\\DataGeneration_DeepGTAV-PreSIL\\EXPORTDIR_OWN', help='The directory the generated data is saved to')
    parser.add_argument('-s', '--save_dir', default='Z:\\DeepGTAV-EXPORTDIR-TEST', help='The directory the generated data is saved to')
    # args = parser.parse_args()

    # TODO for running in VSCode
    args = parser.parse_args('')
    args.save_dir = os.path.normpath(args.save_dir)
    

    client = Client(ip=args.host, port=args.port)
    



    # We set the scenario to be in manual driving, and everything else random (time, weather and location). 
    # See deepgtav/messages.py to see what options are supported
    # scenario = Scenario(drivingMode=-1) #manual driving (Works, but is too slow to be used)
    # scenario = Scenario(drivingMode=786603) #automatic driving
    # scenario = Scenario(drivingMode=786603, vehicle="buzzard") #automatic driving
    # scenario = Scenario(drivingMode=786603, vehicle="buzzard", location=[-424.991, -522.049, 50]) #automatic driving
    scenario = Scenario(drivingMode=786603, vehicle="buzzard", location=[245.23306274414062, -998.244140625, 29.205352783203125]) #automatic driving
    # scenario = Scenario(drivingMode=[786603, 20.0], vehicle="buzzard", location=[275.23306274414062, -998.244140625, 29.205352783203125]) #automatic driving
    # scenario = Scenario(drivingMode=-1, vehicle="buzzard") #automatic driving

    dataset=Dataset(showBoxes=True, location=True)

    client.sendMessage(Start(scenario=scenario, dataset=dataset))
    

    # Adjustments for recording from UAV perspective
    client.sendMessage(SetCameraPositionAndRotation(z = -3, rot_x = -90))


    # Start listening for messages coming from DeepGTAV. We do it for 20 hours

    stoptime = time.time() + 20 * 3600
    count = 0
    bbox2d_old = ""
    errors = []



    # SETTINGS
    STARTING_COUNT = 20




    currentTravelHeight = 100

    x_target, y_target = generateNewTargetLocation(-1960, 1900, -3360, 2000)
    # x_target, y_target = (-182.139, -507.529)
    correcting_height = False

    if not os.path.exists(os.path.join(args.save_dir, 'images')):
        os.makedirs(os.path.join(args.save_dir, 'images'))
    if not os.path.exists(os.path.join(args.save_dir, 'labels')):
        os.makedirs(os.path.join(args.save_dir, 'labels'))

    run_count = getRunCount(args.save_dir)


    while time.time() < stoptime:
        try:
            # We receive a message as a Python dictionary
            count += 1
            print("count: ", count)

            # Only record every 10th frame
            if count > STARTING_COUNT and count % 10 == 0:
                client.sendMessage(StartRecording())
            if count > STARTING_COUNT and count % 10 == 1:
                client.sendMessage(StopRecording())


            if count == 2:
                client.sendMessage(TeleportToLocation(x_target, y_target, 200))

            message = client.recvMessage()  
            
            # None message from utf-8 decode error
            # TODO this should be managed better
            if message == None:
                continue


            # Generate a new Travelheight for every x frames (which are x / 10 recorded frames):
            if count % 500 == 0:
                currentTravelHeight = uniform(10, 150)
            
            
            estimated_ground_height = message["location"][2] - message["HeightAboveGround"]

            # if we are near the target location generate a new target location
            if sqrt((message["location"][0] - x_target) ** 2 + (message["location"][1] - y_target) ** 2) < 10:
                x_target, y_target = generateNewTargetLocation(-1960, 1900, -3360, 2000)
                client.sendMessage(GoToLocation(x_target, y_target, estimated_ground_height + currentTravelHeight))
                print("Going to new loctation: ", x_target, y_target, currentTravelHeight)



            # keep the currentTravelHeight under the wanted one
            # Move a little bit in the desired direction but primarily correct the height
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

            # print("current target: ", x_target, y_target, currentTravelHeight)
            # print("current location: ", message["location"])
            # print("HeightAboveGround: ", message["HeightAboveGround"])

            # print("vehicles: ", message["vehicles"])
            # print("peds: ", message["peds"])
            # print("location: ", message["location"])
            # print("index: ", message["index"])
            # print("focalLen: ", message["focalLen"])
            # print("curPosition: ", message["curPosition"])
            # print("seriesIndex: ", message["seriesIndex"])
            # print("bbox2d: ", message["bbox2d"])

            if message["bbox2d"] != bbox2d_old and message["bbox2d"] != None:
                try: # Sometimes there are errors with the message, i catch those here

                    # save Data
                    # Use filename of the format [run]_[count] with padding, e.g. for the 512th image in the 21th run:
                    # 0021_000000512
                    filename = f'{run_count:04}' + '_' + f'{count:010}'
                    save_image_and_bbox(args.save_dir, filename, frame2numpy(message['frame'], (1920,1080)), message["bbox2d"])


                    img = add_bboxes(frame2numpy(message['frame'], (1920,1080)), parseBBox2d(message["bbox2d"]))
                    cv2.imshow("test", img)
                    cv2.waitKey(1) 
                    bbox2d_old = message["bbox2d"]
                except Exception as e:
                    print(e)
                    errors.append(e)

            
        except KeyboardInterrupt:
            break
            
    # We tell DeepGTAV to stop
    client.sendMessage(Stop())
    client.close()

