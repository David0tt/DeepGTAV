#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from utils.Constants import IMG_WIDTH, IMG_HEIGHT


from deepgtav.messages import Start, Stop, Scenario, Dataset, Commands, frame2numpy, GoToLocation, TeleportToLocation, SetCameraPositionAndRotation
from deepgtav.messages import StartRecording, StopRecording, SetClockTime, SetWeather, CreatePed, CreateVehicle
from deepgtav.client import Client

from utils.BoundingBoxes import add_bboxes, parseBBox2d, convertBBoxesDeepGTAToYolo, parseBBox_YoloFormat_to_Image, parseBBox_to_List, revertParseBBox_to_List, combineBBoxesProcessedUnprocessed
from utils.utils import save_image_and_bbox, save_meta_data, getRunCount, generateNewTargetLocation
from utils.colors import pickRandomColor

import argparse
import time
import cv2
import matplotlib.pyplot as plt
from PIL import Image
from random import uniform
from math import sqrt
import numpy as np
import os

import base64
import open3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random


BOATS = ["Dinghy", "Dinghy2", "Dinghy3", "Dinghy4", "Jetmax", "Marquis", "Seashark", "Seashark2", "Seashark3", "Speeder", "Speeder2", "Squalo", # "Submersible", "Submersible2",
         "Suntrap", "Toro", "Toro2", "Tropic", "Tropic2", "Tug"]

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=None)
    parser.add_argument('-l', '--host', default='127.0.0.1', help='The IP where DeepGTAV is running')
    parser.add_argument('-p', '--port', default=8000, help='The port where DeepGTAV is running')
    parser.add_argument('-s', '--save_dir', default='G:\\EXPORTDIR\\ExportWater_ProcessedPeds', help='The directory the generated data is saved to')
    # args = parser.parse_args()

    # TODO for running in VSCode
    args = parser.parse_args('')
    
    args.save_dir = os.path.normpath(args.save_dir)

    client = Client(ip=args.host, port=args.port)
    
    # scenario = Scenario(drivingMode=786603, vehicle="buzzard", location=[245.23306274414062, -998.244140625, 29.205352783203125]) #automatic driving
    # scenario = Scenario(drivingMode=0, vehicle="buzzard", location=[245.23306274414062, -998.244140625, 29.205352783203125]) #automatic driving
    scenario = Scenario(drivingMode=1, vehicle="buzzard", location=[245.23306274414062, -998.244140625, 29.205352783203125]) #automatic driving
    # dataset=Dataset(location=True, time=True, instanceSegmentationImageColor=True, exportBBox2D=True, occlusionImage=True, segmentationImage=True) #,exportStencilImage=True, exportLiDAR=True, maxLidarDist=50)
    # dataset=Dataset(location=True, time=True, exportBBox2D=True, segmentationImage=True, instanceSegmentationImageColor=True) #exportIndividualStencilImages=True)
    dataset=Dataset(location=True, time=True, exportBBox2D=True, exportBBox2DUnprocessed=True, segmentationImage=True, exportStencilImage=True) # , exportIndividualStencilImages=True) #exportIndividualStencilImages=True)
    
    # dataset=Dataset(location=True, time=True, exportLiDAR=True, maxLidarDist=120) #exportIndividualStencilImages=True)
    
    
    client.sendMessage(Start(scenario=scenario, dataset=dataset))

    CAMERA_OFFSET_Z = -20
    CAMERA_ROT_X_LOW = -90
    CAMERA_ROT_X_HIGH = -20

    # Adjustments for recording from UAV perspective
    client.sendMessage(SetCameraPositionAndRotation(z = CAMERA_OFFSET_Z, rot_x = uniform(CAMERA_ROT_X_LOW, CAMERA_ROT_X_HIGH)))

    count = 0
    bbox2d_old = ""
    errors = []


    # SETTINGS
    STARTING_COUNT = 100

    TRAVEL_HEIGHT_LOW = 20
    TRAVEL_HEIGHT_HIGH = 60

    currentTravelHeight = uniform(TRAVEL_HEIGHT_LOW, TRAVEL_HEIGHT_HIGH)

    xAr_min, xAr_max, yAr_min, yAr_max = -2800, -1800, -2500, -1300
    x_start, y_start = generateNewTargetLocation(xAr_min, xAr_max, yAr_min, yAr_max)
    x_target, y_target = generateNewTargetLocation(xAr_min, xAr_max, yAr_min, yAr_max)


    if not os.path.exists(os.path.join(args.save_dir, 'images')):
        os.makedirs(os.path.join(args.save_dir, 'images'))
    if not os.path.exists(os.path.join(args.save_dir, 'labels')):
        os.makedirs(os.path.join(args.save_dir, 'labels'))
    if not os.path.exists(os.path.join(args.save_dir, 'meta_data')):
        os.makedirs(os.path.join(args.save_dir, 'meta_data'))

    if not os.path.exists(os.path.join(args.save_dir, 'image')):
        os.makedirs(os.path.join(args.save_dir, 'image'))
    if not os.path.exists(os.path.join(args.save_dir, 'SegmentationAndBBox')):
        os.makedirs(os.path.join(args.save_dir, 'SegmentationAndBBox'))
    
    
        

    run_count = getRunCount(args.save_dir)


    messages = []
    emptybbox = []

    while True:
        try:
            count += 1
            print("count: ", count)

            # Only record every 10th frame
            if count > STARTING_COUNT and count % 10 == 0:
                client.sendMessage(StartRecording())
            if count > STARTING_COUNT and count % 10 == 1:
                client.sendMessage(StopRecording())

            if count % 30 == 0:
                for i in range (-40, 40, 10):
                    rand_x = uniform(-10,10)
                    rand_y = uniform(-10,10)
                    client.sendMessage(CreatePed(150+rand_x, i+rand_y, heading=uniform(-180,180)))
            if count % 30 == 4:
                for i in [-30, 0, 30]:
                    rand_x = uniform(-10,10)
                    rand_y = uniform(-10,10)
                    client.sendMessage(CreateVehicle(random.choice(BOATS), relativeForward=190+rand_x, relativeRight=i+rand_y, color=pickRandomColor(), color2=pickRandomColor(), heading=uniform(-180,180)))


            if count == 2:
                client.sendMessage(TeleportToLocation(x_start, y_start, 200))
                # client.sendMessage(TeleportToLocation(-24d89, -513, 200))
                client.sendMessage(GoToLocation(x_target, y_target, currentTravelHeight))
            if count == 4:
                client.sendMessage(SetClockTime(int(uniform(0,24))))





            message = client.recvMessage()  
            
            # None message from utf-8 decode error
            if message == None:
                continue


            # Generate a new Travelheight for every x frames (which are x / 10 recorded frames):
            if count % 500 == 0:
                currentTravelHeight = uniform(TRAVEL_HEIGHT_LOW, TRAVEL_HEIGHT_HIGH)
            
            if count % 700 == 0:
                client.sendMessage(SetCameraPositionAndRotation(z = CAMERA_OFFSET_Z, rot_x = uniform(CAMERA_ROT_X_LOW, CAMERA_ROT_X_HIGH)))



            estimated_ground_height = message["location"][2] - message["HeightAboveGround"]
            # print("estimated_ground_height: ", estimated_ground_height)
            # print("heightAboveGround: ", message["HeightAboveGround"])

            # Sometimes Generate a new location, to prevent getting stuck
            if count % 600 == 0:
                x_target, y_target = generateNewTargetLocation(xAr_min, xAr_max, yAr_min, yAr_max)
                client.sendMessage(GoToLocation(x_target, y_target, estimated_ground_height + currentTravelHeight))



            # if we are near the target location generate a new target location
            if sqrt((message["location"][0] - x_target) ** 2 + (message["location"][1] - y_target) ** 2) < 10:
                x_target, y_target = generateNewTargetLocation(xAr_min, xAr_max, yAr_min, yAr_max)
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



            # Plot Segmentation Image and Bounding Box image overlayed for testing 
            if message["bbox2d"] != bbox2d_old and message["bbox2d"] != None:
                try: # Sometimes there are errors with the message, i catch those here

                    # save Data
                    # Use filename of the format [run]_[count] with padding, e.g. for the 512th image in the 21th run:
                    # 0021_000000512
                    filename = f'{run_count:04}' + '_' + f'{count:010}'
                    
                    # print(message["bbox2dUnprocessed"])

                    # bboxes = combineBBoxesProcessedUnprocessed(message["bbox2d"], message["bbox2dUnprocessed"])
                    bboxes =  convertBBoxesDeepGTAToYolo(message["bbox2d"], include_boats=True)

                    if bboxes != "":
                        save_image_and_bbox(args.save_dir, filename, frame2numpy(message['frame'], (IMG_WIDTH,IMG_HEIGHT)), bboxes)
                        save_meta_data(args.save_dir, filename, message["location"], message["HeightAboveGround"], message["CameraPosition"], message["CameraAngle"], message["time"], "CLEAR")
                        
                    
                    # img = add_bboxes(frame2numpy(message['frame'], (IMG_WIDTH,IMG_HEIGHT)), parseBBox_YoloFormat_to_Image(bboxes))
                    # cv2.imshow("test", img)
                    # cv2.waitKey(1) 
                    bbox2d_old = message["bbox2d"]

                except Exception as e:
                    print(e)
                    errors.append(e)

            

            
        except KeyboardInterrupt:
            break
            
    # We tell DeepGTAV to stop
    client.sendMessage(Stop())
    client.close()




# Manual showing of message
def showmessage(idx):
    message = messages[idx]

    bboxes = convertBBoxesDeepGTAToYolo(message["bbox2d"])
    bbox_image = add_bboxes(frame2numpy(message['frame'], (IMG_WIDTH,IMG_HEIGHT)), parseBBox_YoloFormat_to_Image(bboxes))
    
    nparr = np.fromstring(base64.b64decode(message["instanceSegmentationImageColor"]), np.uint8)
    segmentationImage = cv2.imdecode(nparr, cv2.IMREAD_ANYCOLOR)
    
    dst = cv2.addWeighted(bbox_image, 0.5, segmentationImage, 0.5, 0.0)
    # plt.figure(figsize=(15,15))
    # plt.imshow(cv2.cvtColor(dst, cv2.COLOR_BGR2RGB))
    # plt.show()
    cv2.imshow("CombinedImage", dst)
    cv2.waitKey(1)

def showBBox(idx):
    message = messages[idx]
    bboxes = convertBBoxesDeepGTAToYolo(message["bbox2d"])
    bbox_image = add_bboxes(frame2numpy(message['frame'], (IMG_WIDTH,IMG_HEIGHT)), parseBBox_YoloFormat_to_Image(bboxes))
    cv2.imshow("BBoxImage", bbox_image)
    cv2.waitKey(1)

def showOcclusionImage(idx):
    message = messages[idx]
    if message["occlusionImage"] != None and message["occlusionImage"] != "":
        # print(message["occlusionImage"])
        nparr = np.fromstring(base64.b64decode(message["occlusionImage"]), np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_ANYCOLOR)
        cv2.imshow("occlusionImage", img)
        cv2.waitKey(1)

def showStencil1(idx):
    message = messages[idx]
    if message["segmentationImage"] != None and message["segmentationImage"] != "":
        # print(message["occlusionImage"])
        nparr = np.fromstring(base64.b64decode(message["segmentationImage"]), np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_ANYCOLOR)
        cv2.imshow("segmentationImage", img)
        cv2.waitKey(1)


def ShowUsefulMessages():
    for idx in range(len(messages)):
        message = messages[idx]
        if message["bbox2d"] != None:
            print(idx)



# For parsing the stencil message with individual stencil values
# stencilMessage = stencilMessage[8:]

def parseMultipleStencil(stencilMessage):
    msgs = stencilMessage.split("StencilVal")
    vals = [msg[:2] for msg in msgs]
    imgs = [msg[2:] for msg in msgs]


    nparr = np.fromstring(base64.b64decode(imgs[1]), np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_ANYCOLOR)
    cv2.imshow("img1", img)
    cv2.waitKey(1)

    nparr = np.fromstring(base64.b64decode(imgs[2]), np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_ANYCOLOR)
    cv2.imshow("img2", img)
    cv2.waitKey(1)

    nparr = np.fromstring(base64.b64decode(imgs[3]), np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_ANYCOLOR)
    cv2.imshow("img3", img)
    cv2.waitKey(3)

    nparr = np.fromstring(base64.b64decode(imgs[4]), np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_ANYCOLOR)
    cv2.imshow("img4", img)
    cv2.waitKey(4)

    nparr = np.fromstring(base64.b64decode(imgs[5]), np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_ANYCOLOR)
    cv2.imshow("img5", img)
    cv2.waitKey(5)

    nparr = np.fromstring(base64.b64decode(imgs[6]), np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_ANYCOLOR)
    cv2.imshow("img6", img)
    cv2.waitKey(6)


cv2.destroyAllWindows()





