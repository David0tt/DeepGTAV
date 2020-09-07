#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from deepgtav.messages import Start, Stop, Scenario, Dataset, Commands, frame2numpy, GoToLocation, TeleportToLocation, SetCameraPositionAndRotation
from deepgtav.messages import StartRecording, StopRecording
from deepgtav.messages import CreatePed, CreateVehicle

from deepgtav.client import Client

import argparse
import time
import cv2

import matplotlib.pyplot as plt

from PIL import Image

from random import uniform
from math import sqrt
import numpy as np

import os


NUMBER_TO_OBJECT_CATEGORIES = {0: 'ignored_regions',
                                1: 'pedestrian',
                                2: 'people',
                                3: 'bicycle',
                                4: 'car',
                                5: 'van',
                                6: 'truck',
                                7: 'tricycle',
                                8: 'awning-tricycle',
                                9: 'bus',
                                10: 'motor',
                                11: 'other'}

OBJECT_CATEGORY_TO_NUMBER = {v: k for k, v in NUMBER_TO_OBJECT_CATEGORIES.items()}



class Model:
    def run(self,frame):
        return [1.0, 0.0, 0.0] # throttle, brake, steering





def add_bboxes(image, bboxes):
    """Display image with object bounding boxes 

    bboxes as list of dicts, e.g.
        [{'category': 'Car',
          'left': '537',
          'top': '117',
          'right': '585',
          'bottom': '204'},
         {'category': 'Car',
          'left': '546',
          'top': '385',
          'right': '595',
          'bottom': '468'},
         {'category': 'Car',
          'left': '792',
          'top': '617',
          'right': '837',
          'bottom': '704'},
         {'category': 'Car',
          'left': '683',
          'top': '251',
          'right': '741',
          'bottom': '336'}]

    
    Args:
        image: the image to add bounding boxes into
        bboxes: the bounding box data
    """

    for bbox in bboxes:
        x1 = bbox['left']
        y1 = bbox['top']
        x2 = bbox['right']
        y2 = bbox['bottom']
        label = bbox['label']
        cv2.putText(image, label, (x1, y1+25), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), thickness = 2, lineType=cv2.LINE_AA) 
        cv2.rectangle(image, (x1, y1), (x2, y2), (255, 255, 0), 2)

    return image


    
def parseBBox2d(bbox2d):
    if bbox2d == None:
        return []
    items = bbox2d.split("\n")
    ret = []
    for item in items[:-1]:
        data = item.split(" ")
        # Indices can be found in /DeepGTAV-PreSIL/dataformat-augmented.txt
        label = data[0]
        left = int(data[4])
        top = int(data[5])
        right = int(data[6])
        bottom = int(data[7])

        # ignore 1920 1080 0 0 boxes
        if not (left > right or top > bottom):
            ret.append({"label": label,"left": left,"top": top,"right": right,"bottom": bottom})
    return ret


# img = add_bboxes(frame2numpy(message['frame'], (1920,1080)), parseBBox2d(message["bbox2d"]))
# cv2.imshow("test", img)
# cv2.waitKey() 
# cv2.destroyAllWindows()


# plt.imshow(img)
# img = Image.fromarray(img, 'RGB')
# img.show()

def convertBBoxYolo(bboxes):
    # convert from {'left' 'top' 'right' 'bottom'} in px to {x_center, y_center, width, height} 
    bboxes_new =  [[OBJECT_CATEGORY_TO_NUMBER[b['label']], (b['left'] + b['right']) / 2, (b['top'] + b['bottom']) / 2, b['right'] - b['left'], b['bottom'] - b['top']] for b in bboxes]
    # bboxes_new =  [[OBJECT_CATEGORY_TO_DEEPGTAV_NUMBER[b['label']], b['left'] + b['right'] / 2, b['top'] + b['bottom'] / 2, b['right'] - b['left'], b['bottom'] - b['top']] for b in bboxes if b[0] in OBJECT_CATEGORY_TO_DEEPGTAV_NUMBER]
    return bboxes_new

def convertBBoxesYolo_relative(bboxes_yolo, img_width, img_height):
    bboxes_new = [[b[0], b[1] / img_width, b[2] / img_height, b[3] / img_width, b[4] / img_height] for b in bboxes_yolo]
    return bboxes_new

def revertConvertBBoxYolo(bboxes):
    bboxes_new = [{'label': NUMBER_TO_OBJECT_CATEGORIES[b[0]], 'left': b[1] - b[3]/2, 'right': b[1] + b[3]/2, 'top': b[2] - b[4]/2, 'bottom': b[2] + b[4]/2} for b in bboxes]
    return bboxes_new

def revertConvertBBoxesYolo_relative(bboxes, img_width, img_height):
    bboxes_new = [[b[0], b[1] * img_width, b[2] * img_height, b[3] * img_width, b[4] * img_height] for b in bboxes]
    return bboxes_new
    

    
def generateNewTargetLocation(x_min, x_max, y_min, y_max):
    x_target = uniform(x_min, x_max)
    y_target = uniform(y_min, y_max)
    return x_target, y_target



def save_image_and_bbox(save_dir, filename, image, bboxes):
    img = Image.fromarray(image)

    bboxes = convertBBoxesYolo_relative(convertBBoxYolo(bboxes), 1920, 1080)
    label_texts = ["{:d} {:1.6f} {:1.6f} {:1.6f} {:1.6f}".format(*bbox) for bbox in bboxes]
    label_texts = "\n".join(label_texts)
    with open(os.path.join(save_dir, 'labels', filename + ".txt"), 'w') as file:
        file.write(label_texts)

# returns the highest run in the directory + 1
def getRunCount(save_dir):
    # TODO
    return 1

# Controls the DeepGTAV vehicle
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=None)
    parser.add_argument('-l', '--host', default='localhost', help='The IP where DeepGTAV is running')
    parser.add_argument('-p', '--port', default=8000, help='The port where DeepGTAV is running')
    parser.add_argument('-s', '--save_dir', default='E:\\Bachelorarbeit\\DataGeneration_DeepGTAV-PreSIL\\EXPORTDIR_OWN', help='The directory the generated data is saved to')
    # args = parser.parse_args()

    # TODO for running in VSCode
    args = parser.parse_args('')
    args.save_dir = os.path.normpath(args.save_dir)

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
    scenario = Scenario(drivingMode=786603, vehicle="buzzard", location=[245.23306274414062, -998.244140625, 29.205352783203125]) #automatic driving
    # scenario = Scenario(drivingMode=[786603, 20.0], vehicle="buzzard", location=[275.23306274414062, -998.244140625, 29.205352783203125]) #automatic driving
    # scenario = Scenario(drivingMode=-1, vehicle="buzzard") #automatic driving

    dataset=Dataset(showBoxes=True, location=True)
    # dataset = Dataset(recordScenario=True)
    # dataset = Dataset(stationaryScene=True)
   


    # Send the Start request to DeepGTAV. Dataset is set as default, we only receive frames at 10Hz (320, 160)
    client.sendMessage(Start(scenario=scenario, dataset=dataset))
    

    # Adjustments for recording from UAV perspective
    client.sendMessage(SetCameraPositionAndRotation(z = -3, rot_x = -90))

    # client.sendMessage(StartRecording())

    # Manual Control
    # client.sendMessage(Commands(throttle=1.))

    # Dummy agent
    model = Model()

    # Start listening for messages coming from DeepGTAV. We do it for 20 hours

    stoptime = time.time() + 20 * 3600
    count = 0
    bbox2d_old = ""
    errors = []



    # SETTINGS
    STARTING_COUNT = 20




    currentTravelHeight = 70
    # x_target, y_target = generateNewTargetLocation(-1960, 1900, -3360, 2000)
    x_target, y_target = (-182.139, -507.529)
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


            # if count % 20 == 0:
            #     client.sendMessage(CreateVehicle('blista', 5, 0, 0, 1, 1))
            
            # if count % 20 == 10:
            #     client.sendMessage(CreatePed(1, 5, 0, 0, 0))


            message = client.recvMessage()  
            
            # None message from utf-8 decode error
            # TODO this should be managed better
            if message == None:
                continue

            # Go to some random location in area 
            # x in [-1960, 1900]
            # y in [-3360, 2000]
            # This is the metropolitan area and some outskirts
            # locations can be found here https://www.gtagmodding.com/maps/gta5/

            # Generate a new Travelheight for every x frames (which are x / 10 recorded frames):
            # if count % 500 == 0:
                # currentTravelHeight = uniform(10, 150)
            
            
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
                correcting_height = True
                print("Correcting height")
            # elif correcting_height:
            #     # end height correction
            #     client.sendMessage(GoToLocation(x_target, y_target, currentTravelHeight))
            #     correcting_height = False
            else:
                # end height correction
                client.sendMessage(GoToLocation(x_target, y_target, estimated_ground_height + currentTravelHeight))
                correcting_height = False

            print("current target: ", x_target, y_target, currentTravelHeight)
            print("current location: ", message["location"])
            print("HeightAboveGround: ", message["HeightAboveGround"])

            # print("vehicles: ", message["vehicles"])
            # print("peds: ", message["peds"])
            # print("location: ", message["location"])
            # print("index: ", message["index"])
            # print("focalLen: ", message["focalLen"])
            # print("curPosition: ", message["curPosition"])
            # print("seriesIndex: ", message["seriesIndex"])
            # print("bbox2d: ", message["bbox2d"])

            if message["bbox2d"] != bbox2d_old:
                try: # Sometimes there are errors with the message, i catch those here

                    # save Data
                    # Use filename of the format [run]_[count] with padding, e.g. for the 512th image in the 21th run:
                    # 0021_000000512
                    filename = f'{run_count:04}' + '_' + f'{count:010}'
                    save_image_and_bbox(args.save_dir, filename, frame2numpy(message['frame'], (1920,1080)), parseBBox2d(message["bbox2d"]))

                    img = add_bboxes(frame2numpy(message['frame'], (1920,1080)), parseBBox2d(message["bbox2d"]))
                    cv2.imshow("test", img)
                    cv2.waitKey(1) 
                    bbox2d_old = message["bbox2d"]
                except Exception as e:
                    print(e)
                    errors.append(e)



            
            # imgplot = plt.imshow(img)
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

