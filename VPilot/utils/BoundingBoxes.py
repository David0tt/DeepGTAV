# Functionality concerning the bounding boxes


import cv2
import numpy as np

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



def add_bboxes(image, bboxes):
    """Add bounding boxes to an image

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

# parse the bbboxes from label_aug format to VisDrone format. This also catches some wrong class labels by looking at the models, e.g. SUVs as  truck
# TODO improve
def parseBBoxLabel_augToVisDrone(bboxes):
    items = bboxes.split("\n")
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


def convertBBoxVisDroneToYolo(bboxes):
    # convert from {'left' 'top' 'right' 'bottom'} in px to {x_center, y_center, width, height} 
    bboxes_new =  [[OBJECT_CATEGORY_TO_NUMBER[b['label']], (b['left'] + b['right']) / 2, (b['top'] + b['bottom']) / 2, b['right'] - b['left'], b['bottom'] - b['top']] for b in bboxes]
    # bboxes_new =  [[OBJECT_CATEGORY_TO_DEEPGTAV_NUMBER[b['label']], b['left'] + b['right'] / 2, b['top'] + b['bottom'] / 2, b['right'] - b['left'], b['bottom'] - b['top']] for b in bboxes if b[0] in OBJECT_CATEGORY_TO_DEEPGTAV_NUMBER]
    return bboxes_new

def convertBBoxesYolo_relative(bboxes_yolo, img_width, img_height):
    bboxes_new = [[b[0], b[1] / img_width, b[2] / img_height, b[3] / img_width, b[4] / img_height] for b in bboxes_yolo]
    return bboxes_new

def revertconvertBBoxVisDroneToYolo(bboxes):
    bboxes_new = [{'label': NUMBER_TO_OBJECT_CATEGORIES[b[0]], 'left': b[1] - b[3]/2, 'right': b[1] + b[3]/2, 'top': b[2] - b[4]/2, 'bottom': b[2] + b[4]/2} for b in bboxes]
    return bboxes_new

def revertConvertBBoxesYolo_relative(bboxes, img_width, img_height):
    bboxes_new = [[b[0], b[1] * img_width, b[2] * img_height, b[3] * img_width, b[4] * img_height] for b in bboxes]
    return bboxes_new

bbox_test = [[5, 0.474632, 0.828758, 0.104412, 0.100654],
            [5, 0.485662, 0.751634, 0.108824, 0.079739],
            [4, 0.484559, 0.686275, 0.113235, 0.078431],
            [4, 0.511029, 0.614379, 0.097794, 0.075817],
            [4, 0.267279, 0.660131, 0.100000, 0.073203],
            [4, 0.334926, 0.594118, 0.100735, 0.074510],
            [6, 0.309926, 0.481699, 0.134559, 0.116340],
            [4, 0.418382, 0.419608, 0.082353, 0.062745]
            ]
assert np.isclose(bbox_test, convertBBoxesYolo_relative(revertConvertBBoxesYolo_relative(bbox_test, 1920, 1080), 1920, 1080)).all()
assert np.isclose(bbox_test, convertBBoxesYolo_relative(convertBBoxVisDroneToYolo(revertconvertBBoxVisDroneToYolo(revertConvertBBoxesYolo_relative(bbox_test, 1920, 1080))), 1920, 1080)).all()


bbox_test2 = [{'label': 'truck', 'left': 240, 'right': 260, 'top': 450, 'bottom': 550},
            {'label': 'truck', 'left': 440, 'right': 460, 'top': 550, 'bottom': 650},
            {'label': 'truck', 'left': 540, 'right': 560, 'top': 450, 'bottom': 550},
            {'label': 'truck', 'left': 240, 'right': 260, 'top': 150, 'bottom': 250}]
assert bbox_test2 == revertconvertBBoxVisDroneToYolo(convertBBoxVisDroneToYolo(bbox_test2))





# fully converts bounding boxes from label_aug format to format as required by ultralytics yolo training
def convertBBoxesDeepGTAToYolo(bboxes):
    bboxes = parseBBoxLabel_augToVisDrone(bboxes)
    bboxes = convertBBoxVisDroneToYolo(bboxes)
    bboxes = convertBBoxesYolo_relative(bboxes, 1920, 1080)
    return bboxes



