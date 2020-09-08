# Functionality concerning the bounding boxes


import cv2
import numpy as np
import os

# I use VisDrone Categories as follows:
OBJECT_CATEGORY_TO_NUMBER = {'pedestrian': 0,
                             'people': 1,
                             'bicycle': 2,
                             'car': 3,
                             'van': 4,
                             'truck': 5,
                             'tricycle': 6,
                             'awning-tricycle': 7,
                             'bus': 8,
                             'motor': 9}


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

with open(os.path.normpath("utils/vehicle_names_and_categories.csv"), "r") as namefile:
    namedata = namefile.read()
namedata = namedata.split("\n")
namedata = [n.split(",") for n in namedata]
namedata = {n[0].lower(): n[2] for n in namedata}
VEHICLE_NAME_TO_CATEGORY = namedata

# This mapping is made according to 
# https://wiki.gtanet.work/index.php?title=Vehicle_Models
GTAV_CATEGORY_TO_VISDRONE_CATEGORY = {'Boats': 'UNRECOGNIZED_CATEGORY', # 'boat', # This should not happen
                                      'Commercials': 'truck',
                                      'Compacts': 'car',
                                      'Coupes': 'car',
                                      'Cycles': 'bicycle',
                                      'Emergency': 'MANUAL_CHECK',
                                      'Helicopters': 'UNRECOGNIZED_CATEGORY', # 'helicopter'
                                      'Industrial': 'MANUAL_CHECK',
                                      'Military': 'MANUAL_CHECK',
                                      'Motorcycles': 'motor',
                                      'Muscle': 'MANUAL_CHECK',
                                      'Off-Road': 'MANUAL_CHECK',
                                      'Planes': 'UNRECOGNIZED_CATEGORY',
                                      'SUVs': 'van',
                                      'Sedans': 'car',
                                      'Service': 'MANUAL_CHECK',
                                      'Sports': 'car',
                                      'Sports Classic': 'car',
                                      'Sports Classics': 'car',
                                      'Super': 'car',
                                      'Trailer': 'UNRECOGNIZED_CATEGORY',
                                      'Trains': 'UNRECOGNIZED_CATEGORY',
                                      'Utility': 'MANUAL_CHECK',
                                      'Vans': 'van',
                                      'Uncategorized': 'UNRECOGNIZED_CATEGORY'}






                                      


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


MANUAL_CATEGORY = {#Emergency:
                   'Ambulance': 'truck',
                   'FBI': 'car',
                   'FBI2': 'van',
                   'FireTruck': 'truck',
                   'PBus': 'bus',
                   'Police': 'car',
                   'Police2': 'car',
                   'Police3': 'car',
                   'Police4': 'car',
                   'PoliceOld1': 'car',
                   'PoliceOld2': 'car',
                   'PoliceT': 'van',
                   'Policeb': 'motor',
                   'Polmav': 'UNRECOGNIZED_CATEGORY', # 'helicopter'
                   'Pranger': 'van',
                   'Predator': 'UNRECOGNIZED_CATEGORY', # 'boat'
                   'Riot': 'truck',
                   'Sheriff': 'car',
                   'Sheriff2': 'van',
                   #Industrial:
                   'Bulldozer': 'UNRECOGNIZED_CATEGORY', # other
                   'Cutter': 'UNRECOGNIZED_CATEGORY', #other
                   'Dump': 'truck',
                   'Flatbed': 'truck',
                   'Guardian': 'truck',
                   'Handler': 'UNRECOGNIZED_CATEGORY', # other
                   'Mixer': 'truck',
                   'Mixer2': 'truck',
                   'Rubble': 'truck',
                   'TipTruck': 'truck',
                   'TipTruck2': 'truck',
                    #Service:
                    'Airbus': 'bus',
                    'Brickade': 'truck',
                    'Bus': 'bus',
                    'Coach': 'bus',
                    'Rallytruck': 'truck',
                    'RentalBus': 'bus',
                    'Taxi': 'car',
                    'Tourbus': 'bus',
                    'Trash': 'truck',
                    'Trash2': 'truck',
                    #Utility:
                    'Airtug': 'UNRECOGNIZED_CATEGORY',
                    'Caddy': 'UNRECOGNIZED_CATEGORY',
                    'Caddy2': 'UNRECOGNIZED_CATEGORY',
                    'Caddy3': 'UNRECOGNIZED_CATEGORY',
                    'Docktug': 'UNRECOGNIZED_CATEGORY',
                    'Forklift': 'UNRECOGNIZED_CATEGORY',
                    'Mower': 'UNRECOGNIZED_CATEGORY',
                    'Ripley': 'truck',
                    'Sadler': 'van',
                    'Scrap': 'truck',
                    'TowTruck': 'truck',
                    'TowTruck2': 'truck',
                    'Tractor': 'UNRECOGNIZED_CATEGORY',
                    'Tractor2': 'UNRECOGNIZED_CATEGORY',
                    'Tractor3': 'UNRECOGNIZED_CATEGORY',
                    'TrailerLarge': 'UNRECOGNIZED_CATEGORY',
                    'TrailerS4': 'UNRECOGNIZED_CATEGORY',
                    'UtiliTruck': 'truck',
                    'UtiliTruck3': 'truck',
                    'UtiliTruck2': 'truck'}
MANUAL_CATEGORY = {k.lower(): v for k,v in MANUAL_CATEGORY.items()}

def getLabelFromObjectName(obj_name):
    # Make function case insensitive (all lists are lower case)
    obj_name = obj_name.lower()

    gtav_category = VEHICLE_NAME_TO_CATEGORY[obj_name]
    obj_category = GTAV_CATEGORY_TO_VISDRONE_CATEGORY[gtav_category]
    if obj_category == 'MANUAL_CHECK':
        if gtav_category == 'Military':
            MILITARY_NAMES = {'Rhino'}
            MILITARY_NAMES = {v.lower() for v in MILITARY_NAMES}
            if obj_name in MILITARY_NAMES:
                obj_category = 'UNRECOGNIZED_CATEGORY'
            else:
                obj_category = 'truck'
        elif gtav_category == 'Muscle':
            MUSCLE_NAMES = {'Moonbeam', 'Moonbeam2', 'RatLoader', 'RatLoader2', 'Sadler2', 'SlamVan', 'SlamVan2', 'SlamVan3'}
            MUSCLE_NAMES = {v.lower() for v in MUSCLE_NAMES}
            if obj_name in MUSCLE_NAMES:
                obj_category = 'van'
            else:
                obj_category = 'car'
        elif gtav_category == 'Off-Road':
            OFF_ROAD_NAMES = {'Bifta', 'Blazer', 'Blazer2', 'Blazer3', 'Blazer5', 'Dune4', 'Dune5'}
            OFF_ROAD_NAMES = {v.lower() for v in OFF_ROAD_NAMES}
            if obj_name in OFF_ROAD_NAMES:
                obj_category = 'UNRECOGNIZED_CATEGORY'
            else:
                obj_category = 'van'

        else:
            obj_category = MANUAL_CATEGORY[obj_name]
    
    return obj_category


assert getLabelFromObjectName('blista') == 'car'
assert getLabelFromObjectName('dinghy') == 'UNRECOGNIZED_CATEGORY'
assert getLabelFromObjectName('Dinghy') == 'UNRECOGNIZED_CATEGORY'
assert getLabelFromObjectName('Biff') == 'truck'
assert getLabelFromObjectName('Pounder') == 'truck'
assert getLabelFromObjectName('Issi2') == 'car'
assert getLabelFromObjectName('Sentinel2') == 'car'
assert getLabelFromObjectName('Bmx') == 'bicycle'
assert getLabelFromObjectName('cruiser') == 'bicycle'
assert getLabelFromObjectName('Ambulance') == 'truck'
assert getLabelFromObjectName('Pranger') == 'van'
assert getLabelFromObjectName('Frogger2') == 'UNRECOGNIZED_CATEGORY'
assert getLabelFromObjectName('Flatbed') == 'truck'
assert getLabelFromObjectName('Barracks3') == 'truck'
assert getLabelFromObjectName('Akuma') == 'motor'
assert getLabelFromObjectName('Daemon') == 'motor'
assert getLabelFromObjectName('Blade') == 'car'
assert getLabelFromObjectName('RatLoader2') == 'van'
assert getLabelFromObjectName('Insurgent') == 'van'
assert getLabelFromObjectName('Blazer2') == 'UNRECOGNIZED_CATEGORY'
assert getLabelFromObjectName('Besra') == 'UNRECOGNIZED_CATEGORY'
assert getLabelFromObjectName('Luxor') == 'UNRECOGNIZED_CATEGORY'
assert getLabelFromObjectName('BJXL') == 'van'
assert getLabelFromObjectName('Dubsta2') == 'van'
assert getLabelFromObjectName('Fugitive') == 'car'
assert getLabelFromObjectName('Stanier') == 'car'
assert getLabelFromObjectName('Airbus') == 'bus'
assert getLabelFromObjectName('Coach') == 'bus'
assert getLabelFromObjectName('Trash') == 'truck'
assert getLabelFromObjectName('Elegy') == 'car'
assert getLabelFromObjectName('Ninef2') == 'car'
assert getLabelFromObjectName('Peyote') == 'car'
assert getLabelFromObjectName('Tornado5') == 'car'
assert getLabelFromObjectName('GP1') == 'car'
assert getLabelFromObjectName('Turismo2') == 'car'
assert getLabelFromObjectName('ArmyTrailer2') == 'UNRECOGNIZED_CATEGORY'
assert getLabelFromObjectName('Trailers') == 'UNRECOGNIZED_CATEGORY'
assert getLabelFromObjectName('Freight') == 'UNRECOGNIZED_CATEGORY'
assert getLabelFromObjectName('TankerCar') == 'UNRECOGNIZED_CATEGORY'
assert getLabelFromObjectName('Airtug') == 'UNRECOGNIZED_CATEGORY'
assert getLabelFromObjectName('Ripley') == 'truck'
assert getLabelFromObjectName('Bison2') == 'van'
assert getLabelFromObjectName('Camper') == 'van'














# parse the bbboxes from label_aug format to VisDrone format. This also catches some wrong class labels by looking at the models, e.g. SUVs as  truck
# TODO improve
# see file "Decision on the classes"
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

        object_name = data[21]

        ignore_this_bbox = False
        # Convert Label to VisDrone Label
        if label == 'Airplane':
            ignore_this_bbox = True
        if label == 'Animal':
            ignore_this_bbox = True
        if label == 'Boat':
            ignore_this_bbox = True
        if label == 'Bus':
            label = 'bus'
        
        if label == 'Car':
            label = 'car'
        
        if label == 'Cyclist':
            label = 'bicycle'
        if label == 'Motorbike':
            label = 'motor'
        if label == 'Pedestrian':
            label = 'pedestrian'
        if label == 'PersPerson_sitting':
            label = 'people'
        if label == 'Railed':
            ignore_this_bbox = True
        
        if label == 'Trailer':
            # Trailer Bounding Boxes on vehicles are wrong, but on standing trailers they are right
            # Also some of the standing Trailers are not labeled at all
            ignore_this_bbox = True

        if label == 'Truck':
            label = 'truck'
        if label == 'Utility':
            pass

        # This does not seem to exist
        if label == 'Van':
            label = 'van'

        # TODO Bus

        
        if label == 'UNRECOGNIZED_CATEGORY':
            raise ValueError('Encountered Unrecognized object_label in line: \n' + item)

        # ignore 1920 1080 0 0 boxes
        if not (left > right or top > bottom) and not ignore_this_bbox:
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



