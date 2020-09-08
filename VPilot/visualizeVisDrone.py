import time

import os

from shutil import copyfile
from shutil import rmtree

# Modification for testing and running with python 2 on the cluster
sysversion = sys.version_info[0]
# sysversion = 2

if sysversion >= 3:
    from PIL import Image
    import numpy as np
    import cv2

else:
    # As i can not import this on the cluster i have to use

    import StringIO
    import struct

    def getImageInfo(data):
        data = str(data)
        size = len(data)
        height = -1
        width = -1
        content_type = ''

        # handle GIFs
        if (size >= 10) and data[:6] in ('GIF87a', 'GIF89a'):
            # Check to see if content_type is correct
            content_type = 'image/gif'
            w, h = struct.unpack("<HH", data[6:10])
            width = int(w)
            height = int(h)

        # See PNG 2. Edition spec (http://www.w3.org/TR/PNG/)
        # Bytes 0-7 are below, 4-byte chunk length, then 'IHDR'
        # and finally the 4-byte width, height
        elif ((size >= 24) and data.startswith('\211PNG\r\n\032\n')
            and (data[12:16] == 'IHDR')):
            content_type = 'image/png'
            w, h = struct.unpack(">LL", data[16:24])
            width = int(w)
            height = int(h)

        # Maybe this is for an older PNG version.
        elif (size >= 16) and data.startswith('\211PNG\r\n\032\n'):
            # Check to see if we have the right content type
            content_type = 'image/png'
            w, h = struct.unpack(">LL", data[8:16])
            width = int(w)
            height = int(h)

        # handle JPEGs
        elif (size >= 2) and data.startswith('\377\330'):
            content_type = 'image/jpeg'
            jpeg = StringIO.StringIO(data)
            jpeg.read(2)
            b = jpeg.read(1)
            try:
                while (b and ord(b) != 0xDA):
                    while (ord(b) != 0xFF): b = jpeg.read(1)
                    while (ord(b) == 0xFF): b = jpeg.read(1)
                    if (ord(b) >= 0xC0 and ord(b) <= 0xC3):
                        jpeg.read(3)
                        h, w = struct.unpack(">HH", jpeg.read(4))
                        break
                    else:
                        jpeg.read(int(struct.unpack(">H", jpeg.read(2))[0])-2)
                    b = jpeg.read(1)
                width = int(w)
                height = int(h)
            except struct.error:
                pass
            except ValueError:
                pass

        return content_type, width, height


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

# OBJECT_CATEGORY_TO_DEEPGTAV_NUMBER = 


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

    image = image.convert('RGB')
    image = np.array(image)

    for bbox in bboxes:
        x1 = int(bbox['left'])
        y1 = int(bbox['top'])
        x2 = int(bbox['right'])
        y2 = int(bbox['bottom'])
        label = bbox['label']

        # if label == 'bicycle' or label == 'tricycle' or label == 'awning-tricycle' or label == 'motor' or label == 'people':
        if label == 'van':
            cv2.putText(image, label, (x1, y1+25), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), thickness = 2, lineType=cv2.LINE_AA) 
            cv2.rectangle(image, (x1, y1), (x2, y2), (255, 255, 0), 2)

    return image

def add_bboxes_relative(image, bboxes):
    width, height = image.size

    image = image.convert('RGB')
    image = np.array(image)

    for bbox in bboxes:
        x1 = bbox[1] - bbox[3] / 2
        y1 = bbox[2] - bbox[4] / 2
        x2 = bbox[1] + bbox[3] / 2
        y2 = bbox[2] + bbox[4] / 2 
        
        x1 = int(x1 * width)
        x2 = int(x2 * width)
        y1 = int(y1 * height)
        y2 = int(y2 * height)

        label = str(bbox[0])
        cv2.putText(image, label, (x1, y1+25), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), thickness = 2, lineType=cv2.LINE_AA) 
        cv2.rectangle(image, (x1, y1), (x2, y2), (255, 255, 0), 2)

    return image


def drawImageWithBBox(image, bboxes):
    img = add_bboxes(image, bboxes)
    cv2.imshow("test", img)
    cv2.waitKey(-1) 


def drawImageWithBBox_relative_format(image, bboxes):
    img = add_bboxes_relative(image, bboxes)
    cv2.imshow("test", img)
    cv2.waitKey(-1) 

class Image_Class:
    def __init__(self, width, height):
        self.size = (width, height)

def loadImageAndBBoxes(path):
    if sysversion >= 3:
        image = Image.open(path)
    else:
        with open(path, "rb") as file:
            content_type, width, height = getImageInfo(file.read())
        image = Image_Class(width, height)


    with open(path.replace("images", "annotations").replace("jpg", "txt"), 'r') as file:
        bboxes = file.read()
        bboxes = bboxes.split("\n")
        bboxes = [b.split(',') for b in bboxes]
        bboxes = [{'left': float(b[0]), 'top': float(b[1]), 'right': float(b[0]) + float(b[2]), 'bottom': float(b[1]) + float(b[3]), 'label': NUMBER_TO_OBJECT_CATEGORIES[int(b[5])] } for b in bboxes if b != ['']] #, 'label': NUMBER_TO_OBJECT_CATEGORIES[b[5]]} for b in bboxes]
    # return bboxes
    return image, bboxes


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


if sysversion >= 3:    
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
    assert np.isclose(bbox_test, convertBBoxesYolo_relative(convertBBoxYolo(revertConvertBBoxYolo(revertConvertBBoxesYolo_relative(bbox_test, 1920, 1080))), 1920, 1080)).all()


    bbox_test2 = [{'label': 'truck', 'left': 240, 'right': 260, 'top': 450, 'bottom': 550},
                {'label': 'truck', 'left': 440, 'right': 460, 'top': 550, 'bottom': 650},
                {'label': 'truck', 'left': 540, 'right': 560, 'top': 450, 'bottom': 550},
                {'label': 'truck', 'left': 240, 'right': 260, 'top': 150, 'bottom': 250}]
    assert bbox_test2 == revertConvertBBoxYolo(convertBBoxYolo(bbox_test2))

# # TODO change
# def build_example(idx, set_kind, TOP_DIRECTORY, addnoise_gaussian_SD=None, addnoise_uniform=None, addnoise_enlarge_absolute=None):
#     sample = auair.get_data_by_index(idx, ret_img=False)
#     file_name = sample['image_name']

#     img_path = os.path.join(data_folder_dir, file_name)

#     newdir = os.path.join('.', TOP_DIRECTORY, set_kind, 'images')
#     new_file_dir = os.path.join(newdir, file_name)
#     if not os.path.exists(newdir):
#         os.makedirs(newdir)
#     copyfile(img_path, new_file_dir)
    
#     with open(os.path.join('.', TOP_DIRECTORY, 'au-air-' + set_kind + '.txt'), 'a+') as setfile:
#         setfile.write(new_file_dir + "\n")

#     label_dir = os.path.join('.', TOP_DIRECTORY, set_kind , 'labels')
    
#     if not os.path.exists(label_dir):
#         os.makedirs(label_dir)
    
#     label_file = open(os.path.join(label_dir, file_name[:-4] + '.txt'), 'w+')
#     for bbox in sample['bbox']:

#         # Format for Yolo:
#         #   <object-class> <x_center> <y_center> <width> <height> 
#         #   object-class: integer in (0, classes-1)
#         #   others: float values relative to width and height of image, it can be equal from (0.0 to 1.0]
#         x = bbox['left']
#         y = bbox['top']
#         w = bbox['width']
#         h = bbox['height']

#         x,y,w,h = addnoise(x,y,w,h, addnoise_gaussian_SD, addnoise_uniform, addnoise_enlarge_absolute)

#         x_center = x + w / 2.0
#         y_center = y + h / 2.0

#         x_center = x_center / IMAGE_WIDTH
#         y_center = y_center / IMAGE_HEIGHT
#         w = w / IMAGE_WIDTH
#         h = h / IMAGE_HEIGHT


#         class_number = bbox['class']

#         label_text = "{:d} {:1.6f} {:1.6f} {:1.6f} {:1.6f}".format(class_number, x_center, y_center, w, h)

#         label_text = label_text + "\n"
#         label_file.write(label_text)
#     label_file.close()


# image, bboxes = loadImageAndBBoxes("E:/Bachelorarbeit/Datasets/VisDrone/VisDrone2019-DET-train/VisDrone2019-DET-train/images/0000197_02416_d_0000152.jpg")

# drawImageWithBBox(image, bboxes)



def main(_argv):
    train_dir = "./VisDrone2019-DET-train/images/"
    val_dir = "./VisDrone2019-DET-val/images/"
    savedir = "./VisDrone_YOLO_FORMAT"

    train_dir = os.path.normpath(train_dir)
    savedir = os.path.normpath(savedir)

    if sysversion >= 3:
        os.chdir("E:\Bachelorarbeit\Datasets\VisDrone")

    # if os.path.exists(savedir):
    #     rmtree(savedir)


    # if not os.path.exists(savedir):
    #     os.makedirs(savedir)
    # if not os.path.exists(os.path.join(savedir, "train")):
    #     os.makedirs(os.path.join(savedir, "train", "images"))
    #     os.makedirs(os.path.join(savedir, "train", "labels"))
    # if not os.path.exists(os.path.join(savedir, "val")):
    #     os.makedirs(os.path.join(savedir, "val", "images"))
    #     os.makedirs(os.path.join(savedir, "val", "labels"))
        


    print("Writing Training Data")
    for file in os.listdir(train_dir):
        img_dir = os.path.join(train_dir, file)
        # copyfile(img_dir, os.path.join(savedir, "train", "images", file))

         
        image, bboxes = loadImageAndBBoxes(img_dir)
    
        if [b for b in bboxes if b['label'] == 'van'] != []:
            drawImageWithBBox(image, bboxes)




        # label_texts = ["{:d} {:1.6f} {:1.6f} {:1.6f} {:1.6f}".format(*bbox) for bbox in bboxes]
        # label_texts = "\n".join(label_texts)



    #     with open(os.path.join(savedir, "train", "labels", file.replace("jpg", "txt")), "w") as file:
    #         file.write(label_texts)


    #     # write file location to index file
    
    # filepath = os.path.join(savedir, "train", "images")
    # filelist = [os.path.join(".", "VisDrone_YOLO_FORMAT", "train", "images", p) for p in os.listdir(filepath)]
    # filelist = "\n".join(filelist)

    # with open(os.path.join(savedir, "VisDrone-train.txt"), "w") as file:
    #     file.write(filelist)
        




    # print("Writing Val Data")
    # for file in os.listdir(val_dir):
    #     img_dir = os.path.join(val_dir, file)
    #     copyfile(img_dir, os.path.join(savedir, "val", "images", file))
    #     image, bboxes = loadImageAndBBoxes(img_dir)
    #     width, height = image.size
    #     bboxes = convertBBoxesYolo_relative(convertBBoxYolo(bboxes), img_width=width, img_height=height)

            

    #     label_texts = ["{:d} {:1.6f} {:1.6f} {:1.6f} {:1.6f}".format(*bbox) for bbox in bboxes]
    #     label_texts = "\n".join(label_texts)

    #     with open(os.path.join(savedir, "val", "labels", file.replace("jpg", "txt")), "w") as file:
    #         file.write(label_texts)

    #     # write file location to index file
    
    # filepath = os.path.join(savedir, "val", "images")
    # filelist = [os.path.join(".", "VisDrone_YOLO_FORMAT", "val", "images", p) for p in os.listdir(filepath)]
    # filelist = "\n".join(filelist)

    # with open(os.path.join(savedir, "VisDrone-val.txt"), "w") as file:
    #     file.write(filelist)


if __name__ == '__main__':
    main("")
