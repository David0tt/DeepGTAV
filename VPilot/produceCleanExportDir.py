from PIL import Image
import cv2
import os
import numpy as np

from shutil import copyfile
from shutil import rmtree



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
        # and ignore bboxes with area smaller than 25 px
        if not (left > right or top > bottom) and (right-left) * (bottom-top) > 100:
            ret.append({"label": label,"left": left,"top": top,"right": right,"bottom": bottom})

        # ignore small bboxes:
    return ret



if __name__ == '__main__':
    images_path = os.path.normpath("E:\\HiWi\\Record_Khalil\\object_RECORD_KHALIL2\\image_2")
    savedir = os.path.normpath("E:\\HiWi\\Record_Khalil\\Cleaned\\object_RECORD_KHALIL2")
    
    if os.path.exists(savedir):
        rmtree(savedir)

    os.makedirs(os.path.join(savedir, "image_2"))
    os.makedirs(os.path.join(savedir, "label_aug_2"))

    for file_name in os.listdir(images_path):
        # image = Image.open(os.path.join(images_path, file_name))
        with open(os.path.join(images_path.replace("image_2", "label_aug_2"), file_name.replace(".png", ".txt")), "r") as bbox_file:
            bbox2d_aug = bbox_file.read()
        
        bboxes = parseBBox2d(bbox2d_aug)

        if bboxes == []:
            pass
        else:
            copyfile(os.path.join(images_path, file_name), os.path.join(savedir, "image_2", file_name))
            copyfile(os.path.join(images_path.replace("image_2", "label_aug_2"), file_name.replace(".png", ".txt")), os.path.join(savedir, "label_aug_2", file_name.replace(".png", ".txt")))
        



        # image = np.array(image)

        # img = add_bboxes(image, parseBBox2d(bbox2d_aug))
        # cv2.imshow("test", img)
        # cv2.waitKey(-1)
         
