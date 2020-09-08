from PIL import Image
import cv2
import os
import numpy as np

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



if __name__ == '__main__':
    images_path = os.path.normpath("Z:\\DeepGTAV-EXPORTDIR-TEST\\images")
    for file_name in os.listdir(images_path):
        image = Image.open(os.path.join(images_path, file_name))
        with open(os.path.join(images_path.replace("images", "labels"), file_name.replace(".jpg", ".txt")), "r") as bbox_file:
            bbox2d_aug = bbox_file.read()
        
        image = np.array(image)
        bboxes = parseBBox2d(bbox2d_aug)
        img = add_bboxes(image, bboxes)

        # if [b for b in bboxes if b['label'] == 'van'] != []:
        cv2.imshow("test", img)
        cv2.waitKey(-1)
         
