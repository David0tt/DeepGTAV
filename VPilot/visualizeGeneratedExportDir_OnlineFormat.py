from PIL import Image
import cv2
import os
import numpy as np

from utils.BoundingBoxes import parseBBox_YoloFormat_to_Image, show_image_with_bboxes
    
def show_dataset_online_format(images_path):
    images_path = os.path.normpath(images_path)
    for file_name in os.listdir(images_path):
        image = Image.open(os.path.join(images_path, file_name))
        with open(os.path.join(images_path.replace("images", "labels"), file_name.replace(".jpg", ".txt")), "r") as bbox_file:
            bboxes = bbox_file.read()
        
        image = np.array(image)
        bboxes = parseBBox_YoloFormat_to_Image(bboxes)
        show_image_with_bboxes(image, bboxes)

show_dataset_online_format("Z:\\DeepGTAV-EXPORTDIR-TEST\\images")


if __name__ == '__main__':
    pass