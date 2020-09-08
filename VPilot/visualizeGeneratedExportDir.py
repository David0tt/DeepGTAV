from PIL import Image
import cv2
import os
import numpy as np

from utils.BoundingBoxes import add_bboxes, parseBBoxLabel_augToVisDrone, parseBBox2d


if __name__ == '__main__':
    images_path = os.path.normpath("C:\\EXPORTDIR\\Record_Khalil\\object_RECORD_KHALIL2\\image_2")
    for file_name in os.listdir(images_path):
        image = Image.open(os.path.join(images_path, file_name))
        with open(os.path.join(images_path.replace("image_2", "label_aug_2"), file_name.replace(".png", ".txt")), "r") as bbox_file:
            bbox2d_aug = bbox_file.read()
        
        image = np.array(image)
        bboxes = parseBBoxLabel_augToVisDrone(bbox2d_aug)
        img = add_bboxes(image, bboxes)

        print("file_name: ", file_name)
        print(bbox2d_aug)

        # if [b for b in bboxes if b['label'] == 'van'] != []:
        cv2.imshow("test", img)
        cv2.waitKey(-1)




def show_specified_label(label):
    images_path = os.path.normpath("C:\\EXPORTDIR\\Record_Khalil\\object_RECORD_KHALIL2\\image_2")
    for file_name in os.listdir(images_path):
        with open(os.path.join(images_path.replace("image_2", "label_aug_2"), file_name.replace(".png", ".txt")), "r") as bbox_file:
            bbox2d_aug = bbox_file.read()
        bboxes = parseBBox2d(bbox2d_aug)


        if [b for b in bboxes if b['label'] == label] != []:
            image = Image.open(os.path.join(images_path, file_name))
            image = np.array(image)
            img = add_bboxes(image, bboxes)

            print("file_name: ", file_name)
            print(bbox2d_aug)

            cv2.imshow("test", img)
            cv2.waitKey(-1)




def produce_all_classnames():
    all_class_names = set()

    images_path = os.path.normpath("C:\\EXPORTDIR\\Record_Khalil\\object_RECORD_KHALIL2\\image_2")

    for file_name in os.listdir(images_path):
        with open(os.path.join(images_path.replace("image_2", "label_aug_2"), file_name.replace(".png", ".txt")), "r") as bbox_file:
            bbox2d_aug = bbox_file.read()
        
        bboxes = parseBBox2d(bbox2d_aug)
        for bbox in bboxes:
            all_class_names.add(bbox["label"])

    images_path = os.path.normpath("C:\\EXPORTDIR\\Record_Khalil\\object_RECORD_KHALIL1\\image_2")

    for file_name in os.listdir(images_path):
        with open(os.path.join(images_path.replace("image_2", "label_aug_2"), file_name.replace(".png", ".txt")), "r") as bbox_file:
            bbox2d_aug = bbox_file.read()
        
        bboxes = parseBBox2d(bbox2d_aug)
        for bbox in bboxes:
            all_class_names.add(bbox["label"])

    return all_class_names

# names = produce_all_classnames()