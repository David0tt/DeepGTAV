from PIL import Image
from random import uniform
import cv2
import os






# saves image in np format and bboxes in string format
def save_image_and_bbox(save_dir, filename, image, bboxes):
    # convert image BGR -> RGB
    # image = image[...,::-1]
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    img = Image.fromarray(image)
    img.save(os.path.join(save_dir, 'images', filename + ".jpg"))

    with open(os.path.join(save_dir, 'labels', filename + ".txt"), 'w') as file:
        file.write(bboxes)



# returns the highest run in the directory + 1
def getRunCount(save_dir):
    files = os.listdir(os.path.join(save_dir, 'images'))
    files = [int(f[:4]) for f in files]
    if files == []:
        return 0
    else:
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
