#Helper functions for exporting data
def getText(classID, distance):
    if distance > 80: #In metres
        return "DontCare"
    return {
        0: "Car",
        1: "Motorcycle",
        2: "Cyclist",
        3: "QuadBike",
        7: "Train",
        10: "Pedestrian"
    }.get(classID,"DontCare")

test2DBoxes = False
repeats = 1
if test2DBoxes:
    repeats = 3

def outputObjectInfo(text_file, instance, bboxType = 1):
    #TODO Determine actual class of vehicle
    if test2DBoxes:
        text = "Train"
        if bboxType == 1:
            text = "Car"
        if bboxType == 2:
            text = "DontCare"
    else:
        text = getText(instance['classID'], instance['distance'])
    text_file.write("%s" % text)

    #TODO - Determine if truncated
    text_file.write(" 0.00")

    #TODO - Occlusion
    text_file.write(" 0")
    
    #Observation angle alpha TODO
    text_file.write(" %f" % instance['alpha'])

    #2D Bounding box TODO
    bboxNums = instance['bbox2d']
    if test2DBoxes:
        if bboxType == 2:
            bboxNums = instance['bbox2dEigen']
        if bboxType == 3:
            bboxNums = instance['bbox2dGame']

    for num in bboxNums:
        text_file.write(" %d" % num)
    
    #3D dimensions
    for num in instance['dimensions']:
        text_file.write(" %f" % num)
    
    #3D location
    for num in instance['location']:
        text_file.write(" %f" % num)
    
    #Rotation_y
    text_file.write(" %f" % instance['rotation_y'])
    
def printInstances(filename, list, augment, tracking=False):
    text_file = open(filename, "a")
    for instance in list:
        #Only print animals (classId 11) for augmented albels
        if instance['classID'] != 11 or augment:
            for bboxType in range(3,repeats + 1):
                if tracking:
                    text_file.write("%d" % instance['trackFirstFrame'])
                    text_file.write(" %d " % instance['entityID'])
                    outputObjectInfo(text_file, instance)
                else:
                    outputObjectInfo(text_file, instance, bboxType)

                    if augment:
                        text_file.write(" Augmentations:")
                        text_file.write(" %d" % instance['entityID'])
                        text_file.write(" %d" % instance['pointsHit'])
                        text_file.write(" %f" % instance['speed'])
                        text_file.write(" %f" % instance['heading'])
                        text_file.write(" %d" % instance['classID'])
                        
                        if instance['offscreen']:
                            text_file.write(" 0")
                        else:
                            text_file.write(" 1")

                        #3D dimensions offcenter
                        for num in instance['offcenter']:
                            text_file.write(" %f" % num)
                    
                        # for num in instance['FUR']:
                        #     text_file.write(" %f" % num)
                        # for num in instance['BLL']:
                        #     text_file.write(" %f" % num)

                text_file.write("\n")
    text_file.close()

def printCalib(filename, focalLen, width, height):
    text_file = open(filename, "w")
    for i in range(0,4):
        text_file.write("P%d:" % i)
        text_file.write(" %f 0 %d 0" % (focalLen, width/2))
        text_file.write(" 0 %f %d 0" % (focalLen, height/2))
        text_file.write(" 0 0 1 0\n")

    text_file.write("R0_rect: 1 0 0 0 1 0 0 0 1\n")
    #From KITTI velo to KITTI cam
    text_file.write("Tr_velo_to_cam: 0 -1 0 0 0 0 -1 0 1 0 0 0\n")
    text_file.write("Tr_imu_to_velo: 1 0 0 0 0 1 0 0 0 0 1 0")

    # From GTA to KITTI
    # text_file.write("Tr_velo_to_cam: 1 0 0 0 0 0 -1 0 0 1 0 0")
    text_file.close()