import numpy as np
import open3d


# filename = "E:/Bachelorarbeit/Datasets/Dataset Pregenerated DeepGTAV-PreSIL/velodyne/000002.bin"
filename = "C:/EXPORTDIR/object/velodyneRaycast/000000.bin"

with open(filename, "r") as file:
    a = np.fromfile(file, dtype = np.float32)

a = a.reshape((-1, 4))
points3d = np.delete(a, 3, 1)

point_cloud = open3d.geometry.PointCloud()
point_cloud.points = open3d.utility.Vector3dVector(points3d)

open3d.visualization.draw_geometries([point_cloud])

