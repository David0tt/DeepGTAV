#pragma once

#define PI 3.1415926535898
#define D2R PI/180.0

//Floats per point in the pointcloud (x, y, z, reflectance)
const int FLOATS_PER_POINT = 4;
const float MAX_LIDAR_DIST = 120.0f;//in metres

//Some settings for testing pointcloud generation
//This prints the 2D map of where lidar beams hit on the screen (not consistent when moving)
const bool GENERATE_2D_POINTMAP = false;
//Overlays adjusted points on top of pointcloud (using raycasting + depth map)
const bool OUTPUT_ADJUSTED_POINTS = false;
//Outputs secondary pointcloud with raycast points
const bool OUTPUT_RAYCAST_POINTS = true;

//Sends animals to client. Client only outputs them in augmented labels at the moment
const bool RETAIN_ANIMALS = true;