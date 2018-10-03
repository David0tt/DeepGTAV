#pragma once

#define PI 3.1415926535898
#define D2R PI/180.0

//Floats per point in the pointcloud (x, y, z, reflectance)
const int FLOATS_PER_POINT = 4;
const float MAX_LIDAR_DIST = 120.0f;//in metres

const bool SET_PED_BOXES = true;
const float PED_BOX_WIDTH = 1.2f;
const float PED_BOX_LENGTH = 0.5f;
const float PED_BOX_WALKING_LEN = 1.44f;

//Use the same time of day throughout the collection process
const bool SAME_TIME_OF_DAY = true;
//Drive in specified area or wander entire map
const bool DRIVE_SPEC_AREA = true;

//TODO -> stencil/depth semantic segmentation for depthmap->lidar interpolation

//Some settings for testing pointcloud generation
//This prints the 2D map of where lidar beams hit on the screen (not consistent when moving)
const bool GENERATE_2D_POINTMAP = false;
//Outputs secondary pointcloud with raycast points
const bool OUTPUT_RAYCAST_POINTS = true;
const bool OUTPUT_DM_POINTCLOUD = true;

//Sends animals to client. Check client also outputs them
const bool RETAIN_ANIMALS = true;

//For testing
const bool OUTPUT_OCCLUSION_IMAGE = true;
const bool OUTPUT_UNUSED_PIXELS_IMAGE = true;

//Can be used to get general outline of some objects with raycasting
//WARNING: NOT ALL VEHICLES ARE HIT WITH RAYCASTING
const bool OBTAIN_RAY_POINTS_HIT = false;
//Overlays adjusted points on top of pointcloud (using raycasting + depth map)
const bool OUTPUT_ADJUSTED_POINTS = false;

//2d points will be shrunk with stencil cull
const bool CORRECT_2D_POINTS_BEHIND_CAMERA = false;