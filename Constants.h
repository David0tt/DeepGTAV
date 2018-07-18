#pragma once

#define PI 3.1415926535898
#define D2R PI/180.0

//Floats per point in the pointcloud (x, y, z, reflectance)
const int FLOATS_PER_POINT = 4;
const float MAX_LIDAR_DIST = 120.0f;//in metres

//Some settings for testing pointcloud generation
const bool GENERATE_2D_POINTMAP = false;
const bool OUTPUT_ADJUSTED_POINTS = false;
const bool OUTPUT_RAYCAST_POINTS = true;

const bool RETAIN_ANIMALS = false;