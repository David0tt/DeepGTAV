#pragma once

#define PI 3.1415926535898
#define D2R PI/180.0

static const Eigen::Vector3f WORLD_NORTH(0.0, 1.0, 0.0);
static const Eigen::Vector3f WORLD_UP(0.0, 0.0, 1.0);
static const Eigen::Vector3f WORLD_EAST(1.0, 0.0, 0.0);

//Floats per point in the pointcloud (x, y, z, reflectance)
const int FLOATS_PER_POINT = 4;
const float MAX_LIDAR_DIST = 120.0f;//in metres
const int OBJECT_MAX_DIST = 200;//in metres (label_aug will have objects past this value)

const float CAM_OFFSET_FORWARD = 0;// .5;
const float CAM_OFFSET_UP = 1.065;
const float CAR_CENTER_OFFSET_UP = 0.665;//Distance to ground is ~1.73m (as per kitti specs for lidar)

const bool SET_PED_BOXES = true;
const float PED_BOX_WIDTH = 1.2f;
const float PED_BOX_LENGTH = 1.0f;
const float PED_BOX_WALKING_LEN = 1.44f;

const float BBOX_ADJUSTMENT_FACTOR = 1.1f;

//TODO: These should be moved to a settings file so don't need to rebuild when only changing settings

//Use the same time of day throughout the collection process
const bool SAME_TIME_OF_DAY = true;
//Drive in specified area or wander entire map
const bool DRIVE_SPEC_AREA = true;
const bool START_SPEC_AREA = true;

//TODO -> stencil/depth semantic segmentation for depthmap->lidar interpolation

//Some settings for testing pointcloud generation
//This prints the 2D map of where lidar beams hit on the screen (not consistent when moving)
const bool GENERATE_2D_POINTMAP = false;
//Outputs secondary pointcloud with raycast points
const bool OUTPUT_RAYCAST_POINTS = false;

//Outputs pointcloud with 1:1 ratio of pixels in image < MAX_LIDAR_DIST
const bool OUTPUT_DM_POINTCLOUD = false;
//If OUTPUT_DM_POINTCLOUD, outputs all points, even those past MAX_LIDAR_DIST (good for testing)
const bool OUTPUT_FULL_DM_POINTCLOUD = false;

//Sends animals to client. Check client also outputs them
const bool RETAIN_ANIMALS = true;

//For testing
const bool OUTPUT_OCCLUSION_IMAGE = false;
const bool OUTPUT_UNUSED_PIXELS_IMAGE = false;

//Can be used to get general outline of some objects with raycasting
//WARNING: NOT ALL VEHICLES ARE HIT WITH RAYCASTING
const bool OBTAIN_RAY_POINTS_HIT = false;
//Overlays adjusted points on top of pointcloud (using raycasting + depth map)
const bool OUTPUT_ADJUSTED_POINTS = false;

//2d points will be shrunk with stencil cull
const bool CORRECT_2D_POINTS_BEHIND_CAMERA = false;

const bool LIDAR_GAUSSIAN_NOISE = true;
const double DEPTH_NOISE_STDDEV = 0.006;//3 standard deviations is approximately 2cm
const double DEPTH_NOISE_MEAN = 0.0;

//Extends bboxes/segmentation for bike type vehicles with rider (ped) information
const bool PROCESS_PEDS_ON_BIKES = true;
const bool TESTING_PEDS_ON_BIKES = true;

//Outputs stats on lidar vs depth map conversion (used for testing)
const bool OUTPUT_DEPTH_STATS = false;

//Maximum distance from ground to be considered ground point:
const float GROUND_POINT_MAX_DIST = 0.1;//in metres
//Prints out image in groundPointsImg for testing
const bool OUTPUT_GROUND_PIXELS = false;

//Outputs separate stencil segmentation images for each stencil value
const bool OUTPUT_SEPARATE_STENCILS = true;

//Needs OUTPUT_SEPARATE_STENCILS to be true, only outputs log messages for values which are unknown 
const bool ONLY_OUTPUT_UNKNOWN_STENCILS = true;

//If set to true DeepGTAV outputs all information from nearby vehicles as well
const bool GENERATE_SECONDARY_PERSPECTIVES = false;

//Outputs self location (For finding spots for stationary scenes)
const bool OUTPUT_SELF_LOCATION = false;