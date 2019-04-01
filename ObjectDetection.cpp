#define NOMINMAX

#include "ObjectDetection.h"
#include "..\ObjectDetIncludes.h"
#include <Windows.h>
#include <time.h>
#include <fstream>
#include <string>
#include <sstream>
#include "Functions.h"
#include "Constants.h"
#include <Eigen/Core>
#include <sstream>
#include "lodepng.h"
#include <opencv2\opencv.hpp>

ObjectDetection::ObjectDetection()
{
}

ObjectDetection::~ObjectDetection()
{
}

//Global variable for storing camera parameters
CamParams s_camParams;

const float VERT_CAM_FOV = 59; //In degrees
                               //Need to input the vertical FOV with GTA functions.
                               //90 degrees horizontal (KITTI) corresponds to 59 degrees vertical (https://www.gtaall.com/info/fov-calculator.html).
const float HOR_CAM_FOV = 90; //In degrees

//Known stencil types
const int STENCIL_TYPE_DEFAULT = 0;//Ground, buildings, etc...
const int STENCIL_TYPE_NPC = 1;
const int STENCIL_TYPE_VEHICLE = 2;
const int STENCIL_TYPE_VEGETATION = 3;
const int STENCIL_TYPE_FLOOR = 4;//Seems to be floors and some boulevards
const int STENCIL_TYPE_SKY = 7;
const int STENCIL_TYPE_SELF = 129;
const int STENCIL_TYPE_OWNCAR = 130;
const std::vector<int> KNOWN_STENCIL_TYPES = { STENCIL_TYPE_DEFAULT, STENCIL_TYPE_NPC, STENCIL_TYPE_VEHICLE, STENCIL_TYPE_VEGETATION, STENCIL_TYPE_FLOOR, STENCIL_TYPE_SKY, STENCIL_TYPE_SELF, STENCIL_TYPE_OWNCAR };

const int PEDESTRIAN_CLASS_ID = 10;

void ObjectDetection::initCollection(UINT camWidth, UINT camHeight, bool exportEVE, int startIndex) {
    if (m_initialized) {
        return;
    }
    m_eve = exportEVE;
    instance_index = startIndex;

    ped = PLAYER::PLAYER_PED_ID();
    m_ownVehicle = PED::GET_VEHICLE_PED_IS_IN(ped, false);
    m_vehicle = m_ownVehicle;
    setOwnVehicleObject();

    char temp[] = "%06d";
    char strComp[sizeof temp + 100];
    sprintf(strComp, temp, instance_index);
    instance_string = strComp;
    s_camParams.width = (int)camWidth;
    s_camParams.height = (int)camHeight;
    //LOG(LL_ERR, "Testing4");

    //Need to set camera params
    s_camParams.init = false;
    camera = CAM::GET_RENDERING_CAM();
    //Create camera intrinsics matrix
    calcCameraIntrinsics();
    pointclouds = true;
    collectTracking = false;

    //Export directory
    log("Before getting export dir");
    baseFolder = std::string(getenv("DEEPGTAV_EXPORT_DIR")) + "\\";
    CreateDirectory(baseFolder.c_str(), NULL);
    if (exportEVE) {
        baseFolder += "eve\\";
    }
    if (collectTracking) {
        baseFolder += "tracking\\";
    }
    else {
        baseFolder += "object\\";
    }
    log("After getting export dir");
    CreateDirectory(baseFolder.c_str(), NULL);
    m_timeTrackFile = baseFolder + "\\TimeAnalysis.txt";
    m_usedPixelFile = baseFolder + "\\UsedPixels.txt";
    log("After getting export dir2");

    //Overwrite previous time analysis file so it is empty
    FILE* f = fopen(m_timeTrackFile.c_str(), "w");
    std::ostringstream oss;
    oss << "Mean err, var, avg speed, avg dist";
    oss << "\nResults are in metres. Frames attempted to capture at 10 Hz.";
    std::string str = oss.str();
    fprintf(f, str.c_str());
    fprintf(f, "\n");
    fclose(f);

    //Overwrite previous stencil pixel used file so it is empty
    f = fopen(m_usedPixelFile.c_str(), "w");
    std::ostringstream oss1;
    oss1 << "Unused pixels, index, series (if tracking)";
    std::string str1 = oss1.str();
    fprintf(f, str1.c_str());
    fprintf(f, "\n");
    fclose(f);
    log("Before initVehicleLookup");

    initVehicleLookup();
    //Setup LiDAR before collecting
    setupLiDAR();
    m_initialized = true;
}

//Set own object info for exporting position_world
void ObjectDetection::setOwnVehicleObject() {
    Vector3 min, max;
    Hash model = ENTITY::GET_ENTITY_MODEL(m_ownVehicle);
    Vector3 dim = getVehicleDims(m_ownVehicle, model, min, max);
    m_ownVehicleObj = ObjEntity(m_ownVehicle);

    //Fill out info for own car
    m_ownVehicleObj.objType = "Car";

    float kittiHeight = 2 * dim.z;
    float kittiWidth = 2 * dim.x;
    float kittiLength = 2 * dim.y;
    m_ownVehicleObj.width = kittiWidth;
    m_ownVehicleObj.height = kittiHeight;
    m_ownVehicleObj.length = kittiLength;

    Vector3 position;
    position.x = 0;
    position.y = 0;
    position.z = 0;
    m_ownVehicleObj.location = position;
    m_ownVehicleObj.rotation_y = -1;
    m_ownVehicleObj.alpha = -1;

    BBox2D bbox2d;
    bbox2d.bottom = -1;
    bbox2d.top = -1;
    bbox2d.left = -1;
    bbox2d.right = -1;
    m_ownVehicleObj.bbox2d = bbox2d;

    m_ownVehicleObj.truncation = -1;
    m_ownVehicleObj.occlusion = -1;
    m_ownVehicleObj.modelString = VEHICLE::GET_DISPLAY_NAME_FROM_VEHICLE_MODEL(model);

    m_ownVehicleObj.speed = -1;
    m_ownVehicleObj.roll = -1;
    m_ownVehicleObj.pitch = -1;

    m_ownVehicleObj.pointsHit2D = -1;
    m_ownVehicleObj.pointsHit3D = -1;
}

//For updating all depth/stencil related variables when depth/stencil buffer are one frame after game functions
FrameObjectInfo ObjectDetection::setDepthAndStencil(bool prevDepth, float* pDepth, uint8_t* pStencil) {
    if (prevDepth) {
        m_pDepth = pDepth;
        m_pStencil = pStencil;
    }
    else {
        setFilenames();
    }

    if (lidar_initialized) setDepthBuffer(prevDepth);
    if (lidar_initialized) setStencilBuffer();

    if (prevDepth) {
        if (lidar_initialized) printSegImage();
        if (lidar_initialized) outputOcclusion();
        if (lidar_initialized) outputUnusedStencilPixels();
    }

    return m_curFrame;
}

FrameObjectInfo ObjectDetection::generateMessage(float* pDepth, uint8_t* pStencil, int entityID) {
    //LOG(LL_ERR, "Depth data generate: ", pDepth[0], pDepth[1], pDepth[2], pDepth[3], pDepth[4], pDepth[5], pDepth[6], pDepth[7]);
    m_pDepth = pDepth;
    m_pStencil = pStencil;
    m_vPerspective = entityID;
    if (entityID != -1) {
        m_vehicle = entityID;
    }
    else {
        m_vehicle = m_ownVehicle;
    }

    //TODO pass this through
    bool depthMap = true;

    setIndex();
    setPosition();
    outputRealSpeed();
    setDepthAndStencil();
    //Need to set peds list first for integrating peds on bikes
    setPedsList();
    setVehiclesList();
    //setDirection();
    //setSteering();
    setSpeed();
    setYawRate();
    setTime();
    setFocalLength();
    log("After focalLength");
    //if (depthMap && lidar_initialized) outputGroundSeg();
    //if (depthMap && lidar_initialized) updateSegImage();

    if (lidar_initialized) getContours();
    //TODO: Need to update 2D bboxes after receiving new depth image
    processSegmentation();
    processOcclusion();

    if (pointclouds && lidar_initialized) collectLiDAR();
    update3DPointsHit();

    if (depthMap && lidar_initialized) printSegImage();
    log("After printSeg");
    if (depthMap && lidar_initialized) outputOcclusion();
    log("After output occlusion");
    if (depthMap && lidar_initialized) outputUnusedStencilPixels();
    log("After output unused stencil");

    setGroundPlanePoints();
    getNearbyVehicles();

    return m_curFrame;
}

//Returns the angle between a relative position vector and the forward vector (rotated about up axis)
float ObjectDetection::observationAngle(Vector3 position) {
    float x1 = m_camRightVector.x;
    float y1 = m_camRightVector.y;
    float z1 = m_camRightVector.z;
    float x2 = position.x;
    float y2 = position.y;
    float z2 = position.z;
    float xn = m_camUpVector.x;
    float yn = m_camUpVector.y;
    float zn = m_camUpVector.z;

    float dot = x1 * x2 + y1 * y2 + z1 * z2;
    float det = x1 * y2*zn + x2 * yn*z1 + xn * y1*z2 - z1 * y2*xn - z2 * yn*x1 - zn * y1*x2;
    float observationAngle = atan2(det, dot);

    if (DEBUG_LOGGING) {
        std::ostringstream oss;
        oss << "Forward is: " << x1 << ", " << y1 << ", " << z1 <<
            "\nNormal is: " << x2 << ", " << y2 << ", " << z2 <<
            "\nPosition is: " << position.x << ", " << position.y << ", " << position.z << " and angle is: " << observationAngle;
        std::string str = oss.str();
        log(str);
    }

    return observationAngle;
}

void ObjectDetection::drawVectorFromPosition(Vector3 vector, int blue, int green) {
    GRAPHICS::DRAW_LINE(s_camParams.pos.x, s_camParams.pos.y, s_camParams.pos.z, vector.x * 1000 + s_camParams.pos.x, vector.y * 1000 + s_camParams.pos.y, vector.z * 1000 + s_camParams.pos.z, 0, green, blue, 200);
    WAIT(0);
}

//Saves the position and vectors of the capture vehicle
void ObjectDetection::setPosition() {
    //NOTE: The forward and right vectors are swapped (compared to native function labels) to keep consistency with coordinate system
    if (m_eve) {
        ENTITY::GET_ENTITY_MATRIX(m_vehicle, &vehicleForwardVector, &vehicleRightVector, &vehicleUpVector, &currentPos); //Blue or red pill

        /*LOG(LL_ERR, "Eve Forward vector: ", m_camForwardVector.x, " Y: ", m_camForwardVector.y, " Z: ", m_camForwardVector.z);
        LOG(LL_ERR, "Forward vector: ", vehicleForwardVector.x, " Y: ", vehicleForwardVector.y, " Z: ", vehicleForwardVector.z);
        LOG(LL_ERR, "Right vector: ", vehicleRightVector.x, " Y: ", vehicleRightVector.y, " Z: ", vehicleRightVector.z);
        LOG(LL_ERR, "Up vector: ", vehicleUpVector.x, " Y: ", vehicleUpVector.y, " Z: ", vehicleUpVector.z);
        LOG(LL_ERR, "Cam Forward vector: ", m_camForwardVector.x, " Y: ", m_camForwardVector.y, " Z: ", m_camForwardVector.z);
        LOG(LL_ERR, "Cam Right vector: ", m_camRightVector.x, " Y: ", m_camRightVector.y, " Z: ", m_camRightVector.z);
        LOG(LL_ERR, "Cam Up vector: ", m_camUpVector.x, " Y: ", m_camUpVector.y, " Z: ", m_camUpVector.z);
        LOG(LL_ERR, "Curr position: ", s_camParams.pos.x, " Y: ", s_camParams.pos.y, " Z: ", s_camParams.pos.z);
        LOG(LL_ERR, "Theta: ", s_camParams.theta.x, " Y: ", s_camParams.theta.y, " Z: ", s_camParams.theta.z);*/

        float ogThetaZ = tan(-vehicleForwardVector.x / vehicleForwardVector.y) * 180 / PI;
        float newThetaZ = tan(-m_camForwardVector.x / m_camForwardVector.y) * 180 / PI;
        float ogThetaZ2 = atan2(-vehicleForwardVector.x, vehicleForwardVector.y) * 180 / PI;
        float newThetaZ2 = atan2(-m_camForwardVector.x, m_camForwardVector.y) * 180 / PI;
        float ogThetaX2 = atan2(vehicleForwardVector.z, sqrt(pow(vehicleForwardVector.y, 2) + pow(vehicleForwardVector.x, 2))) * 180 / PI;
        float newThetaX2 = atan2(m_camForwardVector.z, sqrt(pow(m_camForwardVector.y, 2) + pow(m_camForwardVector.x, 2))) * 180 / PI;
        float ogThetaX = tan(vehicleForwardVector.z / sqrt(pow(vehicleForwardVector.y, 2) + pow(vehicleForwardVector.x, 2))) * 180 / PI;
        float newThetaX = tan(m_camForwardVector.z / sqrt(pow(m_camForwardVector.y, 2) + pow(m_camForwardVector.x, 2))) * 180 / PI;
        //LOG(LL_ERR, "Theta og/new Z: ", ogThetaZ, ", ", newThetaZ, " og/new Z2: ", ogThetaZ2, ", ", newThetaZ2, " og, new X: ", ogThetaX, ", ", newThetaX, " og, new ThetaX2: ", ogThetaX2, ", ", newThetaX2);

        float ogThetaY2 = atan2(vehicleRightVector.z, sqrt(pow(vehicleRightVector.y, 2) + pow(vehicleRightVector.x, 2))) * 180 / PI;
        float newThetaY2 = atan2(-m_camRightVector.z, sqrt(pow(m_camRightVector.y, 2) + pow(m_camRightVector.x, 2))) * 180 / PI;
        //LOG(LL_ERR, "Theta og/new Y2: ", ogThetaY2, ", ", newThetaY2);
        s_camParams.theta.x = newThetaX2;
        s_camParams.theta.y = newThetaY2;
        s_camParams.theta.z = newThetaZ2;
        //LOG(LL_ERR, "Theta: ", s_camParams.theta.x, " Y: ", s_camParams.theta.y, " Z: ", s_camParams.theta.z);
    }
    else {
        //If not eve, the camera and vehicle are aligned by pausing and flushing the buffers
        ENTITY::GET_ENTITY_MATRIX(m_vehicle, &m_camForwardVector, &m_camRightVector, &m_camUpVector, &currentPos);
        ENTITY::GET_ENTITY_MATRIX(m_vehicle, &vehicleForwardVector, &vehicleRightVector, &vehicleUpVector, &currentPos); //Blue or red pill
    }

    m_curFrame.position = currentPos;

    m_curFrame.roll = atan2(-vehicleRightVector.z, sqrt(pow(vehicleRightVector.y, 2) + pow(vehicleRightVector.x, 2)));
    m_curFrame.pitch = atan2(-vehicleForwardVector.z, sqrt(pow(vehicleForwardVector.y, 2) + pow(vehicleForwardVector.x, 2)));

    //Should use atan2 over gameplay heading
    //float heading = GAMEPLAY::GET_HEADING_FROM_VECTOR_2D(vehicleForwardVector.x, vehicleForwardVector.y);
    m_curFrame.heading = atan2(vehicleForwardVector.y, vehicleForwardVector.x);

    m_curFrame.forwardVec = vehicleForwardVector;
    m_curFrame.upVec = vehicleUpVector;
    m_curFrame.rightVec = vehicleRightVector;
    m_curFrame.camPos = s_camParams.pos;

    //Check if we see it (not occluded)
    Vector3 min, max, offcenter;
    Hash model = ENTITY::GET_ENTITY_MODEL(m_vehicle);
    GAMEPLAY::GET_MODEL_DIMENSIONS(model, &min, &max);
    Vector3 kittiWorldPos = correctOffcenter(currentPos, min, max, vehicleForwardVector, vehicleRightVector, vehicleUpVector, offcenter);
    m_curFrame.kittiWorldPos = kittiWorldPos;
}

void ObjectDetection::setSpeed() {
    m_curFrame.speed = ENTITY::GET_ENTITY_SPEED(m_vehicle);
}

void ObjectDetection::setYawRate() {
    Vector3 rates = ENTITY::GET_ENTITY_ROTATION_VELOCITY(m_vehicle);
    m_curFrame.yawRate = rates.z*180.0 / 3.14159265359;
}

void ObjectDetection::setTime() {
    m_curFrame.timeHours = TIME::GET_CLOCK_HOURS();
}

//Cycle through 8 corners of bbox and see if the ray makes it to or past this point
bool ObjectDetection::hasLOSToEntity(Entity entityID, Vector3 position, Vector3 dim, Vector3 forwardVector, Vector3 rightVector, Vector3 upVector, bool useOrigin, Vector3 origin) {
    Vector3 oPos;
    if (useOrigin) {
        oPos = origin;
    }
    else {
        oPos = s_camParams.pos;
    }

    for (int right = -1; right <= 1; right += 2) {
        for (int forward = -1; forward <= 1; forward += 2) {
            for (int up = -1; up <= 1; up += 2) {
                Vector3 pos;
                pos.x = position.x + forward * dim.y*forwardVector.x + right * dim.x*rightVector.x + up * dim.z*upVector.x;
                pos.y = position.y + forward * dim.y*forwardVector.y + right * dim.x*rightVector.y + up * dim.z*upVector.y;
                pos.z = position.z + forward * dim.y*forwardVector.z + right * dim.x*rightVector.z + up * dim.z*upVector.z;

                Vector3 relPos;
                relPos.x = pos.x - oPos.x;
                relPos.y = pos.y - oPos.y;
                relPos.z = pos.z - oPos.z;

                BOOL isHit;
                Entity hitEntity;
                Vector3 target, endCoord, surfaceNorm;
                target.x = relPos.x * 200 + pos.x;
                target.y = relPos.y * 200 + pos.y;
                target.z = relPos.z * 200 + pos.z;

                //options: -1=everything
                //New function is called _START_SHAPE_TEST_RAY
                int raycast_handle = WORLDPROBE::_CAST_RAY_POINT_TO_POINT(oPos.x, oPos.y, oPos.z, target.x, target.y, target.z, -1, m_vehicle, 7);

                //New function is called GET_SHAPE_TEST_RESULT
                WORLDPROBE::_GET_RAYCAST_RESULT(raycast_handle, &isHit, &endCoord, &surfaceNorm, &hitEntity);

                float distance = sqrt(SYSTEM::VDIST2(oPos.x, oPos.y, oPos.z, pos.x, pos.y, pos.z));
                float rayDistance = sqrt(SYSTEM::VDIST2(oPos.x, oPos.y, oPos.z, endCoord.x, endCoord.y, endCoord.z));

                if (!isHit || rayDistance > distance) {
                    return true;
                }
            }
        }
    }
    return false;
}

BBox2D ObjectDetection::BBox2DFrom3DObject(Vector3 position, Vector3 dim, Vector3 forwardVector, Vector3 rightVector, Vector3 upVector, bool &success, float &truncation) {
    //Adjust position back to middle of the object for calculating 2D bounding box (Kitti has it at bottom)
    position.z += dim.z;

    BBox2D bbox2d;
    bbox2d.left = 1.0;// width;
    bbox2d.right = 0.0;
    bbox2d.top = 1.0;// height;
    bbox2d.bottom = 0.0;

    for (int right = -1; right <= 1; right += 2) {
        for (int forward = -1; forward <= 1; forward += 2) {
            for (int up = -1; up <= 1; up += 2) {
                Vector3 pos;
                pos.x = position.x + forward * dim.y*forwardVector.x + right * dim.x*rightVector.x + up * dim.z*upVector.x;
                pos.y = position.y + forward * dim.y*forwardVector.y + right * dim.x*rightVector.y + up * dim.z*upVector.y;
                pos.z = position.z + forward * dim.y*forwardVector.z + right * dim.x*rightVector.z + up * dim.z*upVector.z;

                float screenX, screenY;
                //This function always returns false, do not worry about return value
                bool success = GRAPHICS::_WORLD3D_TO_SCREEN2D(pos.x, pos.y, pos.z, &screenX, &screenY);

                std::ostringstream oss2;
                oss2 << "\nnew ScreenX: " << screenX << " ScreenY: " << screenY;
                std::string str2 = oss2.str();
                log(str2);

                //Calculate with eigen if off-screen
                Eigen::Vector3f pt(pos.x, pos.y, pos.z);
                if (screenX < 0 || screenX > 1 || screenY < 0 || screenY > 1) {
                    Eigen::Vector2f uv = get_2d_from_3d(pt,
                        Eigen::Vector3f(s_camParams.pos.x, s_camParams.pos.y, s_camParams.pos.z),
                        Eigen::Vector3f(s_camParams.theta.x, s_camParams.theta.y, s_camParams.theta.z), s_camParams.nearClip, s_camParams.fov);
                    screenX = uv(0);
                    screenY = uv(1);
                }

                if (CORRECT_2D_POINTS_BEHIND_CAMERA) {
                    //Corrections for points which are behind camera
                    Vector3 relativePos;
                    relativePos.x = pos.x - s_camParams.pos.x;
                    relativePos.y = pos.y - s_camParams.pos.y;
                    relativePos.z = pos.z - s_camParams.pos.z;
                    relativePos = convertCoordinateSystem(relativePos, m_camForwardVector, m_camRightVector, m_camUpVector);

                    //If behind camera update left/right bounds to reflect its position
                    if (relativePos.y < 0) {
                        if (relativePos.x > 0) {
                            screenX = 1.0;
                        }
                        else {
                            screenX = 0.0;
                        }
                    }
                }

                //Update if value outside current box
                if (screenX < bbox2d.left) bbox2d.left = screenX;
                if (screenX > bbox2d.right) bbox2d.right = screenX;
                if (screenY < bbox2d.top) bbox2d.top = screenY;
                if (screenY > bbox2d.bottom) bbox2d.bottom = screenY;
            }
        }
    }

    //Calculate truncation
    float absL = std::max<float>(0, bbox2d.left);
    float absR = std::min<float>(1, bbox2d.right);
    float absT = std::max<float>(0, bbox2d.top);
    float absB = std::min<float>(1, bbox2d.bottom);
    float areaInside = (absR - absL) * (absB - absT);
    float areaTotal = (bbox2d.right - bbox2d.left) * (bbox2d.bottom - bbox2d.top);
    truncation = 1 - (areaInside / areaTotal);

    //Set bbox boundaries
    bbox2d.left = std::max(0.0f, bbox2d.left);
    bbox2d.right = std::min(1.0f, bbox2d.right);
    bbox2d.top = std::max(0.0f, bbox2d.top);
    bbox2d.bottom = std::min(1.0f, bbox2d.bottom);

    //Entire object is out of bounds - do not count
    if (bbox2d.left == bbox2d.right || bbox2d.top == bbox2d.bottom) {
        success = false;
        return bbox2d;
    }

    std::ostringstream oss2;
    oss2 << "BBox left: " << bbox2d.left << " right: " << bbox2d.right << " top: " << bbox2d.top << " bot: " << bbox2d.bottom << std::endl <<
        "PosX: " << bbox2d.posX() << " PosY: " << bbox2d.posY() << " Width: " << bbox2d.width() << " Height: " << bbox2d.height();
    std::string str2 = oss2.str();
    log(str2);
    return bbox2d;
}

bool ObjectDetection::checkDirection(Vector3 unit, Vector3 point, Vector3 min, Vector3 max) {
    float dotPoint = dotProd(point, unit);
    float dotMax = dotProd(max, unit);
    float dotMin = dotProd(min, unit);

    if ((dotMax <= dotPoint && dotPoint <= dotMin) ||
        (dotMax >= dotPoint && dotPoint >= dotMin)) {
        return true;
    }
    return false;
}

//Point and objPos should be in world coordinates
void ObjectDetection::setEntityBBoxParameters(ObjEntity *e) {
    //Added BBOX_ADJUSTMENT_FACTOR as detailed models sometimes go outside 3D bboxes
    Vector3 forward; forward.y = e->dim.y * BBOX_ADJUSTMENT_FACTOR; forward.x = 0; forward.z = 0;
    forward = convertCoordinateSystem(forward, e->yVector, e->xVector, e->zVector);

    Vector3 right; right.x = e->dim.x * BBOX_ADJUSTMENT_FACTOR; right.y = 0; right.z = 0;
    right = convertCoordinateSystem(right, e->yVector, e->xVector, e->zVector);

    Vector3 up; up.z = e->dim.z * BBOX_ADJUSTMENT_FACTOR; up.x = 0; up.y = 0;
    up = convertCoordinateSystem(up, e->yVector, e->xVector, e->zVector);

    //position is given at bottom of bounding box (as per kitti)
    Vector3 objPos = e->worldPos;

    e->rearBotLeft.x = objPos.x - forward.x - right.x;
    e->rearBotLeft.y = objPos.y - forward.y - right.y;
    e->rearBotLeft.z = objPos.z - forward.z - right.z;

    e->frontBotLeft.x = objPos.x + forward.x - right.x;
    e->frontBotLeft.y = objPos.y + forward.y - right.y;
    e->frontBotLeft.z = objPos.z + forward.z - right.z;

    e->rearTopLeft.x = objPos.x - forward.x - right.x + 2 * up.x;
    e->rearTopLeft.y = objPos.y - forward.y - right.y + 2 * up.y;
    e->rearTopLeft.z = objPos.z - forward.z - right.z + 2 * up.z;

    e->rearBotRight.x = objPos.x - forward.x + right.x;
    e->rearBotRight.y = objPos.y - forward.y + right.y;
    e->rearBotRight.z = objPos.z - forward.z + right.z;

    e->u = getUnitVector(subtractVecs(e->frontBotLeft, e->rearBotLeft));
    e->v = getUnitVector(subtractVecs(e->rearTopLeft, e->rearBotLeft));
    e->w = getUnitVector(subtractVecs(e->rearBotRight, e->rearBotLeft));

    e->bbox2d.left = 1.0;// width;
    e->bbox2d.right = 0.0;
    e->bbox2d.top = 1.0;// height;
    e->bbox2d.bottom = 0.0;
}

//Point and objPos should be in world coordinates
bool ObjectDetection::in3DBox(Vector3 point, Vector3 objPos, Vector3 dim, Vector3 yVector, Vector3 xVector, Vector3 zVector) {
    Vector3 forward; forward.y = dim.y; forward.x = 0; forward.z = 0;
    forward = convertCoordinateSystem(forward, yVector, xVector, zVector);

    Vector3 right; right.x = dim.x; right.y = 0; right.z = 0;
    right = convertCoordinateSystem(right, yVector, xVector, zVector);

    Vector3 up; up.z = dim.z; up.x = 0; up.y = 0;
    up = convertCoordinateSystem(up, yVector, xVector, zVector);

    Vector3 rearBotLeft;
    rearBotLeft.x = objPos.x - forward.x - right.x - up.x;
    rearBotLeft.y = objPos.y - forward.y - right.y - up.y;
    rearBotLeft.z = objPos.z - forward.z - right.z - up.z;

    Vector3 frontBotLeft;
    frontBotLeft.x = objPos.x + forward.x - right.x - up.x;
    frontBotLeft.y = objPos.y + forward.y - right.y - up.y;
    frontBotLeft.z = objPos.z + forward.z - right.z - up.z;

    Vector3 rearTopLeft;
    rearTopLeft.x = objPos.x - forward.x - right.x + up.x;
    rearTopLeft.y = objPos.y - forward.y - right.y + up.y;
    rearTopLeft.z = objPos.z - forward.z - right.z + up.z;

    Vector3 rearBotRight;
    rearBotRight.x = objPos.x - forward.x + right.x - up.x;
    rearBotRight.y = objPos.y - forward.y + right.y - up.y;
    rearBotRight.z = objPos.z - forward.z + right.z - up.z;

    std::ostringstream oss2;
    oss2 << " rearBotLeft are: " << rearBotLeft.x << ", " << rearBotLeft.y << ", " << rearBotLeft.z;
    oss2 << "\nfrontBotLeft: " << frontBotLeft.x << ", " << frontBotLeft.y << ", " << frontBotLeft.z;
    oss2 << "\nrearTopLeft: " << rearTopLeft.x << ", " << rearTopLeft.y << ", " << rearTopLeft.z;
    oss2 << "\nrearBotRight: " << rearBotRight.x << ", " << rearBotRight.y << ", " << rearBotRight.z;
    oss2 << "\ndim: " << dim.x << ", " << dim.y << ", " << dim.z;
    std::string str2 = oss2.str();
    log(str2);

    Vector3 u = getUnitVector(subtractVecs(frontBotLeft, rearBotLeft));
    Vector3 v = getUnitVector(subtractVecs(rearTopLeft, rearBotLeft));
    Vector3 w = getUnitVector(subtractVecs(rearBotRight, rearBotLeft));

    if (!checkDirection(u, point, rearBotLeft, frontBotLeft)) return false;
    if (!checkDirection(v, point, rearBotLeft, rearTopLeft)) return false;
    if (!checkDirection(w, point, rearBotLeft, rearBotRight)) return false;

    return true;
}

//Takes in an entity and a point (in world coordinates) and returns true if the point resides within the
//entity's 3D bounding box.
//Note: Need to set the entity's parameters u,v,w, and rearBotLeft, etc...
bool ObjectDetection::in3DBox(ObjEntity* e, Vector3 point) {

    if (!checkDirection(e->u, point, e->rearBotLeft, e->frontBotLeft)) return false;
    if (!checkDirection(e->v, point, e->rearBotLeft, e->rearTopLeft)) return false;
    if (!checkDirection(e->w, point, e->rearBotLeft, e->rearBotRight)) return false;

    return true;
}

bool ObjectDetection::isPointOccluding(Vector3 worldPos, Vector3 position) {
    float pointDist = sqrt(SYSTEM::VDIST2(s_camParams.pos.x, s_camParams.pos.y, s_camParams.pos.z, worldPos.x, worldPos.y, worldPos.z));
    float distObjCenter = sqrt(SYSTEM::VDIST2(s_camParams.pos.x, s_camParams.pos.y, s_camParams.pos.z, position.x, position.y, position.z));
    if (pointDist < distObjCenter) {
        float groundZ;
        GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(worldPos.x, worldPos.y, worldPos.z, &(groundZ), 0);
        //Check it is not the ground in the image (or the ground is much higher/lower than the object)
        if ((groundZ + 0.1) < worldPos.z || s_camParams.pos.z > (position.z + 4) || s_camParams.pos.z < (position.z - 2)) {
            return true;
        }
    }
    return false;
}

void ObjectDetection::processSegmentation() {
    //Converting vehicle dimensions from vehicle to world coordinates for offset position
    Vector3 worldX; worldX.x = 1; worldX.y = 0; worldX.z = 0;
    Vector3 worldY; worldY.x = 0; worldY.y = 1; worldY.z = 0;
    Vector3 worldZ; worldZ.x = 0; worldZ.y = 0; worldZ.z = 1;
    Vector3 xVectorCam = convertCoordinateSystem(worldX, m_camForwardVector, m_camRightVector, m_camUpVector);
    Vector3 yVectorCam = convertCoordinateSystem(worldY, m_camForwardVector, m_camRightVector, m_camUpVector);
    Vector3 zVectorCam = convertCoordinateSystem(worldZ, m_camForwardVector, m_camRightVector, m_camUpVector);

    int stencilPointCount = 0;
    int occlusionPointCount = 0;

    //Set the bounding box parameters (for reducing # of calculations per pixel)
    for (auto &entry : m_curFrame.vehicles) {
        setEntityBBoxParameters(&entry.second);
    }
    for (auto &entry : m_curFrame.peds) {
        setEntityBBoxParameters(&entry.second);
    }

    for (int j = 0; j < s_camParams.height; ++j) {
        for (int i = 0; i < s_camParams.width; ++i) {
            uint8_t stencilVal = m_pStencil[j * s_camParams.width + i];

            if (stencilVal == STENCIL_TYPE_VEHICLE || stencilVal == STENCIL_TYPE_NPC) {
                processStencilPixel(stencilVal, j, i, xVectorCam, yVectorCam, zVectorCam);
            }
            else if (stencilVal == STENCIL_TYPE_OWNCAR) {
                addPointToSegImages(i, j, m_ownVehicle);
            }
        }
    }
}

//j is y coordinate (top=0), i is x coordinate (left = 0)
void ObjectDetection::processStencilPixel(const uint8_t &stencilVal, const int &j, const int &i,
                                          const Vector3 &xVectorCam, const Vector3 &yVectorCam, const Vector3 &zVectorCam) {
    float ndc = m_pDepth[j * s_camParams.width + i];
    Vector3 relPos = depthToCamCoords(ndc, i, j);
    Vector3 worldPos = convertCoordinateSystem(relPos, yVectorCam, xVectorCam, zVectorCam);
    worldPos.x += s_camParams.pos.x;
    worldPos.y += s_camParams.pos.y;
    worldPos.z += s_camParams.pos.z;

    //Obtain proper map for stencil type
    EntityMap* eMap;
    if (stencilVal == STENCIL_TYPE_VEHICLE) {
        eMap = &m_curFrame.vehicles;
    }
    else {
        eMap = &m_curFrame.peds;
    }

    //Get vector of entities which point resides in their 3D box
    std::vector<ObjEntity*> pointEntities;
    for (auto &entry : *eMap) {
        ObjEntity* e = &(entry.second);
        if (in3DBox(e, worldPos)) {
            pointEntities.push_back(e);
        }
    }

    //3 choices, no, single, or multiple 3D box matches
    if (pointEntities.empty()) {
        //TODO
    }
    else if (pointEntities.size() == 1) {
        addPoint(i, j, *pointEntities[0]);
    }
    else {
        log("pointEntities duplicate.", true);
        //TODO
    }
}

//Add 2D point to an entity
void ObjectDetection::addPoint(int i, int j, ObjEntity &e) {
    float x = (float)i / (float)s_camParams.width;
    float y = (float)j / (float)s_camParams.height;

    if (x < e.bbox2d.left) e.bbox2d.left = x;
    if (x > e.bbox2d.right) e.bbox2d.right = x;
    if (y < e.bbox2d.top) e.bbox2d.top = y;
    if (y > e.bbox2d.bottom) e.bbox2d.bottom = y;
    ++e.pointsHit2D;

    addPointToSegImages(i, j, e.entityID);
}

void ObjectDetection::addPointToSegImages(int i, int j, int entityID) {
    //Index of point in all image buffers
    int idx = j * s_camParams.width + i;

    //instance seg is image with exact entityIDs
    m_pInstanceSeg[idx] = (uint32_t)entityID;

    //RGB image is 3 bytes per pixel
    int segIdx = 3 * idx;
    uint8_t red = m_pStencilSeg[segIdx];
    uint8_t green = m_pStencilSeg[segIdx + 1];
    uint8_t blue = m_pStencilSeg[segIdx + 2];
    if (red == 0 && green == 0 && blue == 0) {
        int newVal = 47 * entityID; //Just to produce unique but different colours
        red = (newVal + 13 * entityID) % 255;
        green = (newVal / 255) % 255;
        blue = newVal % 255;
    }
    else {
        red = 255;
        green = 255;
        blue = 255;
    }
    uint8_t* p = m_pStencilSeg + segIdx;
    *p = red;
    *(p + 1) = green;
    *(p + 2) = blue;
}

void ObjectDetection::processOcclusion() {
    //TODO -> Take occlusion from processBBox2D, process after all 2D points are segmented
}

//Position is world position of object center
BBox2D ObjectDetection::processBBox2D(BBox2D bbox, uint8_t stencilType, Vector3 position, Vector3 dim, Vector3 forwardVector, Vector3 rightVector, Vector3 upVector,
    Vector3 xVector, Vector3 yVector, Vector3 zVector, int entityID, int &pointsHit2D, float &occlusion, bool pedOnBike) {
    //position is given at bottom of bounding box (as per kitti)
    position.z += dim.z;

    BBox2D processed;
    processed.left = 1;
    processed.right = 0;
    processed.top = 1;
    processed.bottom = 0;

    if (m_pStencil == NULL || m_pDepth == NULL) return processed;

    int top = (int)floor(bbox.top * s_camParams.height);
    int bot = std::min(s_camParams.height, (int)ceil(bbox.bottom * s_camParams.height));
    int right = std::min(s_camParams.width, (int)ceil(bbox.right * s_camParams.width));
    int left = (int)floor(bbox.left * s_camParams.width);

    //Converting vehicle dimensions from vehicle to world coordinates for offset position
    Vector3 worldX; worldX.x = 1; worldX.y = 0; worldX.z = 0;
    Vector3 worldY; worldY.x = 0; worldY.y = 1; worldY.z = 0;
    Vector3 worldZ; worldZ.x = 0; worldZ.y = 0; worldZ.z = 1;
    Vector3 xVectorCam = convertCoordinateSystem(worldX, m_camForwardVector, m_camRightVector, m_camUpVector);
    Vector3 yVectorCam = convertCoordinateSystem(worldY, m_camForwardVector, m_camRightVector, m_camUpVector);
    Vector3 zVectorCam = convertCoordinateSystem(worldZ, m_camForwardVector, m_camRightVector, m_camUpVector);

    int stencilPointCount = 0;
    int occlusionPointCount = 0;

    for (int j = top; j < bot; ++j) {
        for (int i = left; i < right; ++i) {
            uint8_t stencilVal = m_pStencil[j * s_camParams.width + i];
            float ndc = m_pDepth[j * s_camParams.width + i];
            Vector3 relPos = depthToCamCoords(ndc, i, j);
            Vector3 worldPos = convertCoordinateSystem(relPos, yVectorCam, xVectorCam, zVectorCam);
            worldPos.x += s_camParams.pos.x;
            worldPos.y += s_camParams.pos.y;
            worldPos.z += s_camParams.pos.z;

            if (stencilType == stencilVal || (pedOnBike && stencilVal == STENCIL_TYPE_NPC)) {
                ++stencilPointCount;

                //Index of point
                int idx = j * s_camParams.width + i;

                if (in3DBox(worldPos, position, dim, yVector, xVector, zVector)) {
                    float x = (float)i / (float)s_camParams.width;
                    float y = (float)j / (float)s_camParams.height;

                    std::ostringstream oss;
                    oss << "x,y: " << x << ", " << y;
                    std::string str = oss.str();
                    log(str);
                    if (x < processed.left) processed.left = x;
                    if (x > processed.right) processed.right = x;
                    if (y < processed.top) processed.top = y;
                    if (y > processed.bottom) processed.bottom = y;
                    ++pointsHit2D;

                    uint32_t val = m_pInstanceSeg[idx];
                    if (val != 0) {
                        if (m_duplicateBoxPoints.find(idx) == m_duplicateBoxPoints.end()) {
                            std::vector<int> vec;
                            vec.push_back(val);
                            //TODO test if entityID is larger than max value
                            vec.push_back(entityID);
                            m_duplicateBoxPoints.insert(std::pair<int, std::vector<int>>(idx, vec));
                        }
                        else {
                            m_duplicateBoxPoints[idx].push_back(entityID);
                        }
                    }
                    m_pInstanceSeg[j * s_camParams.width + i] = (uint32_t)entityID;

                    //RGB image is 3 bytes per pixel
                    int segIdx = 3 * idx;
                    uint8_t red = m_pStencilSeg[segIdx];
                    uint8_t green = m_pStencilSeg[segIdx + 1];
                    uint8_t blue = m_pStencilSeg[segIdx + 2];
                    if (red == 0 && green == 0 && blue == 0) {
                        int newVal = 47 * entityID; //Just to produce unique but different colours
                        red = (newVal + 13 * entityID) % 255;
                        green = (newVal / 255) % 255;
                        blue = newVal % 255;
                    }
                    else {
                        red = 255;
                        green = 255;
                        blue = 255;
                    }
                    uint8_t* p = m_pStencilSeg + segIdx;
                    *p = red;
                    *(p + 1) = green;
                    *(p + 2) = blue;
                }
                else {
                    if (m_coarseInstancePoints.find(idx) == m_coarseInstancePoints.end()) {
                        std::vector<int> vec;
                        //TODO test if entityID is larger than max value
                        vec.push_back(entityID);
                        m_coarseInstancePoints.insert(std::pair<int, std::vector<int>>(idx, vec));
                    }
                    else {
                        m_coarseInstancePoints[idx].push_back(entityID);
                    }

                    //TODO this should probably be moved until after all points are decided
                    if (isPointOccluding(worldPos, position)) {
                        ++occlusionPointCount;
                        if (STENCIL_TYPE_NPC == stencilType) {
                            m_pOcclusionImage[j * s_camParams.width + i] = 255;
                        }
                    }
                }
            }
            else if (stencilVal != STENCIL_TYPE_SKY && isPointOccluding(worldPos, position)) {
                ++occlusionPointCount;
                if (STENCIL_TYPE_NPC == stencilType) {
                    m_pOcclusionImage[j * s_camParams.width + i] = 255;
                }
            }
        }
    }

    occlusion = 1.0;
    int divisor = occlusionPointCount + pointsHit2D;
    if (divisor != 0) {
        occlusion = (float)occlusionPointCount / (float)(occlusionPointCount + pointsHit2D);
    }

    std::ostringstream oss;
    oss << "top, bot, right, left: " << top << ", " << bot << ", " << left << ", " << right <<
        "\nProcessed: " << processed.top << ", " << processed.bottom << ", " << processed.left << ", " << processed.right <<
        "\nStencil points: " << stencilPointCount << " points: " << pointsHit2D;
    std::string str = oss.str();
    log(str);

    return processed;
}

//dim is in full width/height/length
static void updatePosition(float &origPos, float pedPos, float &origDim, float pedDim) {
    float diffPos = pedPos + pedDim / 2 - (origPos + origDim / 2);
    float diffNeg = pedPos - pedDim / 2 - (origPos - origDim / 2);
    if (diffPos > 0) { //If ped dim is outside original bike dimension
        origPos += diffPos / 2;
        origDim += diffPos;
    }
    if (diffNeg < 0) { //If ped dim is outside original bike dimension
        //If it is negative we need to add the position (since it is relative)
        //but subtract the dimension (really make it bigger since diffNeg < 0 if it is bigger)
        origPos += diffNeg / 2;
        origDim -= diffNeg;
    }
}

void ObjectDetection::getRollAndPitch(Vector3 rightVector, Vector3 forwardVector, Vector3 upVector, float &pitch, float &roll) {
    Vector3 kittiForwardVector = convertCoordinateSystem(forwardVector, m_camForwardVector, m_camRightVector, m_camUpVector);
    Vector3 kittiRightVector = convertCoordinateSystem(rightVector, m_camForwardVector, m_camRightVector, m_camUpVector);

    roll = atan2(-kittiRightVector.z, sqrt(pow(kittiRightVector.y, 2) + pow(kittiRightVector.x, 2)));
    pitch = atan2(-kittiForwardVector.z, sqrt(pow(kittiForwardVector.y, 2) + pow(kittiForwardVector.x, 2)));
}

Vector3 ObjectDetection::correctOffcenter(Vector3 position, Vector3 min, Vector3 max, Vector3 forwardVector, Vector3 rightVector, Vector3 upVector, Vector3 &offcenter) {
    //Amount dimensions are offcenter
    offcenter.x = (max.x + min.x) / 2;
    offcenter.y = (max.y + min.y) / 2;
    offcenter.z = min.z; //KITTI position is at object ground plane

    //Converting vehicle dimensions from vehicle to world coordinates for offset position
    Vector3 worldX; worldX.x = 1; worldX.y = 0; worldX.z = 0;
    Vector3 worldY; worldY.x = 0; worldY.y = 1; worldY.z = 0;
    Vector3 worldZ; worldZ.x = 0; worldZ.y = 0; worldZ.z = 1;
    Vector3 xVector = convertCoordinateSystem(worldX, forwardVector, rightVector, upVector);
    Vector3 yVector = convertCoordinateSystem(worldY, forwardVector, rightVector, upVector);
    Vector3 zVector = convertCoordinateSystem(worldZ, forwardVector, rightVector, upVector);
    Vector3 offcenterPosition = convertCoordinateSystem(offcenter, yVector, xVector, zVector);

    //Update object position to be consistent with KITTI (symmetrical dimensions except for z which is ground)
    position.x = position.x + offcenterPosition.x;
    position.y = position.y + offcenterPosition.y;
    position.z = position.z + offcenterPosition.z;

    return position;
}

void ObjectDetection::update3DPointsHit(ObjEntity* e) {
    if (m_entitiesHit.find(e->entityID) != m_entitiesHit.end()) {
        HitLidarEntity* hitLidarEnt = m_entitiesHit[e->entityID];
        e->pointsHit3D = hitLidarEnt->pointsHit;
        std::ostringstream oss2;
        oss2 << "***entity Id: " << e->entityID << ", point hit: " << hitLidarEnt->pointsHit << ", " << e->pointsHit3D;
        std::string str = oss2.str();
        log(str, true);
    }
    else {
        std::ostringstream oss2;
        oss2 << "***entity Id: " << e->entityID << " not found";
        std::string str = oss2.str();
        log(str, true);
    }
}

void ObjectDetection::update3DPointsHit() {
    //Set the bounding box parameters (for reducing # of calculations per pixel)
    for (auto &entry : m_curFrame.vehicles) {
        update3DPointsHit(&entry.second);
    }
    for (auto &entry : m_curFrame.peds) {
        update3DPointsHit(&entry.second);
    }
}

bool ObjectDetection::getEntityVector(ObjEntity &entity, int entityID, Hash model, int classid, std::string type, std::string modelString, bool isPedInV, int vPedIsIn) {
    bool success = false;

    Vector3 FUR; //Front Upper Right
    Vector3 BLL; //Back Lower Lelft
    Vector3 dim; //Vehicle dimensions
    Vector3 upVector, rightVector, forwardVector, position; //Vehicle position
    Vector3 min;
    Vector3 max;
    Vector3 speedVector;
    float heading, speed;

    bool isOnScreen = ENTITY::IS_ENTITY_ON_SCREEN(entityID);
    if (isOnScreen) {
        //Check if it is in screen
        ENTITY::GET_ENTITY_MATRIX(entityID, &forwardVector, &rightVector, &upVector, &position); //Blue or red pill

        float distance = sqrt(SYSTEM::VDIST2(s_camParams.pos.x, s_camParams.pos.y, s_camParams.pos.z, position.x, position.y, position.z));
        
        //Need to limit distance as pixels won't register entities past the far clip
        if (distance > s_camParams.farClip) return false;

        speed = ENTITY::GET_ENTITY_SPEED(entityID);

        GAMEPLAY::GET_MODEL_DIMENSIONS(model, &min, &max);

        //Need to adjust dimensions for pedestrians
        if (classid == PEDESTRIAN_CLASS_ID) {
            float groundZ;
            GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(position.x, position.y, position.z, &(groundZ), 0);
            float negZ = groundZ - position.z;

            //Pedestrians on balconies can cause problems
            if (negZ > -2.0 && negZ < 0) {
                min.z = groundZ - position.z;
            }

            if (SET_PED_BOXES) {
                min.x = -PED_BOX_WIDTH / 2.0f;
                max.x = PED_BOX_WIDTH / 2.0f;
                if (speed > 1) {
                    min.y = -PED_BOX_WALKING_LEN / 2.0f;
                    max.y = PED_BOX_WALKING_LEN / 2.0f;
                }
                else {
                    min.y = -PED_BOX_LENGTH / 2.0f;
                    max.y = PED_BOX_LENGTH / 2.0f;
                }
            }

            std::ostringstream oss2;
            oss2 << "***min: " << min.x << ", " << min.y << ", " << min.z <<
                "\nmax: " << max.x << ", " << max.y << ", " << max.z;
            std::string str = oss2.str();
            log(str);
        }

        //Calculate size
        dim.x = 0.5*(max.x - min.x);
        dim.y = 0.5*(max.y - min.y);
        dim.z = 0.5*(max.z - min.z);

        //TODO Remove these calculations and put x/y/zVector and m_camRight/Forward/UpVector in s_camParams
        //Converting vehicle dimensions from vehicle to world coordinates for offset position
        Vector3 worldX; worldX.x = 1; worldX.y = 0; worldX.z = 0;
        Vector3 worldY; worldY.x = 0; worldY.y = 1; worldY.z = 0;
        Vector3 worldZ; worldZ.x = 0; worldZ.y = 0; worldZ.z = 1;
        Vector3 xVector = convertCoordinateSystem(worldX, forwardVector, rightVector, upVector);
        Vector3 yVector = convertCoordinateSystem(worldY, forwardVector, rightVector, upVector);
        Vector3 zVector = convertCoordinateSystem(worldZ, forwardVector, rightVector, upVector);

        Vector3 offcenter;
        position = correctOffcenter(position, min, max, forwardVector, rightVector, upVector, offcenter);

        success = true;

        speedVector = ENTITY::GET_ENTITY_SPEED_VECTOR(entityID, false);
        if (speed > 0) {
            heading = GAMEPLAY::GET_HEADING_FROM_VECTOR_2D(speedVector.x - vehicleForwardVector.x, speedVector.y - vehicleForwardVector.y);
        }
        else {
            heading = GAMEPLAY::GET_HEADING_FROM_VECTOR_2D(forwardVector.x - vehicleForwardVector.x, forwardVector.y - vehicleForwardVector.y);
        }

        //Kitti dimensions
        float kittiHeight = 2 * dim.z;
        float kittiWidth = 2 * dim.x;
        float kittiLength = 2 * dim.y;
                    
        //Have only seen sitting people with height <= 1.0
        if (classid == PEDESTRIAN_CLASS_ID && kittiHeight <= 1.0) {
            type = "Person_sitting";
        }

        if (abs(offcenter.y) > 0.5 && classid == 0) {
            std::ostringstream oss2;
            oss2 << "Instance Index: " << instance_index << " Dimensions are: " << dim.x << ", " << dim.y << ", " << dim.z;
            oss2 << "\nMax: " << max.x << ", " << max.y << ", " << max.z;
            oss2 << "\nMin: " << min.x << ", " << min.y << ", " << min.z;
            oss2 << "\noffset: " << offcenter.x << ", " << offcenter.y << ", " << offcenter.z;
            std::string str2 = oss2.str();
            log(str2);
        }

        Vector3 relativePos;
        relativePos.x = position.x - s_camParams.pos.x;
        relativePos.y = position.y - s_camParams.pos.y;
        relativePos.z = position.z - s_camParams.pos.z;

        Vector3 kittiForwardVector = convertCoordinateSystem(forwardVector, m_camForwardVector, m_camRightVector, m_camUpVector);
        float rot_y = -atan2(kittiForwardVector.y, kittiForwardVector.x);

        relativePos = convertCoordinateSystem(relativePos, m_camForwardVector, m_camRightVector, m_camUpVector);

        //Update object position to be consistent with KITTI (symmetrical dimensions except for z which is ground)
        //relativePos.y = relativePos.y - CAM_OFFSET_FORWARD;
        //relativePos.z = relativePos.z - CAM_OFFSET_UP;

        //Convert to KITTI camera coordinates
        Vector3 kittiPos;
        kittiPos.x = relativePos.x;
        kittiPos.y = -relativePos.z;
        kittiPos.z = relativePos.y;

        //alpha is rot_y + tan^-1(z/x) + PI/2
        float beta_kitti = atan2(kittiPos.z, kittiPos.x);
        float alpha_kitti = rot_y + beta_kitti - PI / 2;

        log("After processBBox2D");
        bool foundPedOnBike = false;
        if (PROCESS_PEDS_ON_BIKES) {
            if (m_pedsInVehicles.find(entityID) != m_pedsInVehicles.end()) {
                std::vector<Ped> pedsOnV = m_pedsInVehicles[entityID];

                for (auto ped : pedsOnV) {
                    std::ostringstream oss;
                    oss << "Found ped on bike at index: " << instance_index;
                    std::string str = oss.str();
                    log(str, true);
                    log("Found ped on bike", true);
                    foundPedOnBike = true;
                    //Extend 3D/2D boxes with peds, change id in segmentation image

                    if (m_curFrame.peds.find(ped) != m_curFrame.peds.end()) {
                        ObjEntity pedO = m_curFrame.peds[ped];

                        float horizDist = sqrt(pow(pedO.location.x - kittiPos.x, 2) + pow(pedO.location.z - kittiPos.z, 2));
                        float vertDist = pedO.location.y - kittiPos.y;
                        if (horizDist < 0.5 && vertDist <= 2.0) {
                            foundPedOnBike = true;
                            m_curFrame.peds[ped].isPedInV = true;
                            m_curFrame.peds[ped].vPedIsIn = entityID;

                            //This method assume the pedestrian x/z coordinates are the same as the vehicles (only update relative height position)
                            kittiWidth = kittiWidth > pedO.width ? kittiWidth : pedO.width;
                            kittiLength = kittiLength > pedO.length ? kittiLength : pedO.length;
                            updatePosition(kittiPos.y, pedO.location.y, kittiHeight, pedO.height);

                            //This method would need to be changed to accommodate object coordinate system vs kitti coordinate system
                            /*updatePosition(kittiPos.x, pedO.location.x, kittiWidth, pedO.width);
                            updatePosition(kittiPos.y, pedO.location.y, kittiWidth, pedO.width);
                            updatePosition(kittiPos.z, pedO.location.z, kittiWidth, pedO.width);*/
                        }
                    }
                }
            }
            else if (TESTING_PEDS_ON_BIKES && classid == 1 || classid == 2 || classid == 3) { //bicycle, bike, or quadbike
                for (auto ped : m_curFrame.peds) {
                    ObjEntity pedO = ped.second;
                    float horizDist = sqrt(pow(pedO.location.x - kittiPos.x, 2) + pow(pedO.location.z - kittiPos.z, 2));
                    float vertDist = pedO.location.y - kittiPos.y;
                    if (horizDist < 0.5 && vertDist <= 2.0) {
                        m_curFrame.peds[ped.first].isPedInV = true;
                        m_curFrame.peds[ped.first].vPedIsIn = entityID;
                        foundPedOnBike = true;
                        std::ostringstream oss;
                        oss << "****************************Alternate Found ped on bike at index: " << instance_index;
                        std::string str = oss.str();
                        log(str, true);
                                    
                        //This method assume the pedestrian x/z coordinates are the same as the vehicles (only update relative height position)
                        kittiWidth = kittiWidth > pedO.width ? kittiWidth : pedO.width;
                        kittiLength = kittiLength > pedO.length ? kittiLength : pedO.length;
                        updatePosition(kittiPos.y, pedO.location.y, kittiHeight, pedO.height);

                        //This method would need to be changed to accommodate object coordinate system vs kitti coordinate system
                        /*updatePosition(kittiPos.x, pedO.location.x, kittiWidth, pedO.width);
                        updatePosition(kittiPos.y, pedO.location.y, kittiWidth, pedO.width);
                        updatePosition(kittiPos.z, pedO.location.z, kittiWidth, pedO.width);*/
                    }
                }
            }
        }
        log("After pedsonbikes");

        //Attempts to find bbox on screen, if entire 2D box is offscreen returns false
        float truncation = 0;
        BBox2D bbox2d = BBox2DFrom3DObject(position, dim, forwardVector, rightVector, upVector, success, truncation);
        if (!success) {
            return success;
        }

        //stencil type for vehicles like cars, bikes...
        int stencilType = STENCIL_TYPE_VEHICLE;
        //Pedestrian classid
        if (classid == PEDESTRIAN_CLASS_ID) stencilType = STENCIL_TYPE_NPC; //For NPCs
        int pointsHit2D = 0;
        float occlusion = 0;
        BBox2D bbox2dProcessed;
        //BBox2D bbox2dProcessed = processBBox2D(bbox2d, stencilType, position, dim, forwardVector, rightVector, upVector, xVector, yVector, zVector, entityID, pointsHit2D, occlusion, foundPedOnBike);
        //Do not allow entity through if it has no points hit on the 2D screen
        /*if (pointsHit2D == 0) {
            return false;
        }*/

        float roll = atan2(-rightVector.z, sqrt(pow(rightVector.y, 2) + pow(rightVector.x, 2)));
        float pitch = atan2(-forwardVector.z, sqrt(pow(forwardVector.y, 2) + pow(forwardVector.x, 2)));
        getRollAndPitch(rightVector, forwardVector, upVector, pitch, roll);
        //To prevent negative zeros
        if (abs(roll) <= 0.0001) roll = 0.0f;
        if (abs(pitch) <= 0.0001) pitch = 0.0f;

        entity.entityID = entityID;
        entity.classID = classid;
        entity.speed = speed;
        entity.heading = heading;

        entity.height = kittiHeight;
        entity.width = kittiWidth;
        entity.length = kittiLength;
        entity.dim = dim;

        entity.offcenter = offcenter;
        entity.location = kittiPos;
        entity.worldPos = position;

        entity.rotation_y = rot_y;
        entity.alpha = alpha_kitti;
        entity.distance = distance;

        //TODO This is way it should be when stencil buffer is working
        entity.bbox2d.left = bbox2dProcessed.left * s_camParams.width;
        entity.bbox2d.top = bbox2dProcessed.top * s_camParams.height;
        entity.bbox2d.right = bbox2dProcessed.right * s_camParams.width;
        entity.bbox2d.bottom = bbox2dProcessed.bottom * s_camParams.height;
                    
        entity.bbox2dUnprocessed.left = bbox2d.left * s_camParams.width;
        entity.bbox2dUnprocessed.top = bbox2d.top * s_camParams.height;
        entity.bbox2dUnprocessed.right = bbox2d.right * s_camParams.width;
        entity.bbox2dUnprocessed.bottom = bbox2d.bottom * s_camParams.height;

        entity.pointsHit2D = pointsHit2D;
        entity.pointsHit3D = 0;//Updated later
        entity.truncation = truncation;
        entity.occlusion = occlusion;
        entity.pitch = pitch;
        entity.roll = roll;
        entity.model = model;
        entity.modelString = modelString;
        entity.objType = type;

        if (trackFirstFrame.find(entityID) == trackFirstFrame.end()) {
            trackFirstFrame.insert(std::pair<int, int>(entityID, instance_index));
        }
        entity.trackFirstFrame = trackFirstFrame[entityID];

        entity.isPedInV = isPedInV;
        entity.vPedIsIn = vPedIsIn;

        entity.xVector = xVector;
        entity.yVector = yVector;
        entity.zVector = zVector;
        log("End of getEntityVector");
    }

    return success;
}

void ObjectDetection::setVehiclesList() {
    m_curFrame.vehicles.clear();
    log("Setting vehicles list.");
    const int ARR_SIZE = 1024;
    Vehicle vehicles[ARR_SIZE];

    Hash model;
    int classid;

    int count = worldGetAllVehicles(vehicles, ARR_SIZE);
    for (int i = 0; i < count; i++) {
        if (vehicles[i] == m_vehicle) continue; //Don't process perspective car!

        model = ENTITY::GET_ENTITY_MODEL(vehicles[i]);
        if (VEHICLE::IS_THIS_MODEL_A_CAR(model)) classid = 0;
        else if (VEHICLE::IS_THIS_MODEL_A_BIKE(model)) classid = 1;
        else if (VEHICLE::IS_THIS_MODEL_A_BICYCLE(model)) classid = 2;
        else if (VEHICLE::IS_THIS_MODEL_A_QUADBIKE(model)) classid = 3;
        else if (VEHICLE::IS_THIS_MODEL_A_BOAT(model)) classid = 4;
        else if (VEHICLE::IS_THIS_MODEL_A_PLANE(model)) classid = 5;
        else if (VEHICLE::IS_THIS_MODEL_A_HELI(model)) classid = 6;
        else if (VEHICLE::IS_THIS_MODEL_A_TRAIN(model)) classid = 7;
        else if (VEHICLE::_IS_THIS_MODEL_A_SUBMERSIBLE(model)) classid = 8;
        else classid = 9; //unknown (ufo?)

        //Get the model string, convert it to lowercase then find it in lookup table
        std::string modelString = VEHICLE::GET_DISPLAY_NAME_FROM_VEHICLE_MODEL(model);
        std::string before = modelString;
        std::transform(modelString.begin(), modelString.end(), modelString.begin(), ::tolower);
        std::string type = "Unknown";
        modelString.erase(remove_if(modelString.begin(), modelString.end(), [](char c) { return !isalpha(c); }), modelString.end());
        auto search = m_vLookup.find(modelString);
        if (search != m_vLookup.end()) {
            type = search->second;
        }
        else {
            std::ostringstream oss;
            oss << "Entity Model/type/hash: " << modelString << ", " << type << ", " << model << ", before: " << before << ", index: " << instance_index;
            std::string str = oss.str();
            log(str, true);
        }

        if (type == "Unknown") {
            if (VEHICLE::IS_THIS_MODEL_A_CAR(model)) type = "Car";
            else if (VEHICLE::IS_THIS_MODEL_A_BIKE(model)) type = "Moterbike";
            else if (VEHICLE::IS_THIS_MODEL_A_BICYCLE(model)) type = "Cyclist";
            else if (VEHICLE::IS_THIS_MODEL_A_QUADBIKE(model)) type = "Moterbike";
            else if (VEHICLE::IS_THIS_MODEL_A_BOAT(model)) type = "Boat";
            else if (VEHICLE::IS_THIS_MODEL_A_PLANE(model)) type = "Airplane";
            else if (VEHICLE::IS_THIS_MODEL_A_HELI(model)) type = "Airplane";
            else if (VEHICLE::IS_THIS_MODEL_A_TRAIN(model)) type = "Railed";
            else if (VEHICLE::_IS_THIS_MODEL_A_SUBMERSIBLE(model)) type = "Boat";
        }

        WorldObject wObj;
        wObj.e = vehicles[i];
        wObj.type = type;
        wObj.model = model;
        m_worldVehicles.push_back(wObj);

        ObjEntity objEntity;
        bool success = getEntityVector(objEntity, vehicles[i], model, classid, type, modelString, false, -1);
        if (success) {
            if (m_curFrame.vehicles.find(objEntity.entityID) == m_curFrame.vehicles.end()) {
                m_curFrame.vehicles.insert(std::pair<int, ObjEntity>(objEntity.entityID, objEntity));
            }
        }
    }
}

void ObjectDetection::setPedsList() {
    m_curFrame.peds.clear();
    log("Setting peds list.");
    const int ARR_SIZE = 1024;
    Ped peds[ARR_SIZE];

    Hash model;
    int classid;

    int count = worldGetAllPeds(peds, ARR_SIZE);
    for (int i = 0; i < count; i++) {
        bool isPedInV = false;
        Vehicle vPedIsIn = -1;
        if (PED::IS_PED_IN_ANY_VEHICLE(peds[i], TRUE)) {
            vPedIsIn = PED::GET_VEHICLE_PED_IS_IN(peds[i], FALSE);
            Hash vModel = ENTITY::GET_ENTITY_MODEL(vPedIsIn);

            //Update bounding boxes/stencils for all bike type vehicles
            if (VEHICLE::IS_THIS_MODEL_A_BIKE(vModel) ||
                VEHICLE::IS_THIS_MODEL_A_BICYCLE(vModel) ||
                VEHICLE::IS_THIS_MODEL_A_QUADBIKE(vModel)) {

                if (m_pedsInVehicles.find(vPedIsIn) != m_pedsInVehicles.end()) {
                    log("Putting ped in a vehicle in list.", true);
                    m_pedsInVehicles[vPedIsIn].push_back(peds[i]);
                }
                else {
                    m_pedsInVehicles.insert(std::pair<Vehicle, std::vector<Ped>>(vPedIsIn, { peds[i] }));
                }
            }
            isPedInV = true; //Don't add peds in vehicles as unique objects!
        }

        std::string type = "Pedestrian";
        if (PED::GET_PED_TYPE(peds[i]) == 28) {
            classid = 11; //animal
            type = "Animal";
        }
        else classid = PEDESTRIAN_CLASS_ID;

        if (RETAIN_ANIMALS || classid != 11) {
            if (!isPedInV) {
                WorldObject wObj;
                wObj.e = peds[i];
                wObj.type = type;
                wObj.model = model;
                m_worldPeds.push_back(wObj);
            }

            model = ENTITY::GET_ENTITY_MODEL(peds[i]);

            ObjEntity objEntity;
            bool success = getEntityVector(objEntity, peds[i], model, classid, type, type, isPedInV, vPedIsIn);
            if (success) {
                if (m_curFrame.peds.find(objEntity.entityID) == m_curFrame.peds.end()) {
                    m_curFrame.peds.insert(std::pair<int, ObjEntity>(objEntity.entityID, objEntity));
                }
            }
        }
    }
}

void ObjectDetection::setFilenames() {
    m_imgFilename = getStandardFilename("image_2", ".png");
    m_veloFilename = getStandardFilename("velodyne", ".bin");
    m_depthFilename = getStandardFilename("depth", ".bin");
    m_depthPCFilename = getStandardFilename("depthPC", ".bin");
    m_depthImgFilename = getStandardFilename("depthImage", ".png");
    m_stencilFilename = getStandardFilename("stencil", ".raw");
    m_stencilImgFilename = getStandardFilename("stencilImage", ".png");
    m_segImgFilename = getStandardFilename("segImage", ".png");
    m_occImgFilename = getStandardFilename("occlusionImage", ".png");
    m_unusedPixelsFilename = getStandardFilename("unusedPixelsImage", ".png");
    m_calibFilename = getStandardFilename("calib", ".txt");
    m_labelsFilename = getStandardFilename("label_2", ".txt");
    m_labelsUnprocessedFilename = getStandardFilename("labelsUnprocessed", ".txt");
    m_labelsAugFilename = getStandardFilename("label_aug_2", ".txt");
    m_groundPointsFilename = getStandardFilename("groundPointsImg", ".png");
    m_instSegFilename = getStandardFilename("instSeg", ".png");
    m_instSegImgFilename = getStandardFilename("instSegImage", ".png");
    m_posFilename = getStandardFilename("position_world", ".txt");
    m_egoObjectFilename = getStandardFilename("ego_object", ".txt");

    m_veloFilenameU = getStandardFilename("velodyneU", ".bin");
    m_depthPCFilenameU = getStandardFilename("depthPCU", ".bin");
}

void ObjectDetection::setupLiDAR() {
    if (pointclouds && !lidar_initialized) //flag if activate the LiDAR
    {
        //Specs on Velodyne HDL-64E
        //0.09f azimuth resolution
        //26.8 vertical fov (+2 degrees up to -24.8 degrees down)
        //0.420 vertical resolution
        lidar.Init3DLiDAR_FOV(MAX_LIDAR_DIST, 90.0f, 0.09f, 26.9f, 0.420f, 2.0f);
        lidar.AttachLiDAR2Camera(camera, ped);
        lidar_initialized = true;
        m_pDMPointClouds = (float *)malloc(s_camParams.width * s_camParams.height * FLOATS_PER_POINT * sizeof(float));
        m_pDMImage = (uint16_t *)malloc(s_camParams.width * s_camParams.height * sizeof(uint16_t));
        m_pStencilImage = (uint8_t *)malloc(s_camParams.width * s_camParams.height * sizeof(uint8_t));
        m_pOcclusionImage = (uint8_t *)malloc(s_camParams.width * s_camParams.height * sizeof(uint8_t));
        m_pUnusedStencilImage = (uint8_t *)malloc(s_camParams.width * s_camParams.height * sizeof(uint8_t));
        
        //RGB Image needs 3 bytes per value
        m_stencilSegLength = s_camParams.width * s_camParams.height * 3 * sizeof(uint8_t);
        m_instanceSegLength = s_camParams.width * s_camParams.height * sizeof(uint32_t);
        m_instanceSegImgLength = s_camParams.width * s_camParams.height * 3 * sizeof(uint8_t);

        m_pStencilSeg = (uint8_t *)malloc(m_stencilSegLength);
        m_pInstanceSeg = (uint32_t *)malloc(m_instanceSegLength);
        m_pInstanceSegImg = (uint8_t *)malloc(m_instanceSegImgLength);
        m_pGroundPointsImage = (uint8_t *)malloc(s_camParams.width * s_camParams.height * FLOATS_PER_POINT * sizeof(uint8_t));
    }
}

void ObjectDetection::collectLiDAR() {
    m_entitiesHit.clear();
    lidar.updateCurrentPosition(m_camForwardVector, m_camRightVector, m_camUpVector);
    float * pointCloud = lidar.GetPointClouds(pointCloudSize, &m_entitiesHit, lidar_param, m_pDepth, m_pInstanceSeg, m_vehicle);

    std::ofstream ofile(m_veloFilename, std::ios::binary);
    ofile.write((char*)pointCloud, FLOATS_PER_POINT * sizeof(float)*pointCloudSize);
    ofile.close();

    if (OUTPUT_RAYCAST_POINTS) {
        int pointCloudSize2;
        float* pointCloud2 = lidar.GetRaycastPointcloud(pointCloudSize2);

        std::string filename2 = getStandardFilename("velodyneRaycast", ".bin");
        std::ofstream ofile2(filename2, std::ios::binary);
        ofile2.write((char*)pointCloud2, FLOATS_PER_POINT * sizeof(float)*pointCloudSize2);
        ofile2.close();
    }

    if (GENERATE_2D_POINTMAP) {
        //Used for obtaining the 2D points for sampling depth map to convert to velodyne pointcloud
        int size;
        float * points2D = lidar.Get2DPoints(size);

        std::string filename = getStandardFilename("2dpoints", ".bin");
        std::ofstream ofile2(filename, std::ios::binary);
        ofile2.write((char*)points2D, 2 * sizeof(float) * size);
        ofile2.close();

        //Prints out the real values for a sample of the 
        filename = getStandardFilename("2dpoints", ".txt");
        FILE* f = fopen(filename.c_str(), "w");
        fclose(f);
        f = fopen(filename.c_str(), "a");
        int i = 0;
        std::ostringstream oss;
        while (i < 100) {
            oss << "num: " << i << " x: " << points2D[2 * i] << " y: " << points2D[2 * i + 1] << "\n";
            ++i;
        }
        i = (size / 2);
        int maxPrint = i + 100;
        while (i < maxPrint) {
            oss << "num: " << i << " x: " << points2D[2 * i] << " y: " << points2D[2 * i + 1] << "\n";
            ++i;
        }
        i = size - 100;
        while (i < size) {
            oss << "num: " << i << " x: " << points2D[2 * i] << " y: " << points2D[2 * i + 1] << "\n";
            ++i;
        }
        std::string str = oss.str();
        fprintf(f, str.c_str());
        fclose(f);
    }

    if (OUTPUT_DEPTH_STATS) {
        lidar.printDepthStats();
    }
}

//Maybe todo. Need to see results of other detection first
void ObjectDetection::getContours() {
    /*cv::Mat tempMat(cv::Size(s_camParams.width, s_camParams.height), CV_8UC1, m_pStencil);
    std::vector<std::vector<cv::Point> > contours;
    cv::Mat output;
    im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE);*/

    //Print separate segmentation images using drawContours
}

void ObjectDetection::setStencilBuffer() {
    log("About to set stencil buffer");
    int size = s_camParams.width * s_camParams.height;

    std::ofstream ofile(m_stencilFilename, std::ios::binary);
    ofile.write((char*)m_pStencil, size);
    ofile.close();

    std::vector<int> stencilValues;
    log("After writing stencil buffer");
    for (int j = 0; j < s_camParams.height; ++j) {
        for (int i = 0; i < s_camParams.width; ++i) {
            uint8_t val = m_pStencil[j * s_camParams.width + i];
            uint8_t* p = m_pStencilImage + (j * s_camParams.width) + i;

            if (val == 2) {
                *p = 255; //Vehicles
            }
            else if (val == 1) {
                *p = 128; //NPCs
            }
            else {
                *p = val;
            }

            if (OUTPUT_SEPARATE_STENCILS) {
                bool newValue = true;
                if (ONLY_OUTPUT_UNKNOWN_STENCILS && std::find(KNOWN_STENCIL_TYPES.begin(), KNOWN_STENCIL_TYPES.end(), val) != KNOWN_STENCIL_TYPES.end()) {
                    newValue = false;
                }
                if (std::find(stencilValues.begin(), stencilValues.end(), val) != stencilValues.end()) {
                    newValue = false;
                }
                if (newValue) {
                    stencilValues.push_back(val);
                }
            }
        }
    }

    log("Before saving stencil image");
    std::vector<std::uint8_t> ImageBuffer;
    lodepng::encode(ImageBuffer, (unsigned char*)m_pStencilImage, s_camParams.width, s_camParams.height, LCT_GREY, 8);
    lodepng::save_file(ImageBuffer, m_stencilImgFilename);
    log("After saving stencil image");

    if (OUTPUT_SEPARATE_STENCILS) {
        for (int s : stencilValues) {
            for (int j = 0; j < s_camParams.height; ++j) {
                for (int i = 0; i < s_camParams.width; ++i) {
                    uint8_t val = m_pStencil[j * s_camParams.width + i];
                    uint8_t* p = m_pStencilImage + (j * s_camParams.width) + i;
                    if (val == s) {
                        *p = 255; //Stencil value
                    }
                    else {
                        *p = 0;
                    }
                }
            }

            std::string filename = baseFolder + "stencilImage" + "\\";
            CreateDirectory(filename.c_str(), NULL);
            filename.append("\\");
            filename.append(instance_string);
            filename.append("-");
            filename.append(std::to_string(s));
            filename.append(".png");
            std::vector<std::uint8_t> ImageBuffer;
            lodepng::encode(ImageBuffer, (unsigned char*)m_pStencilImage, s_camParams.width, s_camParams.height, LCT_GREY, 8);
            lodepng::save_file(ImageBuffer, filename);

            if (ONLY_OUTPUT_UNKNOWN_STENCILS) {
                std::ostringstream oss;
                oss << "***************************************unknown stencil type: " << s << " at index: " << instance_index;
                log(oss.str(), true);
            }
        }
    }

    if (ONLY_OUTPUT_UNKNOWN_STENCILS) {
        std::ostringstream oss;
        oss << "Stencil types: ";
        for (int s : KNOWN_STENCIL_TYPES) {
            oss << s << ", ";
        }
        log(oss.str(), true);
    }
}

void ObjectDetection::setDepthBuffer(bool prevDepth) {
    int size = s_camParams.width * s_camParams.height;
    log("About to set depth buffer");

    std::string depthPCFilename = m_depthPCFilename;
    if (prevDepth) {
        float * pointCloud = lidar.UpdatePointCloud(pointCloudSize, m_pDepth);
        std::ofstream ofile1(m_veloFilenameU, std::ios::binary);
        ofile1.write((char*)pointCloud, FLOATS_PER_POINT * sizeof(float) * pointCloudSize);
        ofile1.close();
        depthPCFilename = m_depthPCFilenameU;
    }

    std::ofstream ofile(m_depthFilename, std::ios::binary);
    ofile.write((char*)m_pDepth, size * sizeof(float));
    ofile.close();

    int nonzero = 0;
    if (OUTPUT_DM_POINTCLOUD || OUTPUT_GROUND_PIXELS) {
        int pointCount = 0;
        float maxDepth = 0;
        float minDepth = 1;
        for (int j = 0; j < s_camParams.height; ++j) {
            for (int i = 0; i < s_camParams.width; ++i) {
                float ndc = m_pDepth[j * s_camParams.width + i];
                if (ndc != 0) {
                    ++nonzero;
                }
                Vector3 relPos = depthToCamCoords(ndc, i, j);

                if (OUTPUT_GROUND_PIXELS) {
                    int s = m_pStencil[j * s_camParams.width + i];
                    bool groundPoint;
                    if (s == STENCIL_TYPE_SKY || s == STENCIL_TYPE_NPC || s == STENCIL_TYPE_OWNCAR ||
                        s == STENCIL_TYPE_VEGETATION || s == STENCIL_TYPE_VEHICLE || s == STENCIL_TYPE_SELF) {
                        groundPoint = false;
                    }
                    else {
                        Vector3 worldPos = camToWorld(relPos, m_camForwardVector, m_camRightVector, m_camUpVector);
                        float groundZ;
                        //Note should always do +2 to ensure it hits the proper ground point
                        GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(worldPos.x, worldPos.y, worldPos.z + 2, &(groundZ), 0);
                        groundPoint = (worldPos.z - groundZ) < GROUND_POINT_MAX_DIST;
                    }
                    uint8_t pointVal = groundPoint ? 255 : 0;
                    m_pGroundPointsImage[j * s_camParams.width + i] = pointVal;
                }

                if (OUTPUT_DM_POINTCLOUD) {
                    float distance = sqrt(SYSTEM::VDIST2(0, 0, 0, relPos.x, relPos.y, relPos.z));
                    if ((OUTPUT_FULL_DM_POINTCLOUD || distance <= MAX_LIDAR_DIST) && distance >= s_camParams.nearClip) {
                        float* p = m_pDMPointClouds + (pointCount * 4);
                        *p = relPos.y;
                        *(p + 1) = -relPos.x;
                        *(p + 2) = relPos.z;
                        *(p + 3) = 0;
                        pointCount++;
                    }

                    uint16_t* p = m_pDMImage + (j * s_camParams.width) + i;
                    float distClipped = 1 - std::min<float>(1.0f, (distance - s_camParams.nearClip) / (s_camParams.farClip - s_camParams.nearClip));
                    uint16_t num = (uint16_t)floor(distClipped * 65535);
                    uint16_t swapped = (num >> 8) | (num << 8);
                    *p = swapped;
                }
            }
        }

        if (OUTPUT_GROUND_PIXELS) {
            std::ofstream ofile1(depthPCFilename, std::ios::binary);
            ofile1.write((char*)m_pDMPointClouds, FLOATS_PER_POINT * sizeof(float) * pointCount);
            ofile1.close();

            std::vector<std::uint8_t> ImageBuffer;
            lodepng::encode(ImageBuffer, (unsigned char*)m_pGroundPointsImage, s_camParams.width, s_camParams.height, LCT_GREY, 8);
            lodepng::save_file(ImageBuffer, m_groundPointsFilename);
        }
        
        if (OUTPUT_DM_POINTCLOUD) {
            std::ostringstream oss;
            oss << "Min depth: " << minDepth << " max: " << maxDepth << " pointCount: " << pointCount <<
                " height: " << s_camParams.height << " width: " << s_camParams.width << " size: " << size <<
                "\n Nonzero depth values: " << nonzero;
            std::string str = oss.str();
            log(str);

            std::ofstream ofile1(depthPCFilename, std::ios::binary);
            ofile1.write((char*)m_pDMPointClouds, FLOATS_PER_POINT * sizeof(float) * pointCount);
            ofile1.close();

            std::vector<std::uint8_t> ImageBuffer;
            lodepng::encode(ImageBuffer, (unsigned char*)m_pDMImage, s_camParams.width, s_camParams.height, LCT_GREY, 16);
            lodepng::save_file(ImageBuffer, m_depthImgFilename);

            log("After saving DM pointcloud");
        }
    }
}

//ndc is Normalized Device Coordinates which is value received from depth buffer
Vector3 ObjectDetection::depthToCamCoords(float ndc, float screenX, float screenY) {
    float normScreenX = 2 * screenX / float(s_camParams.width - 1) - 1.0f;
    float normScreenY = 2 * screenY / float(s_camParams.height - 1) - 1.0f;

    float ncX = normScreenX * s_camParams.ncWidth / 2;
    float ncY = normScreenY * s_camParams.ncHeight / 2;

    //Distance to near clip (hypotenus)
    float d2nc = sqrt(s_camParams.nearClip * s_camParams.nearClip + ncX * ncX + ncY * ncY);
    float depth = d2nc / ndc;
    if (ndc <= 0 || depth > s_camParams.farClip) {
        depth = s_camParams.farClip;
    }

    float depthDivisor = (s_camParams.nearClip * depth) / (2 * s_camParams.farClip);
    depth = depth / (1 + depthDivisor);

    //X is right, Y is forward, Z is up (GTA coordinate frame)
    Vector3 unitVec;
    unitVec.x = ncX / d2nc;
    unitVec.y = s_camParams.nearClip / d2nc;
    unitVec.z = -ncY / d2nc;

    Vector3 relPos;
    relPos.x = unitVec.x * depth;
    relPos.y = unitVec.y * depth;
    relPos.z = unitVec.z * depth;

    return relPos;
}

void ObjectDetection::increaseIndex() {
    if (pointclouds) {
        if (lidar_initialized) {
            ++instance_index;
        }
    }
    else {
        ++instance_index;
    }
    if (collectTracking && instance_index == trSeriesLength) {
        instance_index = 0;
        ++series_index;
        trSeriesGap = true;
        trackFirstFrame.clear();

        //update the string
        char temp[] = "%04d";
        char strComp[sizeof temp + 100];
        sprintf(strComp, temp, series_index);
        series_string = strComp;
    }

    char temp[] = "%06d";
    char strComp[sizeof temp + 100];
    sprintf(strComp, temp, instance_index);
    instance_string = strComp;
}

void ObjectDetection::setIndex() {
    m_curFrame.instanceIdx = instance_index;
    m_curFrame.seriesIdx = series_index;
}

//Camera intrinsics are focal length, and center in horizontal (x) and vertical (y)
void ObjectDetection::calcCameraIntrinsics() {
    float f = s_camParams.width / (2 * tan(HOR_CAM_FOV * PI / 360));
    float cx = s_camParams.width / 2;
    float cy = s_camParams.height / 2;

    intrinsics[0] = f;
    intrinsics[1] = cx;
    intrinsics[2] = cy;

    if (DEBUG_LOGGING) {
        std::ostringstream oss;
        oss << "Focal length is: " << f;
        std::string str = oss.str();
        log(str);
    }

    m_curFrame.focalLen = f;
}

//Camera intrinsics are focal length, and center in horizontal (x) and vertical (y)
void ObjectDetection::setFocalLength() {
    m_curFrame.focalLen = intrinsics[0];
}

std::string ObjectDetection::getStandardFilename(std::string subDir, std::string extension) {
    std::string filename = baseFolder;
    CreateDirectory(filename.c_str(), NULL);

    if (m_vPerspective != -1) {
        char temp[] = "%07d";
        char strComp[sizeof temp + 100];
        sprintf(strComp, temp, m_vPerspective);
        std::string entityStr = strComp;

        filename.append("alt_perspective");
        filename.append("\\");
        CreateDirectory(filename.c_str(), NULL);
        filename.append(entityStr);
        filename.append("\\");
        CreateDirectory(filename.c_str(), NULL);
    }

    filename.append(subDir);
    filename.append("\\");
    CreateDirectory(filename.c_str(), NULL);
    if (collectTracking) {
        filename.append(series_string);
        CreateDirectory(filename.c_str(), NULL);
    }
    filename.append("\\");
    filename.append(instance_string);
    filename.append(extension);
    return filename;
}

void ObjectDetection::outputRealSpeed() {
    //Print to file after every complete series
    if (m_trackLastSeqIndex != series_index) {
        m_trackDistErrorTotal /= m_trackDistErrorTotalCount;
        m_trackDistErrorTotalVar /= m_trackDistErrorTotalCount;
        m_trackDistErrorTotalVar = sqrt(m_trackDistErrorTotalVar);
        float avgSpeed = m_trackRealSpeed / m_trackDistErrorTotalCount;
        float avgDist = m_trackDist / m_trackDistErrorTotalCount;

        FILE* f = fopen(m_timeTrackFile.c_str(), "a");
        std::ostringstream oss;
        oss << m_trackDistErrorTotal << " " << m_trackDistErrorTotalVar << " " << avgSpeed << " " << avgDist;
        std::string str = oss.str();
        fprintf(f, str.c_str());
        fprintf(f, "\n");
        fclose(f);

        //Reset for every sequence
        m_trackRealSpeed = 0;
        m_trackDist = 0;
        m_trackDistErrorTotal = 0;
        m_trackDistErrorTotalVar = 0;
        m_trackDistErrorTotalCount = 0;
        m_trackLastSeqIndex = series_index;
    }

    //Initialize for every sequence (after printing)
    if (instance_index == 0) {
        m_trackLastPos = s_camParams.pos;
        m_trackLastIndex = instance_index;
        m_trackLastRealSpeed = ENTITY::GET_ENTITY_SPEED(m_vehicle) / 10;
        return;
    }

    //Do not count if we are in the gap between sequences
    if (m_trackLastIndex == instance_index) {
        return;
    }

    //Intermediate results
    if (instance_index % 10 == 0) {
        if (instance_index != 0) {
            std::ostringstream oss;
            float avgSpeed = m_trackRealSpeed * 10 / m_trackDistErrorTotalCount;
            float avgDist = m_trackDist * 10 / m_trackDistErrorTotalCount;
            oss << "Speed: " << avgSpeed << " dist: " << avgDist;
            std::string str = oss.str();
            log(str);
        }
    }

    //Update values
    //Average of speed at last frame and current frame
    m_trackRealSpeed += (ENTITY::GET_ENTITY_SPEED(m_vehicle) / 10 + m_trackLastRealSpeed) / 2;
    m_trackDist += sqrt(SYSTEM::VDIST2(s_camParams.pos.x, s_camParams.pos.y, s_camParams.pos.z, m_trackLastPos.x, m_trackLastPos.y, m_trackLastPos.z));
    m_trackDistErrorTotal += m_trackDist - m_trackRealSpeed;
    m_trackDistErrorTotalVar += pow((m_trackDist - m_trackRealSpeed), 2);
    m_trackDistErrorTotalCount++;

    m_trackLastPos = s_camParams.pos;
    m_trackLastIndex = instance_index;
    m_trackLastRealSpeed = ENTITY::GET_ENTITY_SPEED(m_vehicle) / 10;
}

void ObjectDetection::setCamParams(float* forwardVec, float* rightVec, float* upVec) {
    //These values stay the same throughout a collection period
    if (!s_camParams.init) {
        s_camParams.nearClip = 0.15;// CAM::_0xD0082607100D7193(); //CAM::GET_CAM_NEAR_CLIP(camera);
        s_camParams.farClip = 10001.5;// 800;// CAM::_0xDFC8CBC606FDB0FC(); //CAM::GET_CAM_FAR_CLIP(camera);
        s_camParams.fov = 59;// CAM::GET_GAMEPLAY_CAM_FOV();//CAM::GET_CAM_FOV(camera);
        s_camParams.ncHeight = 2 * s_camParams.nearClip * tan(s_camParams.fov / 2. * (PI / 180.)); // field of view is returned vertically
        s_camParams.ncWidth = s_camParams.ncHeight * GRAPHICS::_GET_SCREEN_ASPECT_RATIO(false);
        s_camParams.init = true;
    }

    ENTITY::GET_ENTITY_MATRIX(m_vehicle, &m_camForwardVector, &m_camRightVector, &m_camUpVector, &s_camParams.pos);

    if (forwardVec) {
        m_camForwardVector.x = forwardVec[0];
        m_camForwardVector.y = forwardVec[1];
        m_camForwardVector.z = forwardVec[2];

        if (rightVec && upVec) {
            m_camRightVector.x = rightVec[0];
            m_camRightVector.y = rightVec[1];
            m_camRightVector.z = rightVec[2];
            m_camUpVector.x = upVec[0];
            m_camUpVector.y = upVec[1];
            m_camUpVector.z = upVec[2];
        }
    }
    else {
        ENTITY::GET_ENTITY_MATRIX(m_vehicle, &m_camForwardVector, &m_camRightVector, &m_camUpVector, &currentPos);
    }

    //These values change frame to frame
    //Camera functions do not work in eve. Need to use vehicle and offsets.
    //Recordings need to always have the camera aligned with the vehicle for export to be aligned properly.
    if (!m_eve) {
        s_camParams.theta = ENTITY::GET_ENTITY_ROTATION(m_vehicle, 0); //CAM::GET_GAMEPLAY_CAM_ROT(0); //CAM::GET_CAM_ROT(camera, 0);
    }
    //s_camParams.pos = currentPos;// CAM::GET_GAMEPLAY_CAM_COORD();// CAM::GET_CAM_COORD(camera);
    //Use vehicleForwardVector since it corresponds to vehicle forwardVector
    s_camParams.pos.x = s_camParams.pos.x + CAM_OFFSET_FORWARD * vehicleForwardVector.x + CAM_OFFSET_UP * vehicleUpVector.x;
    s_camParams.pos.y = s_camParams.pos.y + CAM_OFFSET_FORWARD * vehicleForwardVector.y + CAM_OFFSET_UP * vehicleUpVector.y;
    s_camParams.pos.z = s_camParams.pos.z + CAM_OFFSET_FORWARD * vehicleForwardVector.z + CAM_OFFSET_UP * vehicleUpVector.z;

    Vector3 theta = CAM::GET_CAM_ROT(camera, 0);
    Vector3 pos1 = CAM::GET_CAM_COORD(camera);
    Vector3 rotation = ENTITY::GET_ENTITY_ROTATION(m_vehicle, 0);

    std::ostringstream oss1;
    oss1 << "\ns_camParams.pos X: " << s_camParams.pos.x << " Y: " << s_camParams.pos.y << " Z: " << s_camParams.pos.z <<
        "\nvehicle.pos X: " << currentPos.x << " Y: " << currentPos.y << " Z: " << currentPos.z <<
        "\npos1 - rendering cam X: " << pos1.x << " Y: " << pos1.y << " Z: " << pos1.z <<
        "\nfar: " << s_camParams.farClip << " nearClip: " << s_camParams.nearClip << " fov: " << s_camParams.fov <<
        "\nrotation gameplay: " << s_camParams.theta.x << " Y: " << s_camParams.theta.y << " Z: " << s_camParams.theta.z <<
        "\nrotation rendering: " << theta.x << " Y: " << theta.y << " Z: " << theta.z <<
        "\nrotation vehicle: " << rotation.x << " Y: " << rotation.y << " Z: " << rotation.z <<
        "\n AspectRatio: " << GRAPHICS::_GET_SCREEN_ASPECT_RATIO(false) <<
        "\nforwardVector: " << vehicleForwardVector.x << " Y: " << vehicleForwardVector.y << " Z: " << vehicleForwardVector.z;
    std::string str1 = oss1.str();
    log(str1);
}

void ObjectDetection::printSegImage() {
    int notUsedStencilPoints = 0;

    for (int j = 0; j < s_camParams.height; ++j) {
        for (int i = 0; i < s_camParams.width; ++i) {
            uint8_t stencilVal = m_pStencil[j * s_camParams.width + i];
            if (stencilVal == STENCIL_TYPE_NPC || stencilVal == STENCIL_TYPE_VEHICLE) {
                if (m_pStencilSeg[3 * (j * s_camParams.width + i)] == 0 &&
                    m_pStencilSeg[3 * (j * s_camParams.width + i) + 1] == 0 &&
                    m_pStencilSeg[3 * (j * s_camParams.width + i) + 2] == 0) {
                    m_pUnusedStencilImage[j * s_camParams.width + i] = 255;
                    ++notUsedStencilPoints;
                }
            }
        }
    }
    if (notUsedStencilPoints > 10000) {
        std::ostringstream oss;
        oss << "***Lots of unused stencil points, total unused: " << notUsedStencilPoints << " instance idx: " << instance_index;
        if (collectTracking) {
            oss << " seq idx: " << series_index;
        }
        std::string str = oss.str();
        log(str, true);
    }
    FILE* f = fopen(m_usedPixelFile.c_str(), "a");
    std::ostringstream oss;
    oss << notUsedStencilPoints << " " << instance_index;
    if (collectTracking) {
        oss << " " << series_index;
    }
    std::string str = oss.str();
    fprintf(f, str.c_str());
    fprintf(f, "\n");
    fclose(f);

    std::vector<std::uint8_t> ImageBuffer;
    lodepng::encode(ImageBuffer, (unsigned char*)m_pStencilSeg, s_camParams.width, s_camParams.height, LCT_RGB, 8);
    lodepng::save_file(ImageBuffer, m_segImgFilename);

    //Loop through coarse instance points, if there's points that only belong to one instance then put that value in
    for (auto point : m_coarseInstancePoints) {
        int idx = point.first;
        std::vector<int> entities = point.second;

        if (m_pInstanceSeg[idx] == 0 && entities.size() == 1) {
            m_pInstanceSeg[idx] = entities[0];
        }
    }

    //Print instance segmented image
    cv::Mat tempMat(cv::Size(s_camParams.width, s_camParams.height), CV_32SC1, m_pInstanceSeg);
    imwrite(m_instSegFilename, tempMat);


    //Create and print out instance seg image in colour for visualization
    for (int j = 0; j < s_camParams.height; ++j) {
        for (int i = 0; i < s_camParams.width; ++i) {
            //RGB image is 3 bytes per pixel
            int idx = j * s_camParams.width + i;
            int segIdx = 3 * idx;
            int entityID = m_pInstanceSeg[idx];

            int newVal = 47 * entityID; //Just to produce unique but different colours
            int red = (newVal + 13 * entityID) % 255;
            int green = (newVal / 255) % 255;
            int blue = newVal % 255;
            uint8_t* p = m_pInstanceSegImg + segIdx;
            *p = red;
            *(p + 1) = green;
            *(p + 2) = blue;
        }
    }

    cv::Mat colorImg(cv::Size(s_camParams.width, s_camParams.height), CV_8UC3, m_pInstanceSegImg);
    log("About to print seg image3", true);
    imwrite(m_instSegImgFilename, colorImg);

    //Clear all maps and seg image arrays
    memset(m_pStencilSeg, 0, m_stencilSegLength);
    memset(m_pInstanceSeg, 0, m_instanceSegLength);
    memset(m_pInstanceSegImg, 0, m_instanceSegImgLength);
    m_coarseInstancePoints.clear();
    m_duplicateBoxPoints.clear();
}

void ObjectDetection::initVehicleLookup() {
    if (!m_vLookupInit) {
        std::string translationFile = std::string(getenv("DEEPGTAV_DIR")) + "\\ObjectDet\\vehicle_labels.csv";
        std::ifstream inFile(translationFile);
        std::string line;
        log("Translation file:", true);
        log(translationFile, true);
        while (std::getline(inFile, line)) // read whole line into line
        {
            std::istringstream iss(line); // string stream
            std::string model;
            std::string vehicleType;
            std::getline(iss, model, ','); // read first part up to comma, ignore the comma
            std::string before = model;
            model.erase(remove_if(model.begin(), model.end(), [](char c) { return !isalpha(c); }), model.end());
            iss >> vehicleType; // read the second part
            m_vLookup.insert(std::pair< std::string, std::string>(model, vehicleType));

            //Modelstrings seem to be missing last few letters on occasion (this should fix that)
            if (model.size() >= 2) {
                model = model.substr(0, model.size() - 1);
                m_vLookup.insert(std::pair< std::string, std::string>(model, vehicleType));
            }
            if (model.size() >= 2) {
                model = model.substr(0, model.size() - 1);
                m_vLookup.insert(std::pair< std::string, std::string>(model, vehicleType));
            }
            if (model.size() >= 2) {
                model = model.substr(0, model.size() - 1);
                m_vLookup.insert(std::pair< std::string, std::string>(model, vehicleType));
            }
        }
        m_vLookupInit = true;
    }
}

void ObjectDetection::outputOcclusion() {
    if (OUTPUT_OCCLUSION_IMAGE) {
        std::vector<std::uint8_t> ImageBuffer;
        lodepng::encode(ImageBuffer, (unsigned char*)m_pOcclusionImage, s_camParams.width, s_camParams.height, LCT_GREY, 8);
        lodepng::save_file(ImageBuffer, m_occImgFilename);
        memset(m_pOcclusionImage, 0, s_camParams.width * s_camParams.height);
    }
}

void ObjectDetection::outputUnusedStencilPixels() {
    if (OUTPUT_UNUSED_PIXELS_IMAGE) {
        std::vector<std::uint8_t> ImageBuffer;
        lodepng::encode(ImageBuffer, (unsigned char*)m_pUnusedStencilImage, s_camParams.width, s_camParams.height, LCT_GREY, 8);
        lodepng::save_file(ImageBuffer, m_unusedPixelsFilename);
        memset(m_pUnusedStencilImage, 0, s_camParams.width * s_camParams.height);
    }
}

void ObjectDetection::exportEntity(ObjEntity e, std::ostringstream& oss, bool unprocessed, bool augmented,
                                    bool checkbbox2d, const int &maxDist, const int &min2DPoints, const int &min3DPoints) {
    BBox2D b = e.bbox2d;
    if (unprocessed) b = e.bbox2dUnprocessed;

    if (checkbbox2d) {
        if ((int)b.left >= s_camParams.width || (int)b.right == 0 || (int)b.bottom == 0 || (int)b.top >= s_camParams.height) return;
        if ((int)b.left == (int)b.right || (int)b.top == (int)b.bottom) return;
    }

    if (maxDist != -1) {
        if (e.distance > float(maxDist)) return;

        //HAS_ENTITY_CLEAR_LOS_TO_ENTITY is from vehicle, NOT camera perspective
        //pointsHit misses some objects
        //hasLOSToEntity retrieves from camera perspective however 3D bboxes are larger than object
        if (ENTITY::IS_ENTITY_OCCLUDED(e.entityID) &&
            !ENTITY::HAS_ENTITY_CLEAR_LOS_TO_ENTITY(m_vehicle, e.entityID, 19) &&
            e.pointsHit3D <= 0) {
            log("Occluded and no points.", true);

            Vector3 upVector, rightVector, forwardVector, position; //Vehicle position
            ENTITY::GET_ENTITY_MATRIX(e.entityID, &forwardVector, &rightVector, &upVector, &position);
            if (!hasLOSToEntity(e.entityID, e.location, e.dim, forwardVector, rightVector, upVector)) {
                log("Occluded and no points2.", true);
                return;
            }
        }
    }
    if (min2DPoints != -1) {
        if (e.pointsHit2D < min2DPoints) {
            log("pointsHit2D failed.", true);
            return;
        }
    }
    //TODO OBTAIN_RAY_POINTS_HIT is off now.
    //3D points hit should be based off stencil seg with depth
    /*if (min3DPoints != -1) {
        if (e.pointsHit3D < min3DPoints) {
            log("pointsHit3D failed.", true);
            return;
        }
    }*/

    oss << e.objType << " " << e.truncation << " " << e.occlusion << " " << e.alpha << " " <<
        (int)b.left << " " << (int)b.top << " " <<
        (int)b.right << " " << (int)b.bottom << " " <<
        e.height << " " << e.width << " " << e.length << " " <<
        e.location.x << " " << e.location.y << " " << e.location.z << " " <<
        e.rotation_y;

    if (augmented) {
        oss << " " << e.entityID << " " << e.pointsHit2D << " " << e.pointsHit3D << " " << e.speed << " "
            << e.roll << " " << e.pitch << " " << e.modelString;
    }
    oss << "\n";
}

void ObjectDetection::exportEntities(EntityMap entMap, std::ostringstream& oss, bool unprocessed, bool augmented, bool checkbbox2d, const int &maxDist, const int &min2DPoints, const int &min3DPoints) {
    for (EntityMap::const_iterator it = entMap.begin(); it != entMap.end(); ++it)
    {
        ObjEntity entity = it->second;
        if (!entity.isPedInV) {
            exportEntity(entity, oss, unprocessed, augmented, checkbbox2d, maxDist, min2DPoints, min3DPoints);
        }
    }
}

void ObjectDetection::exportPosition() {
    FILE* f = fopen(m_posFilename.c_str(), "w");
    std::ostringstream oss;

    oss << m_curFrame.kittiWorldPos.x << " " << m_curFrame.kittiWorldPos.y << " " << m_curFrame.kittiWorldPos.z << "\n"
        << m_curFrame.camPos.x << " " << m_curFrame.camPos.y << " " << m_curFrame.camPos.z << "\n"
        << m_curFrame.forwardVec.x << " " << m_curFrame.forwardVec.y << " " << m_curFrame.forwardVec.z << "\n"
        << m_curFrame.rightVec.x << " " << m_curFrame.rightVec.y << " " << m_curFrame.rightVec.z << "\n"
        << m_curFrame.upVec.x << " " << m_curFrame.upVec.y << " " << m_curFrame.upVec.z;

    std::string str = oss.str();
    fprintf(f, str.c_str());
    fclose(f);
}

void ObjectDetection::exportEgoObject(ObjEntity vPerspective) {
    FILE* f = fopen(m_egoObjectFilename.c_str(), "w");
    std::ostringstream oss;

    exportEntity(vPerspective, oss, false, true, false);

    std::string str = oss.str();
    fprintf(f, str.c_str());
    fclose(f);
}

void ObjectDetection::exportCalib() {
    FILE* f = fopen(m_calibFilename.c_str(), "w");
    std::ostringstream oss;

    for (int i = 0; i <= 3; ++i) {
        oss << "P" << i << ": " <<
            m_curFrame.focalLen << " 0 " << (int)(s_camParams.width / 2) << " 0" <<
            " 0 " << m_curFrame.focalLen << " " << (int)(s_camParams.height / 2) << " 0" <<
            " 0 0 1 0\n";
    }
    oss << "R0_rect: 1 0 0 0 1 0 0 0 1\n" <<
        "Tr_velo_to_cam: 0 -1 0 0 0 0 -1 0 1 0 0 0\n" <<
        "Tr_imu_to_velo: 1 0 0 0 0 1 0 0 0 0 1 0";

    std::string str = oss.str();
    fprintf(f, str.c_str());
    fclose(f);
}

void ObjectDetection::exportDetections(FrameObjectInfo fObjInfo, ObjEntity* vPerspective) {
    if (collectTracking) {
        //TODO
    }
    FILE* f = fopen(m_labelsFilename.c_str(), "w");
    std::ostringstream oss;

    exportEntities(fObjInfo.vehicles, oss, false, false, true, OBJECT_MAX_DIST, 1, 1);
    exportEntities(fObjInfo.peds, oss, false, false, true, OBJECT_MAX_DIST, 1, 1);

    std::string str = oss.str();
    fprintf(f, str.c_str());
    fclose(f);

    f = fopen(m_labelsUnprocessedFilename.c_str(), "w");
    std::ostringstream oss1;

    exportEntities(fObjInfo.vehicles, oss1, true, false, OBJECT_MAX_DIST, 1, 1);
    exportEntities(fObjInfo.peds, oss1, true, false, OBJECT_MAX_DIST, 1, 1);

    std::string str1 = oss1.str();
    fprintf(f, str1.c_str());
    fclose(f);

    f = fopen(m_labelsAugFilename.c_str(), "w");
    std::ostringstream oss2;

    //Augmented files also exports objects at any distance, with no 3D or 2D points
    exportEntities(fObjInfo.vehicles, oss2, false, true, false);
    exportEntities(fObjInfo.peds, oss2, false, true, false);

    std::string str2 = oss2.str();
    fprintf(f, str2.c_str());
    fclose(f);

    exportCalib();
    exportPosition();

    if (!vPerspective) {
        exportEgoObject(m_ownVehicleObj);
    }
    else {
        exportEgoObject(*vPerspective);
    }
}

void ObjectDetection::exportImage(BYTE* data, std::string filename) {
    cv::Mat tempMat(cv::Size(s_camParams.width, s_camParams.height), CV_8UC3, data);
    if (filename.empty()) {
        filename = m_imgFilename;
    }
    cv::imwrite(filename, tempMat);
}

Vector3 ObjectDetection::getGroundPoint(Vector3 point, Vector3 yVectorCam, Vector3 xVectorCam, Vector3 zVectorCam) {

    //The closer to the ground the less vehicle/world z discrepancy there will be
    //point.z -= (CAM_OFFSET_UP + CAR_CENTER_OFFSET_UP);

    Vector3 worldpoint = convertCoordinateSystem(point, yVectorCam, xVectorCam, zVectorCam);

    //Transition from relative to world
    worldpoint.x += s_camParams.pos.x;
    worldpoint.y += s_camParams.pos.y;
    worldpoint.z += s_camParams.pos.z;

    //Obtain groundz at world position
    float groundZ;
    GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(worldpoint.x, worldpoint.y, worldpoint.z, &(groundZ), 0);
    worldpoint.z = groundZ;

    worldpoint.x -= s_camParams.pos.x;
    worldpoint.y -= s_camParams.pos.y;
    worldpoint.z -= s_camParams.pos.z;

    Vector3 relPoint = convertCoordinateSystem(worldpoint, m_camForwardVector, m_camRightVector, m_camUpVector);

    return relPoint;
}

void ObjectDetection::setGroundPlanePoints() {
    //Distance to ground is ~1.73
    //See CAR_CENTER_OFFSET_UP
    /*float groundZ;
    GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(s_camParams.pos.x, s_camParams.pos.y, s_camParams.pos.z, &(groundZ), 0);
    float groundDiff = s_camParams.pos.z - groundZ;
    std::ostringstream osst;
    osst << "Distance to ground: " << groundDiff;
    log(osst.str(), true);*/

    //Vector of directions to test ground plane
    //Forward, right (up should always be 0 since it will be tested with get_ground_z
    std::vector<Vector3> points;
    points.push_back(createVec3(-2.f, 5.0, 0.f));
    points.push_back(createVec3(2.f, 5.0, 0.f));
    points.push_back(createVec3(0.f, 15.0, 0.f));
    points.push_back(createVec3(3.f, 15.0, 0.f));
    points.push_back(createVec3(-3.f, 15.0, 0.f));
    points.push_back(createVec3(-2, 25.0, 0));
    points.push_back(createVec3(2, 25.0, 0));

    //World coordinate vectors
    Vector3 worldX; worldX.x = 1; worldX.y = 0; worldX.z = 0;
    Vector3 worldY; worldY.x = 0; worldY.y = 1; worldY.z = 0;
    Vector3 worldZ; worldZ.x = 0; worldZ.y = 0; worldZ.z = 1;
    Vector3 xVectorCam = convertCoordinateSystem(worldX, m_camForwardVector, m_camRightVector, m_camUpVector);
    Vector3 yVectorCam = convertCoordinateSystem(worldY, m_camForwardVector, m_camRightVector, m_camUpVector);
    Vector3 zVectorCam = convertCoordinateSystem(worldZ, m_camForwardVector, m_camRightVector, m_camUpVector);

    //World coordinates have: y north, x east
    std::string filename = getStandardFilename("ground_points", ".txt");

    FILE* f = fopen(filename.c_str(), "w");
    std::ostringstream oss;

    for (auto point : points) {
        Vector3 relPoint = getGroundPoint(point, yVectorCam, xVectorCam, zVectorCam);

        //Output kitti velodyne coords relative position with ground z
        oss << relPoint.y << ", " << relPoint.x << ", " << relPoint.z << "\n";
    }
    std::string str = oss.str();
    fprintf(f, str.c_str());
    fclose(f);

    //**********************Creating ground point grid****************************************
    //World coordinates have: y north, x east
    std::string filename2 = getStandardFilename("ground_points_grid", ".txt");
    FILE* f2 = fopen(filename2.c_str(), "w");
    std::ostringstream oss2;

    int pointInterval = 2; //Distance between ground points (approximately in metres - game coordinates)
    for (int x = -MAX_LIDAR_DIST; x <= MAX_LIDAR_DIST; x += pointInterval) {
        for (int y = 0; y <= MAX_LIDAR_DIST; y += pointInterval) {
            Vector3 point = createVec3(x, y, 0.0f);
            Vector3 relPoint = getGroundPoint(point, yVectorCam, xVectorCam, zVectorCam);

            //Output kitti velodyne coords relative position with ground z
            oss2 << relPoint.y << ", " << relPoint.x << ", " << relPoint.z << "\n";
        }
    }

    std::string str2 = oss2.str();
    fprintf(f2, str2.c_str());
    fclose(f2);
}

void ObjectDetection::getNearbyVehicles() {
    for (WorldObject v : m_worldVehicles) {
        std::ostringstream oss;
        std::string filename = baseFolder + "extraDetections" + "\\";
        CreateDirectory(filename.c_str(), NULL);
        filename.append(instance_string);
        CreateDirectory(filename.c_str(), NULL);
        filename.append("\\");
        char temp[] = "%06d";
        char strComp[sizeof temp + 100];
        sprintf(strComp, temp, v.e);
        filename.append(strComp);
        filename.append(".txt");

        Vector3 forwardVector, rightVector, upVector, position;
        ENTITY::GET_ENTITY_MATRIX(v.e, &forwardVector, &rightVector, &upVector, &position); //Blue or red pill
        float distance = sqrt(SYSTEM::VDIST2(s_camParams.pos.x, s_camParams.pos.y, s_camParams.pos.z, position.x, position.y, position.z));
        if (distance < 200) {
            Vector3 min, max;
            Vector3 dim = getVehicleDims(v.e, v.model, min, max);

            SubsetInfo s = getObjectInfoSubset(position, forwardVector, dim);
            oss << v.type << " " << s.alpha_kitti << " " <<
                s.kittiHeight << " " << s.kittiWidth << " " << s.kittiLength << " " <<
                s.kittiPos.x << " " << s.kittiPos.y << " " << s.kittiPos.z << " " <<
                s.rot_y << "\n";

            for (WorldObject vh : m_worldVehicles) {
                checkEntity(v.e, vh, position, oss);
            }
            for (WorldObject ped : m_worldPeds) {
                checkEntity(v.e, ped, position, oss);
            }

            FILE* f = fopen(filename.c_str(), "w");
            std::string str = oss.str();
            fprintf(f, str.c_str());
            fclose(f);
        }
    }
    m_worldVehicles.clear();
    m_worldPeds.clear();
}

Vector3 ObjectDetection::getVehicleDims(Entity e, Hash model, Vector3 &min, Vector3 &max) {
    GAMEPLAY::GET_MODEL_DIMENSIONS(model, &min, &max);

    //Calculate size
    Vector3 dim;
    dim.x = 0.5*(max.x - min.x);
    dim.y = 0.5*(max.y - min.y);
    dim.z = 0.5*(max.z - min.z);

    return dim;
}

//Entity p is the vehicle which is perceiving
void ObjectDetection::checkEntity(Vehicle p, WorldObject e, Vector3 pPos, std::ostringstream& oss) {
    if (p != e.e) {
        Vector3 forwardVector, rightVector, upVector, position;
        ENTITY::GET_ENTITY_MATRIX(e.e, &forwardVector, &rightVector, &upVector, &position); //Blue or red pill
        float distance = sqrt(SYSTEM::VDIST2(pPos.x, pPos.y, pPos.z, position.x, position.y, position.z));
        if (distance < 120) {
            bool losToCentre = ENTITY::HAS_ENTITY_CLEAR_LOS_TO_ENTITY(p, e.e, 19);

            Vector3 min, max;
            Vector3 dim = getVehicleDims(e.e, e.model, min, max);

            bool hasLOS = hasLOSToEntity(e.e, position, dim, forwardVector, rightVector, upVector, true, pPos);

            if (hasLOS || losToCentre) {
                SubsetInfo s = getObjectInfoSubset(position, forwardVector, dim);

                oss << e.type << " " << s.alpha_kitti << " " <<
                    s.kittiHeight << " " << s.kittiWidth << " " << s.kittiLength << " " <<
                    s.kittiPos.x << " " << s.kittiPos.y << " " << s.kittiPos.z << " " <<
                    s.rot_y << " " << distance << "\n";
            }
        }
    }
}

SubsetInfo ObjectDetection::getObjectInfoSubset(Vector3 position, Vector3 forwardVector, Vector3 dim) {
    SubsetInfo s;

    Vector3 relativePos;
    relativePos.x = position.x - s_camParams.pos.x;
    relativePos.y = position.y - s_camParams.pos.y;
    relativePos.z = position.z - s_camParams.pos.z;

    Vector3 kittiForwardVector = convertCoordinateSystem(forwardVector, m_camForwardVector, m_camRightVector, m_camUpVector);
    s.rot_y = -atan2(kittiForwardVector.y, kittiForwardVector.x);

    relativePos = convertCoordinateSystem(relativePos, m_camForwardVector, m_camRightVector, m_camUpVector);

    //Convert to KITTI camera coordinates
    s.kittiPos.x = relativePos.x;
    s.kittiPos.y = -relativePos.z;
    s.kittiPos.z = relativePos.y;

    //Kitti dimensions
    s.kittiHeight = 2 * dim.z;
    s.kittiWidth = 2 * dim.x;
    s.kittiLength = 2 * dim.y;

    //alpha is rot_y + tan^-1(z/x) + PI/2
    s.beta_kitti = atan2(s.kittiPos.z, s.kittiPos.x);
    s.alpha_kitti = s.rot_y + s.beta_kitti - PI / 2;

    return s;
}