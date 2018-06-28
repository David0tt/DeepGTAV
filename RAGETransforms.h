#include "lib/script.h"
#include <Eigen/Core>
#include <Eigen/LU>
#include "Constants.h"

#pragma once

const bool LOG_RAGE = false;

//Rockstar Advanced Game Engine (RAGE) transformations between coordinate systems
static bool rage_initialized = false;
static Eigen::Matrix4f rage_CamToWorld;
static Eigen::Matrix4f rage_NDCToCam;
static Vector3 rage_CamPos;
static Vector3 rage_CamRot;

//Values constant for a session
static float rage_nc;
static float rage_fc;
static float rage_fov;
static int rage_width;
static int rage_height;

//Far clip Value calculated by Racinsky Matej which is far clip used in NDC calculation
static const float rage_FC = 10003.814;
static const float rage_NC = 1.500225;

static void rageInitialize(float near_clip, float far_clip, float fov, int width, int height) {
    rage_nc = near_clip;//Trying this out near_clip;
    rage_fc = far_clip;//rage_FC -> reverse engineered by racinmat
    rage_fov = fov * PI / 180;
    std::ostringstream oss;
    oss << "Field of View: " << fov << " Rage fov: " << rage_fov <<
        "\nNear Clip: " << near_clip << "Far clip: " << far_clip;
    std::string str = oss.str();
    log(str);
    rage_width = width;
    rage_height = height;
    rage_initialized = true;
}

//Call this function before starting to convert a depth map
//Input current camera position and rotation
static void rageNewDepthMap(Vector3 pos, Vector3 rot) {
    //Convert rotation to radians
    rot.x = rot.x * PI / 180;
    rot.y = rot.y * PI / 180;
    rot.z = rot.z * PI / 180;

    //World to Cam and inverse calculations
    Eigen::Matrix4f WorldToCamA;
    Eigen::Matrix4f WorldToCamB;
    Eigen::Matrix4f WorldToCamC;
    Eigen::Matrix4f WorldToCamD;
    Eigen::Matrix4f WorldToCam;
    WorldToCamA << 1, 0, 0, 0,
        0, sin(rot.x), cos(rot.x), 0,
        0, cos(rot.x), -sin(rot.x), 0,
        0, 0, 0, 1;

    WorldToCamB << cos(rot.y), 0, -sin(rot.y), 0,
        0, 1, 0, 0,
        sin(rot.y), 0, cos(rot.y), 0,
        0, 0, 0, 1;

    WorldToCamC << cos(rot.z), sin(rot.z), 0, 0,
        sin(rot.z), -cos(rot.z), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    WorldToCamD << 1, 0, 0, pos.x,
                   0, 1, 0, pos.y,
                   0, 0, 1, pos.z,
                   0, 0, 0, 1;

    WorldToCam = WorldToCamA * WorldToCamB * WorldToCamC * WorldToCamD;
    rage_CamToWorld = WorldToCam.inverse();

    //Cam to NDC and inverse calculations
    Eigen::Matrix4f CamToNDC;
    CamToNDC << rage_height / (rage_width*tan(rage_fov / 2)), 0, 0, 0,
        0, 1 / tan(rage_fov / 2), 0, 0,
        0, 0, -rage_fc / (rage_nc - rage_FC), (-rage_FC * rage_nc) / (rage_nc - rage_FC),
        0, 0, -1, 0;

    /*CamToNDC << 1.210067, 0, 0, -0.000004,
        0, 2.144507, 0, 0.000002,
        0, 0, 0.00015, 1.500225,
        0, 0, -1, 0;*/
    rage_NDCToCam = CamToNDC.inverse();

    std::ostringstream oss;
    oss << "Cam rotation: " << rot.x << ", " << rot.y << ", " << rot.z;
    oss << "\nCam position: " << pos.x << ", " << pos.y << ", " << pos.z;
    oss << "\nCos: " << cos(rot.x) << ", " << cos(rot.y) << ", " << cos(rot.z);
    oss << "\nsin: " << sin(rot.x) << ", " << sin(rot.y) << ", " << sin(rot.z);
    oss << "\nWorldToCam:\n" << WorldToCam << "\nCamToNDC:\n" << CamToNDC;
    oss << "\nCamToWorld:\n" << rage_CamToWorld << "\nNDCToCAm:\n" << rage_NDCToCam;
    std::string str = oss.str();
    log(str);
}

//Transforms from NDC (depth buffer) coordinates to GTA world coordinates
static Vector3 rageNDCToWorld(float depth, int x, int y) {
    Vector3 temp;
    if (!rage_initialized) {
        log("ERROR: RAGE not initialized!");
        temp.x = 0;
        temp.y = 0;
        temp.z = 0;
        return temp;
    }
    float x_ndc = (2 * (float)x / rage_width) - 1;
    float y_ndc = -((2 * (float)y / rage_height) - 1);

    Eigen::Vector4f ndc(x_ndc, y_ndc, depth, 1);

    Eigen::Vector4f camCoords = rage_NDCToCam * ndc;
    Eigen::Vector4f worldCoords = rage_CamToWorld * camCoords;

    if (LOG_RAGE) {
        std::ostringstream oss;
        oss << "Depth: " << depth;
        oss << "\nx/y NDC: " << x_ndc << ", " << y_ndc;
        oss << "\nCam coords: " << camCoords(0) << ", " << camCoords(1) << ", " << camCoords(2) << ", " << camCoords(3);
        oss << "\nWorld coords: " << worldCoords(0) << ", " << worldCoords(1) << ", " << worldCoords(2) << ", " << worldCoords(3);
        std::string str = oss.str();
        log(str);
    }

    temp.x = worldCoords(0);
    temp.y = worldCoords(1);
    temp.z = worldCoords(2);
    return temp;
}