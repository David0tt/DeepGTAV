/*

Original source code taken from:
https://github.com/gdpinchina/A-virtual-LiDAR-for-DeepGTAV

*/

#pragma once
#include "lib/script.h"
#include <unordered_map>
#include <Eigen/Core>

#define _LIDAR_NOT_INIT_YET_ 0
#define _LIDAR_INIT_AS_2D_ 1
#define _LIDAR_INIT_AS_3D_ 2

const bool GENERATE_2D_POINTMAP = true;

const int MAX_POINTS = 5000000;

struct HitLidarEntity {
    int pointsHit;
    Vector3 forward;
    Vector3 position;
    float maxFront;
    float maxBack;

    HitLidarEntity(Vector3 _forward, Vector3 _position) {
        pointsHit = 1;
        forward = _forward;
        position = _position;
        maxFront = 0;
        maxBack = 0;
    }
};

class LiDAR
{
public:
    LiDAR();
    ~LiDAR();

public:
    //Note: polar-coordinate definition below: 
    //1. Horizontal angle ranges from 0 to 360 degree.
    //	[horizRiLimit, horizLeLimit), anti-clock, namely from rightside to leftside.
    //	horizRiLimit:[180, 360), horizLeLimit:[0, 180). 
    //	For example, shown as defaults in API Init2DLiDAR_SmplNum, right:[270, 360), left:[0, 90).
    //
    //2. Vertical angle ranges from 0 to 180 degree.
    //	[vertiUnLimit, vertiUpLimit), namely from underside to upside. 
    //	vertiUnLimit:[180, 90), vertiUpLimit:[90, 0).
    //	For example, shown as defaults in API Init3DLiDAR_SmplNum, downside:[135, 90), upside:[90, 45).
    //
    //3. One single frame contains :
    //	2D: HorizSmplNum ranges. 
    //	3D: VertiSmplNum * horizSmplNum ranges. 
    //
    //4. By defaults:
    //	Position: The LiDAR device is set to the same position of camera.


    void Init2DLiDAR_SmplNum(float maxRange = 100.0, int horizSmplNum = 180, float horizLeLimit = 90.0, float horizRiLimit = 270.0);

    void Init3DLiDAR_SmplNum(float maxRange = 100.0, int horizSmplNum = 180, float horizLeLimit = 90.0, float horizRiLimit = 270.0,
        int vertiSmplNum = 9, float vertiUpLimit = 45, float vertiUnLimit = 135.0);

    void Init2DLiDAR_FOV(float maxRange = 100.0, float horizFOV = 180.0, float horizAngResolu = 1.0);

    void Init3DLiDAR_FOV(float maxRange = 100.0, float horizFOV = 180.0, float horizAngResolu = 1.0, float vertiFOV = 90.0, float vertiAngResolu = 10.0, float vertiUpLimit = 2.0);

    void AttachLiDAR2Camera(Cam camera, Entity ownCar, int scrWidth, int scrHeight);

    void DestroyLiDAR();

    float* GetPointClouds(int &size, std::unordered_map<int, HitLidarEntity*> *entitiesHit, int param, float* depthMap);
    float* Get2DPoints(int &size);
    int getTotalSmplNum();
    int getVertiSmplNum();
    int getHorizSmplNum();
    int getCurType();

    void updateCurrentPosition(Vector3 currentForwardVector, Vector3 currentRightVector, Vector3 currentUpVector);


private:

    void GenerateSinglePoint(float phi, float theta, float *p);
    void GenerateHorizPointClouds(float phi, float *p);
    void calcDCM();


private:

    float* m_pPointClouds;
    int m_pointsHit;
    float m_maxRange;//meter
    float m_vertiUpLimit;//deg, the upside limit of zenith direction, namely the min vertical angle, 0 <= up <= phiUp < 90
    float m_vertiUnLimit;//deg, the underside limit of ground direction, namely the max vertical angle, 90 <= phiLo <= un <= 180
    float m_horizLeLimit;//deg, the left limit of horizontal direction, if no limits, set to 180, 0 <= thetaLe < le < 180
    float m_horizRiLimit;//deg, the right limit of horizontal direction, if no limits, set to 180, 180 <= ri <= thetaRi < 360
    int m_vertiSmplNum;
    int m_horizSmplNum;
    float m_vertiResolu;//deg, vertical angle resolution
    float m_horizResolu;//deg, horizontal angle resolution

    Cam m_camera;
    Entity m_ownCar;
    Vector3 m_curPos;
    float m_quaterion[4];
    float m_rotDCM[9];//convert n-coord to b-coord
    int m_initType;
    bool m_isAttach;

    Vector3 currentForwardVec;
    Vector3 currentUpVec;
    Vector3 currentRightVec;

    float m_max_dist;
    float m_min_dist;

    float* m_lidar2DPoints;
    int m_beamCount;

    std::unordered_map<int, HitLidarEntity*>* m_entitiesHit;
    int native_param = 7;

    //Depth map variables
    float * m_depthMap;
    int m_scrWidth;
    int m_scrHeight;
    float m_nearClip;
    float m_fov;
    float m_ncHeight;
    float m_ncWidth;
    Vector3 m_theta;
    Vector3 adjustEndCoord(Vector3 pos, Vector3 relPos);
    float depthFromNDC(int x, int y, float screenX = 0.0f, float screenY = 0.0f);
};