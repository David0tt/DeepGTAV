//File for camera parameters
#pragma once

struct CamParams {
    bool firstInit = false;
    bool init; //initialized
    int width;
    int height;
    float nearClip;
    float farClip;
    float fov;

    //near clip width/height
    float ncHeight;
    float ncWidth;

    //These values change between frames
    Vector3 pos; //In world coordinates
    Vector3 theta; //Camera rotation in world coordinates
};

//Global variable to be used by scenario and LiDAR
//Ensures the calculation for camera parameters is the same across files
extern CamParams s_camParams;