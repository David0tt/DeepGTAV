#include "lib/script.h"
#pragma once

// Converts a vector 'vec' into the coordinate system with the specified unit vectors
static Vector3 convertCoordinateSystem(Vector3 vec, Vector3 forwardVector, Vector3 rightVector, Vector3 upVector) {
    Vector3 newVec;

    newVec.x = vec.x*rightVector.x + vec.y*rightVector.y + vec.z*rightVector.z;
    newVec.y = vec.x*forwardVector.x + vec.y*forwardVector.y + vec.z*forwardVector.z;
    newVec.z = vec.x*upVector.x + vec.y*upVector.y + vec.z*upVector.z;

    return newVec;
}

typedef struct BBox2D {
    float left;
    float top;
    float right;
    float bottom;

    float width() {
        return right - left;
    }

    float height() {
        return bottom - top;
    }

    float posX() {
        return left + width() / 2;
    }

    float posY() {
        return top + height() / 2;
    }
};

typedef struct VehicleToCreate {
    std::string model;
    float forward;
    float right;
    float heading;
    int color;
    int color2;

    VehicleToCreate(std::string _model, float _forward, float _right, float _heading,
        int _color, int _color2) :
        model(_model),
        forward(_forward),
        right(_right),
        heading(_heading),
        color(_color),
        color2(_color2)
    {
    }

    VehicleToCreate(){
    }
};

typedef struct PedToCreate {
    int model;
    float forward;
    float right;
    float heading;
};

static Vector3 subtractVector(Vector3 first, Vector3 subtract) {
    Vector3 diff;
    diff.x = first.x - subtract.x;
    diff.y = first.y - subtract.y;
    diff.z = first.z - subtract.z;
    return diff;
}