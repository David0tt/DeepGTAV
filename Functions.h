#include "lib/script.h"
#pragma once

// Converts a vector 'vec' into the coordinate system with the specified unit vectors
static Vector3 convertCoordinateSystem(Vector3 vec, Vector3 forwardVector, Vector3 rightVector, Vector3 upVector) {
    Vector3 newVec;

    newVec.x = vec.x*rightVector.x + vec.y*forwardVector.x + vec.z*upVector.x;
    newVec.y = vec.x*rightVector.y + vec.y*forwardVector.y + vec.z*upVector.y;
    newVec.z = vec.x*rightVector.z + vec.y*forwardVector.z + vec.z*upVector.z;

    return newVec;
}