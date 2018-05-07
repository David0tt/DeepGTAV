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