#pragma once

#include <map>

struct ObjEntity {
    int entityID;
    int classID;

    //Consistent with kitti coordinate system (relative to camera)
    Vector3 location;
    float speed;
    float heading;
    float height;
    float width;
    float length;
    Vector3 offcenter;

    float rotation_y;
    float alpha;
    
    float distance;
    BBox2D bbox2d;
    BBox2D bbox2dUnprocessed;
    
    int pointsHit3D;
    int pointsHit2D;
    float truncation;
    float occlusion;

    std::string modelString;
    std::string objType;

    int trackFirstFrame; //First frame (of sequence) it is seen in

    //These are in world coords right now (probably needs to be changed)
    float pitch;
    float roll;

    bool isPedInV = false;
    int vPedIsIn;
};

typedef std::pair<int, ObjEntity> EntityMapEntry;
typedef std::map<int, ObjEntity> EntityMap;

struct FrameObjectInfo
{
    int instanceIdx;
    int seriesIdx;

    Vector3 position;
    float heading;
    float roll;
    float pitch;

    //In world coordinates
    Vector3 forwardVec;
    Vector3 rightVec;
    Vector3 upVec;

    float speed;
    float yawRate;

    float focalLen;
    int timeHours;//In-game time of day (hours)

    //These are maps for when we do 4 cameras for full LiDAR
    //It will enable quicker checking to see if entity is already discovered
    EntityMap vehicles;
    EntityMap peds;

    std::vector<Vector3> groundPlanePoints;
};