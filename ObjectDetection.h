#pragma once

#include <stdlib.h>
#include <ctime>

#include "LiDAR.h"
#include "Functions.h"
#include "CamParams.h"
#include "FrameObjectInfo.h"

//#define DEBUG 1

struct WorldObject {
    Entity e;
    std::string type;
    Hash model;
};

static Vector3 createVec3(float x, float y, float z) {
    Vector3 vec;
    vec.x = x;
    vec.y = y;
    vec.z = z;
    return vec;
}

struct SubsetInfo {
    Vector3 kittiPos;
    float kittiHeight;
    float kittiWidth;
    float kittiLength;
    float beta_kitti;
    float rot_y;
    float alpha_kitti;
};

class ObjectDetection {
public:
    ObjectDetection();
    ~ObjectDetection();
private:
    FrameObjectInfo m_curFrame;
    bool m_initialized = false;
    bool m_eve = false;

    Vehicle m_vehicle = NULL;
    Vehicle m_ownVehicle = NULL;
    Player player = NULL;
    Ped ped = NULL;
    Cam camera = NULL;
    Vector3 dir;
    ObjEntity m_ownVehicleObj;

    float x, y, z;
    float startHeading;
    int hour, minute;
    const char* _weather;
    const char* _vehicle;

    bool pointclouds;

    std::string baseFolder;

    float currentThrottle = 0.0;
    float currentBrake = 0.0;
    float currentSteering = 0.0;

    Vector3 currentPos;
    Vector3 vehicleForwardVector;
    Vector3 vehicleUpVector;
    Vector3 vehicleRightVector;

    Vector3 m_camForwardVector;
    Vector3 m_camRightVector;
    Vector3 m_camUpVector;

    bool running = false;

    int m_startArea = 1; //Downtown (see s_locationBounds)
    std::vector<std::vector<char>> m_polyGrid;

    //LiDAR variables
    LiDAR lidar;
    bool lidar_initialized = false;
    int pointCloudSize = 0;
    std::unordered_map<int, HitLidarEntity*> m_entitiesHit;
    int lidar_param = 7;

    //Perspective variables
    int m_vPerspective = -1;//Entity ID of perspective vehicle (-1 if self)

    //Depth Map variables
    float* m_pDepth = NULL;
    uint8_t* m_pStencil = NULL;
    float* m_pDMPointClouds = NULL;
    uint16_t* m_pDMImage = NULL;
    uint8_t* m_pStencilImage = NULL;
    unsigned char* color_buf = NULL;
    uint8_t* m_pStencilSeg = NULL;
    int m_stencilSegLength = 0;
    uint32_t* m_pInstanceSeg = NULL;
    int m_instanceSegLength = 0;
    uint8_t* m_pInstanceSegImg = NULL;
    int m_instanceSegImgLength = 0;
    uint8_t* m_pOcclusionImage = NULL;
    uint8_t* m_pUnusedStencilImage = NULL;
    uint8_t* m_pGroundPointsImage = NULL;

    std::string m_imgFilename;
    std::string m_veloFilename;
    std::string m_depthFilename;
    std::string m_depthPCFilename;
    std::string m_depthImgFilename;
    std::string m_stencilFilename;
    std::string m_stencilImgFilename;
    std::string m_segImgFilename;
    std::string m_occImgFilename;
    std::string m_unusedPixelsFilename;
    std::string m_calibFilename;
    std::string m_labelsFilename;
    std::string m_labelsUnprocessedFilename;
    std::string m_labelsAugFilename;
    std::string m_groundPointsFilename;
    std::string m_instSegFilename;
    std::string m_instSegImgFilename;
    std::string m_posFilename;
    std::string m_egoObjectFilename;

    std::string m_veloFilenameU;
    std::string m_depthPCFilenameU;

    bool vehicles_created = false;
    std::vector<VehicleToCreate> vehiclesToCreate;
    std::vector<PedToCreate> pedsToCreate;

    //For tracking: first frame in a series that an entity appears
    std::unordered_map<int, int> trackFirstFrame;
    //For calculating real speed
    Vector3 m_trackLastPos;
    float m_trackRealSpeed;
    float m_trackLastRealSpeed;
    float m_trackDist;
    float m_trackDistVar;
    float m_trackDistErrorTotal;
    float m_trackDistErrorTotalVar;
    float m_trackDistErrorTotalCount;
    int m_trackLastIndex = -1;
    int m_trackLastSeqIndex = 0;
    std::string m_timeTrackFile;
    std::string m_usedPixelFile;

    //Camera intrinsic parameters
    float intrinsics[3];

    bool m_vLookupInit = false;
    std::unordered_map<std::string, std::string> m_vLookup; //Vehicle lookup

    std::unordered_map<Vehicle, std::vector<Ped>> m_pedsInVehicles;

    //Map for tracking which entities are possible for each point which is in multiple 3D boxes
    std::unordered_map<int, std::vector<ObjEntity*>> m_overlappingPoints;

public:
    void initCollection(UINT camWidth, UINT camHeight, bool exportEVE = true, int startIndex = 0);
    void setCamParams(float* forwardVec = NULL, float* rightVec = NULL, float* upVec = NULL);
    void setOwnVehicleObject();

    FrameObjectInfo setDepthAndStencil(bool prevDepth = false, float* pDepth = NULL, uint8_t* pStencil = NULL);
    //Depth buffer fn/var needs to be accessed by server
    void setDepthBuffer(bool prevDepth = false);
    bool m_prevDepth = false;

    FrameObjectInfo generateMessage(float* pDepth, uint8_t* pStencil, int entityID = -1);
    void exportDetections(FrameObjectInfo fObjInfo, ObjEntity* vPerspective = NULL);
    void exportImage(BYTE* data, std::string filename = "");
    void increaseIndex();
    std::string getStandardFilename(std::string subDir, std::string extension);

    int instance_index = 0;
    int series_index = 0;
    std::string series_string = "0000";
    std::string instance_string;
    int baseTrackingIndex = instance_index;

    //Tracking variables
    bool collectTracking;
    //# of instances in one series
    const int trSeriesLength = 500;
    //# of seconds between series
    const int trSeriesGapTime = 30;
    //Used for keeing track of when to add the gap
    bool trSeriesGap = false;

    //Other vehicle detection labels
    std::vector<WorldObject> m_worldVehicles;
    std::vector<WorldObject> m_worldPeds;

private:
    void setVehiclesList();
    void setPedsList();
    void setTrafficSignsList();
    void setDirection();
    void setReward();
    void setThrottle();
    void setBrake();
    void setSteering();
    void setSpeed();
    void setYawRate();
    void setDrivingMode();
    void setTime();
    void setupLiDAR();
    void collectLiDAR();
    void setIndex();
    void calcCameraIntrinsics();
    void setFocalLength();
    bool getEntityVector(ObjEntity &entity, int entityID, Hash model, int classid, std::string type, std::string modelString, bool isPedInV, Vehicle vPedIsIn);
    void setPosition();
    Vector3 correctOffcenter(Vector3 position, Vector3 min, Vector3 max, Vector3 forwardVector, Vector3 rightVector, Vector3 upVector, Vector3 &offcenter);
    float observationAngle(Vector3 position);
    void drawVectorFromPosition(Vector3 vector, int blue, int green);
    void setDepthParams();
    Vector3 depthToCamCoords(float depth, float screenX, float screenY);
    void outputRealSpeed();
    void setStencilBuffer();
    void setFilenames();

    BBox2D BBox2DFrom3DObject(Vector3 position, Vector3 dim, Vector3 forwardVector, Vector3 rightVector, Vector3 upVector, bool &success, float &truncation);
    bool in3DBox(Vector3 point, Vector3 objPos, Vector3 dim, Vector3 yVector, Vector3 xVector, Vector3 zVector);
    bool in3DBox(ObjEntity *e, Vector3 point);
    bool checkDirection(Vector3 unit, Vector3 point, Vector3 min, Vector3 max);
    void processSegmentation();
    void processOverlappingPoints();
    void setEntityBBoxParameters(ObjEntity *e);
    void processStencilPixel(const uint8_t &stencilVal, const int &j, const int &i, const Vector3 &xVectorCam, const Vector3 &yVectorCam, const Vector3 &zVectorCam);
    void addPoint(int i, int j, ObjEntity &e);
    void addPointToSegImages(int i, int j, int entityID);
    void printSegImage();
    void getContours();
    //void outputGroundSeg();
    //void updateSegImage();

    void update3DPointsHit();
    void update3DPointsHit(ObjEntity* e);

    void processOcclusion();
    void processOcclusionForEntity(ObjEntity *e, const Vector3 &xVectorCam, const Vector3 &yVectorCam, const Vector3 &zVectorCam);

    void getRollAndPitch(Vector3 rightVector, Vector3 forwardVector, Vector3 upVector, float &pitch, float &roll);

    bool hasLOSToEntity(Entity entityID, Vector3 position, Vector3 dim, Vector3 forwardVector, Vector3 rightVector, Vector3 upVector, bool useOrigin = false, Vector3 origin = createVec3(0,0,0));

    void initVehicleLookup();
    bool isPointOccluding(Vector3 worldPos, ObjEntity* e);
    void outputOcclusion();
    void outputUnusedStencilPixels();

    //Export functions
    void exportEntity(ObjEntity e, std::ostringstream& oss, bool unprocessed, bool augmented, bool checkbbox2d = true, const int &maxDist = -1, const int &min2DPoints = -1, const int &min3DPoints = -1);
    void exportEntities(EntityMap entMap, std::ostringstream& oss, bool unprocessed = false, bool augmented = false, bool checkbbox2d = true, const int &maxDist = -1, const int &min2DPoints = -1, const int &min3DPoints = -1);
    void exportCalib();
    void exportPosition();
    void exportEgoObject(ObjEntity vPerspective);

    //Ground plane points
    Vector3 getGroundPoint(Vector3 point, Vector3 yVectorCam, Vector3 xVectorCam, Vector3 zVectorCam);
    void setGroundPlanePoints();

    //Other vehicle detections
    void getNearbyVehicles();
    void checkEntity(Vehicle p, WorldObject e, Vector3 pPos, std::ostringstream& oss);
    SubsetInfo getObjectInfoSubset(Vector3 position, Vector3 forwardVector, Vector3 dim);
    Vector3 getVehicleDims(Entity e, Hash model, Vector3 &min, Vector3 &max);
};