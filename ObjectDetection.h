#pragma once

#include <stdlib.h>
#include <ctime>

#include "LiDAR.h"
#include "Functions.h"
#include "CamParams.h"
#include "FrameObjectInfo.h"

//#define DEBUG 1

class ObjectDetection {
public:
    ObjectDetection();
    ~ObjectDetection();
private:
    FrameObjectInfo m_curFrame;
    bool m_initialized = false;
    bool m_eve = false;

    Vehicle vehicle = NULL;
    Player player = NULL;
    Ped ped = NULL;
    Cam camera = NULL;
    Vector3 dir;

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
    std::unordered_map<int, HitLidarEntity*> entitiesHit;
    int lidar_param = 7;

    //Depth Map variables
    float* m_pDepth = NULL;
    uint8_t* m_pStencil = NULL;
    float* m_pDMPointClouds = NULL;
    uint16_t* m_pDMImage = NULL;
    uint8_t* m_pStencilImage = NULL;
    unsigned char* color_buf = NULL;
    uint8_t* m_pStencilSeg = NULL;
    int m_stencilSegLength = 0;
    uint8_t* m_pOcclusionImage = NULL;
    uint8_t* m_pUnusedStencilImage = NULL;

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

public:
    void initCollection(UINT camWidth, UINT camHeight, bool exportEVE = true, int startIndex = 0);
    void setCamParams(float* forwardVec = NULL, float* rightVec = NULL, float* upVec = NULL);

    FrameObjectInfo setDepthAndStencil(bool prevDepth = false, float* pDepth = NULL, uint8_t* pStencil = NULL);
    //Depth buffer fn/var needs to be accessed by server
    void setDepthBuffer(bool prevDepth = false);
    bool m_prevDepth = false;

    FrameObjectInfo generateMessage(float* pDepth, uint8_t* pStencil);
    void exportDetections();
    void exportImage(BYTE* data);
    void increaseIndex();

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
    float observationAngle(Vector3 position);
    void drawVectorFromPosition(Vector3 vector, int blue, int green);
    void setDepthParams();
    Vector3 depthToCamCoords(float depth, float screenX, float screenY);
    std::string getStandardFilename(std::string subDir, std::string extension);
    void outputRealSpeed();
    void setStencilBuffer();
    void setFilenames();

    BBox2D BBox2DFrom3DObject(Vector3 position, Vector3 dim, Vector3 forwardVector, Vector3 rightVector, Vector3 upVector, bool &success, float &truncation);
    BBox2D processBBox2D(BBox2D bbox, uint8_t stencilType, Vector3 position, Vector3 dim, Vector3 forwardVector, Vector3 rightVector, Vector3 upVector,
        Vector3 xVector, Vector3 yVector, Vector3 zVector, int entityID, int &pointsHit2D, float &occlusion, bool pedOnBike);
    bool in3DBox(Vector3 point, Vector3 objPos, Vector3 dim, Vector3 yVector, Vector3 xVector, Vector3 zVector);
    bool checkDirection(Vector3 unit, Vector3 point, Vector3 min, Vector3 max);
    void printSegImage();

    bool hasLOSToEntity(Entity entityID, Vector3 position, Vector3 dim, Vector3 forwardVector, Vector3 rightVector, Vector3 upVector);

    void initVehicleLookup();
    bool isPointOccluding(Vector3 worldPos, Vector3 position);
    void outputOcclusion();
    void outputUnusedStencilPixels();

    //Export functions
    void exportEntity(ObjEntity e, std::ostringstream& oss, bool unprocessed);
    void exportEntities(EntityMap entMap, std::ostringstream& oss, bool unprocessed = false);
    void exportCalib();

    //Ground plane points
    void setGroundPlanePoints();
};