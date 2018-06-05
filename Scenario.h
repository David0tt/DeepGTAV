#pragma once

#include <stdlib.h>
#include <ctime>

#include "lib/script.h"
#include "lib/utils.h"

#include "lib/rapidjson/document.h"
#include "lib/rapidjson/stringbuffer.h"

#include "ScreenCapturer.h"
#include "Rewarders\Rewarder.h"
#include "LiDAR.h"
#include "Functions.h"

using namespace rapidjson;

static bool LOGGING = true;
static bool DEBUG_LOGGING = false;
static void log(std::string str, bool override = false) {
    if (override || LOGGING) {
        FILE* f = fopen("C:\\GTA V\\Braden.log", "a");
        fprintf(f, str.c_str());
        fprintf(f, "\n");
        fclose(f);
    }
}

//#define DEBUG 1

class Scenario {
private:
	static char* weatherList[14];
	static char* vehicleList[3];

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
	int width, height;

	bool vehicles;
	bool peds;
	bool trafficSigns; //TODO
	bool direction;
	bool reward;
	bool throttle;
	bool brake;
	bool steering;
	bool speed;
	bool yawRate;
	bool drivingMode; //TODO
	bool location;
	bool time;
    bool offscreen;
    bool showBoxes;
    bool pointclouds;
    bool stationaryScene;

	float currentThrottle = 0.0;
	float currentBrake = 0.0;
	float currentSteering = 0.0;

    Vector3 currentPos;
    Vector3 currentForwardVector;
    Vector3 currentUpVector;
    Vector3 currentRightVector;

	Rewarder* rewarder;
	std::clock_t lastSafetyCheck;
	int _drivingMode;
	float _setSpeed;

	bool running = false;
	Document d;

    //LiDAR variables
    LiDAR lidar;
    bool lidar_initialized = false;
    int instance_index = 30;
    int pointCloudSize = 0;
    std::unordered_map<int,int> entitiesHit;
    int lidar_param = 7;

    bool vehicles_created = false;
    std::vector<VehicleToCreate> vehiclesToCreate;
    std::vector<PedToCreate> pedsToCreate;

    //Camera intrinsic parameters
    float intrinsics[3];

public:
	float rate;

	void start(const Value& sc, const Value& dc);
	void stop();
	void config(const Value& sc, const Value& dc);
	void setCommands(float throttle, float brake, float steering);
	void run();

	ScreenCapturer* screenCapturer;
	StringBuffer generateMessage();

private:
	void parseScenarioConfig(const Value& sc, bool setDefaults);
	void parseDatasetConfig(const Value& dc, bool setDefaults);
	void buildScenario();

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
	void setLocation();
	void setTime();
    void setupLiDAR();
    void collectLiDAR();
    void setIndex();
    void drawBoxes(Vector3 BLL, Vector3 FUR, Vector3 dim, Vector3 upVector, Vector3 rightVector, Vector3 forwardVector, Vector3 position, int colour);
    void calcCameraIntrinsics();
    void setFocalLength();
    bool getEntityVector(Value &_entity, Document::AllocatorType& allocator, int entityID, Hash model, int classid);
    void setPosition();
    float observationAngle(Vector3 position);
    void drawVectorFromPosition(Vector3 vector, int blue, int green);
    void createVehicles();
    void createVehicle(const char* model, float relativeForward, float relativeRight, float heading, int color, int color2);
    void createPed(int model, float relativeForward, float relativeRight, float heading, int task);
    void increaseIndex();

    BBox2D BBox2DFrom3DObject(Vector3 position, Vector3 dim, Vector3 forwardVector, Vector3 rightVector, Vector3 upVector);
};