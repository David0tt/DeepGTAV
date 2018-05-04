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

using namespace rapidjson;

static bool LOGGING = true;
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

	float x, y;
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
    bool pointclouds;
    bool offscreen;
    bool showBoxes;

	float currentThrottle = 0.0;
	float currentBrake = 0.0;
	float currentSteering = 0.0;

	Rewarder* rewarder;
	std::clock_t lastSafetyCheck;
	int _drivingMode;
	float _setSpeed;

	bool running = false;
	Document d;

    //LiDAR variables
    LiDAR lidar;
    bool lidar_initialized = false;
    int instance_index = 60;
    int m_pointCloudSize = 0;

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
    void setCameraIntrinsics();
    bool getEntityVector(Value &_entity, Document::AllocatorType& allocator, int entityID, Hash model, int classid);
};