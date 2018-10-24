#include "Scenario.h"
#include "lib/utils.h"
#include "lib/rapidjson/writer.h"
#include "Rewarders\GeneralRewarder.h"
#include "Rewarders\LaneRewarder.h"
#include "Rewarders\SpeedRewarder.h"
#include "defaults.h"
#include <time.h>
#include <fstream>
#include <string>
#include <sstream>
#include "Functions.h"
#include "Constants.h"
#include <Eigen/Core>
#include "lodepng.h"
#include <sstream>
#include "AreaRoaming.h"

extern "C" {
    __declspec(dllimport) int export_get_depth_buffer(void** buf, bool updateWithOffsetDepth);
    __declspec(dllexport) int export_get_color_buffer(void** buf);
    __declspec(dllexport) int export_get_stencil_buffer(void** buf);
}

const float VERT_CAM_FOV = 59; //In degrees
//Need to input the vertical FOV with GTA functions.
//90 degrees horizontal (KITTI) corresponds to 59 degrees vertical (https://www.gtaall.com/info/fov-calculator.html).
const float HOR_CAM_FOV = 90; //In degrees

const float CAM_OFFSET_FORWARD = 0;// .5;
const float CAM_OFFSET_UP = 1.065;

const int VEHICLE_STENCIL_TYPE = 2;
const int NPC_STENCIL_TYPE = 1;
const int SKY_STENCIL_TYPE = 7;
const int PEDESTRIAN_CLASS_ID = 10;

char* Scenario::weatherList[14] = { "CLEAR", "EXTRASUNNY", "CLOUDS", "OVERCAST", "RAIN", "CLEARING", "THUNDER", "SMOG", "FOGGY", "XMAS", "SNOWLIGHT", "BLIZZARD", "NEUTRAL", "SNOW" };
char* Scenario::vehicleList[3] = { "blista", "blista", "blista" };//voltic, packer

void Scenario::parseScenarioConfig(const Value& sc, bool setDefaults) {
	const Value& location = sc["location"];
	const Value& time = sc["time"];
	const Value& weather = sc["weather"];
	const Value& vehicle = sc["vehicle"];
	const Value& drivingMode = sc["drivingMode"];

	if (location.IsArray()) {
		if (!location[0].IsNull()) x = location[0].GetFloat();
		else if (setDefaults) x = 5000 * ((float)rand() / RAND_MAX) - 2500;

		if (!location[1].IsNull()) y = location[1].GetFloat(); 
		else if (setDefaults) y = 8000 * ((float)rand() / RAND_MAX) - 2000;

        if (!location[2].IsNull()) z = location[2].GetFloat();
        else if (setDefaults) z = 0;

        if (!location[3].IsNull()) {
            log("Location 2 is not null");
            startHeading = location[3].GetFloat();
        }
        else if (setDefaults) {
            log("Location 3 is NULL");
            startHeading = 0;
        }
	}
	else if (setDefaults) {
		x = 5000 * ((float)rand() / RAND_MAX) - 2500;
		y = 8000 * ((float)rand() / RAND_MAX) - 2000;
	}
    if (DRIVE_SPEC_AREA) {
        x = s_locationBounds[0][0][0];
        y = s_locationBounds[0][1][0];
    }

	if (time.IsArray()) {
		if (!time[0].IsNull()) hour = time[0].GetInt();
		else if (setDefaults) hour = rand() % 24;

		if (!time[1].IsNull()) minute = time[1].GetInt();
		else if (setDefaults) minute = rand() % 60;
	}
	else if (setDefaults) {
        hour = 16;//TODO Do we want random times? rand() % 24;
		minute = rand() % 60;
	}

	if (!weather.IsNull()) _weather = weather.GetString();
    //TODO: Do we want other weather?
    else if (setDefaults) _weather = "CLEAR";// weatherList[rand() % 14];

	if (!vehicle.IsNull()) _vehicle = vehicle.GetString();
    else if (setDefaults) _vehicle = "ingot";// vehicleList[rand() % 3];

	if (drivingMode.IsArray()) {
		if (!drivingMode[0].IsNull()) _drivingMode = drivingMode[0].GetInt();
		else if (setDefaults)  _drivingMode = rand() % 4294967296;
		if (drivingMode[1].IsNull()) _setSpeed = drivingMode[1].GetFloat(); 
		else if (setDefaults) _setSpeed = 1.0*(rand() % 10) + 10;
	}
	else if (setDefaults) {
		_drivingMode = -1;
	}
}

void Scenario::parseDatasetConfig(const Value& dc, bool setDefaults) {
	if (!dc["rate"].IsNull()) rate = dc["rate"].GetFloat();
	else if (setDefaults) rate = _RATE_;

    if (!dc["startIndex"].IsNull()) {
        instance_index = dc["startIndex"].GetInt();
        baseTrackingIndex = instance_index;
    }

    char temp[] = "%06d";
    char strComp[sizeof temp + 100];
    sprintf(strComp, temp, instance_index);
    instance_string = strComp;
	
	if (!dc["frame"].IsNull()) {
		if (!dc["frame"][0].IsNull()) s_camParams.width = dc["frame"][0].GetInt();
		else if (setDefaults) s_camParams.width = _WIDTH_;

		if (!dc["frame"][1].IsNull()) s_camParams.height = dc["frame"][1].GetInt();
		else if (setDefaults) s_camParams.height = _HEIGHT_;
	}
	else if (setDefaults) {
		s_camParams.width = _WIDTH_;
		s_camParams.height = _HEIGHT_;
	}

    //Need to reset camera params when dataset config is received
    s_camParams.init = false;

	if (!dc["vehicles"].IsNull()) vehicles = dc["vehicles"].GetBool();
	else if (setDefaults) vehicles = _VEHICLES_;

	if (!dc["peds"].IsNull()) peds = dc["peds"].GetBool();
	else if (setDefaults) peds = _PEDS_;

	if (!dc["trafficSigns"].IsNull()) trafficSigns = dc["trafficSigns"].GetBool();
	else if (setDefaults) trafficSigns = _TRAFFIC_SIGNS_;

	if (!dc["direction"].IsNull()) {
		direction = true;
		if (!dc["direction"][0].IsNull()) dir.x = dc["direction"][0].GetFloat();
		else if (setDefaults) direction = _DIRECTION_;

		if (!dc["direction"][1].IsNull()) dir.y = dc["direction"][1].GetFloat();
		else if (setDefaults) direction = _DIRECTION_;

		if (!dc["direction"][2].IsNull()) dir.z = dc["direction"][2].GetFloat();
		else if (setDefaults) direction = _DIRECTION_;
	}
	else if (setDefaults) direction = _DIRECTION_;

	if (dc["reward"].IsArray()) {
		if (dc["reward"][0].IsFloat() && dc["reward"][1].IsFloat()) {
			rewarder = new GeneralRewarder((char*)(GetCurrentModulePath() + "paths.xml").c_str(), dc["reward"][0].GetFloat(), dc["reward"][1].GetFloat());
			reward = true;
		}
		else if (setDefaults) reward = _REWARD_;
	}
	else if (setDefaults) reward = _REWARD_;

	if (!dc["throttle"].IsNull()) throttle = dc["throttle"].GetBool();
	else if (setDefaults) throttle = _THROTTLE_;
	if (!dc["brake"].IsNull()) brake = dc["brake"].GetBool();
	else if (setDefaults) brake = _BRAKE_;
	if (!dc["steering"].IsNull()) steering = dc["steering"].GetBool();
	else if (setDefaults) steering = _STEERING_;
	if (!dc["speed"].IsNull()) speed = dc["speed"].GetBool();
	else if (setDefaults) speed = _SPEED_;
	if (!dc["yawRate"].IsNull()) yawRate = dc["yawRate"].GetBool();
	else if (setDefaults) yawRate = _YAW_RATE_;
	if (!dc["drivingMode"].IsNull()) drivingMode = dc["drivingMode"].GetBool();
	else if (setDefaults) drivingMode = _DRIVING_MODE_;
	if (!dc["location"].IsNull()) location = dc["location"].GetBool();
	else if (setDefaults) location = _LOCATION_;
	if (!dc["time"].IsNull()) time = dc["time"].GetBool();
	else if (setDefaults) time = _TIME_;
    if (!dc["offscreen"].IsNull()) offscreen = dc["offscreen"].GetBool();
    else if (setDefaults) offscreen = _OFFSCREEN_;
    if (!dc["showBoxes"].IsNull()) showBoxes = dc["showBoxes"].GetBool();
    else if (setDefaults) showBoxes = _SHOWBOXES_;
    if (!dc["pointclouds"].IsNull()) pointclouds = dc["pointclouds"].GetBool();
    else if (setDefaults) pointclouds = _POINTCLOUDS_;
    if (!dc["stationaryScene"].IsNull()) stationaryScene = dc["stationaryScene"].GetBool();
    else if (setDefaults) stationaryScene = _STATIONARY_SCENE_;
    if (!dc["collectTracking"].IsNull()) collectTracking = dc["collectTracking"].GetBool();
    else if (setDefaults) collectTracking = _COLLECT_TRACKING_;
    if (!dc["recordScenario"].IsNull()) m_recordScenario = dc["recordScenario"].GetBool();
    else if (setDefaults) m_recordScenario = _RECORD_SCENARIO_;

    if (DRIVE_SPEC_AREA) {
        dir.x = s_locationBounds[0][0][m_startArea];
        dir.y = s_locationBounds[0][1][m_startArea];
        dir.z = 0.f;
    }

    if (stationaryScene) {
        vehiclesToCreate.clear();
        log("About to get vehicles");
        if (!dc["vehiclesToCreate"].IsNull()) {
            log("Vehicles non-null");
            const rapidjson::Value& jsonVehicles = dc["vehiclesToCreate"];
            for (rapidjson::SizeType i = 0; i < jsonVehicles.Size(); i++) {
                log("At least one");
                bool noHit = false;
                VehicleToCreate vehicleToCreate;
                const rapidjson::Value& jVeh = jsonVehicles[i];

                if (!jVeh[0].IsNull()) vehicleToCreate.model = jVeh[0].GetString();
                if (!jVeh[1].IsNull()) vehicleToCreate.forward = jVeh[1].GetFloat();
                if (!jVeh[2].IsNull()) vehicleToCreate.right = jVeh[2].GetFloat();
                if (!jVeh[3].IsNull()) vehicleToCreate.heading = jVeh[3].GetFloat();
                if (!jVeh[4].IsNull()) vehicleToCreate.color = jVeh[4].GetInt();
                if (!jVeh[5].IsNull()) vehicleToCreate.color2 = jVeh[5].GetInt();
                else noHit = true;

                if (!noHit) {
                    log("Pushing back vehicle");
                    vehiclesToCreate.push_back(vehicleToCreate);
                }
            }
        }
        pedsToCreate.clear();
        log("About to get ped");
        if (!dc["pedsToCreate"].IsNull()) {
            log("ped non-null");
            const rapidjson::Value& jsonPeds = dc["pedsToCreate"];
            for (rapidjson::SizeType i = 0; i < jsonPeds.Size(); i++) {
                log("At least one");
                bool noHit = false;
                PedToCreate pedToCreate;
                const rapidjson::Value& jPed = jsonPeds[i];

                if (!jPed[0].IsNull()) pedToCreate.model = jPed[0].GetInt();
                if (!jPed[1].IsNull()) pedToCreate.forward = jPed[1].GetFloat();
                if (!jPed[2].IsNull()) pedToCreate.right = jPed[2].GetFloat();
                if (!jPed[3].IsNull()) pedToCreate.heading = jPed[3].GetFloat();
                else noHit = true;

                if (!noHit) {
                    log("Pushing back ped");
                    pedsToCreate.push_back(pedToCreate);
                }
            }
        }
    }

	//Create JSON DOM
	d.SetObject();
	Document::AllocatorType& allocator = d.GetAllocator();
	Value a(kArrayType);

	if (vehicles) d.AddMember("vehicles", a, allocator);
	if (peds) d.AddMember("peds", a, allocator);
	if (trafficSigns) d.AddMember("trafficSigns", a, allocator);
	if (direction) d.AddMember("direction", a, allocator);
	if (reward) d.AddMember("reward", 0.0, allocator);
	if (throttle) d.AddMember("throttle", 0.0, allocator);
	if (brake) d.AddMember("brake", 0.0, allocator);
	if (steering) d.AddMember("steering", 0.0, allocator);
	if (speed) d.AddMember("speed", 0.0, allocator);
	if (yawRate) d.AddMember("yawRate", 0.0, allocator);
	if (drivingMode) d.AddMember("drivingMode", 0, allocator);
	if (location) d.AddMember("location", a, allocator);
	if (time) d.AddMember("time", 0, allocator);
    d.AddMember("index", 0, allocator);
    d.AddMember("focalLen", 0.0, allocator);
    d.AddMember("curPosition", a, allocator);
    d.AddMember("seriesIndex", a, allocator);

	screenCapturer = new ScreenCapturer(s_camParams.width, s_camParams.height);
}

void Scenario::buildScenario() {
	Vector3 pos, rotation;
	Hash vehicleHash;
	float heading;

    if (!stationaryScene) {
        GAMEPLAY::SET_RANDOM_SEED(std::time(NULL));
        while (!PATHFIND::_0xF7B79A50B905A30D(-8192.0f, 8192.0f, -8192.0f, 8192.0f)) WAIT(0);
        PATHFIND::GET_CLOSEST_VEHICLE_NODE_WITH_HEADING(x, y, 0, &pos, &heading, 0, 0, 0);
    }

	ENTITY::DELETE_ENTITY(&vehicle);
	vehicleHash = GAMEPLAY::GET_HASH_KEY((char*)_vehicle);
	STREAMING::REQUEST_MODEL(vehicleHash);
	while (!STREAMING::HAS_MODEL_LOADED(vehicleHash)) WAIT(0);
    if (stationaryScene) {
        pos.x = x;
        pos.y = y;
        pos.z = 0;
        heading = startHeading;
        std::ostringstream oss;
        oss << "Start heading: " << startHeading;
        std::string str = oss.str();
        log(str);
        vehicles_created = false;
    }
	while (!ENTITY::DOES_ENTITY_EXIST(vehicle)) {
		vehicle = VEHICLE::CREATE_VEHICLE(vehicleHash, pos.x, pos.y, pos.z, heading, FALSE, FALSE);
		WAIT(0);
	}
	VEHICLE::SET_VEHICLE_ON_GROUND_PROPERLY(vehicle);

	while (!ENTITY::DOES_ENTITY_EXIST(ped)) {
		ped = PLAYER::PLAYER_PED_ID();
		WAIT(0);
	}

	player = PLAYER::PLAYER_ID();
	PLAYER::START_PLAYER_TELEPORT(player, pos.x, pos.y, pos.z, heading, 0, 0, 0);
	while (PLAYER::IS_PLAYER_TELEPORT_ACTIVE()) WAIT(0);

	PED::SET_PED_INTO_VEHICLE(ped, vehicle, -1);
	STREAMING::SET_MODEL_AS_NO_LONGER_NEEDED(vehicleHash);

	TIME::SET_CLOCK_TIME(hour, minute, 0);

	GAMEPLAY::SET_WEATHER_TYPE_NOW_PERSIST((char*)_weather);

	rotation = ENTITY::GET_ENTITY_ROTATION(vehicle, 0);
	CAM::DESTROY_ALL_CAMS(TRUE);
	camera = CAM::CREATE_CAM("DEFAULT_SCRIPTED_CAMERA", TRUE);
	if (strcmp(_vehicle, "packer") == 0) CAM::ATTACH_CAM_TO_ENTITY(camera, vehicle, 0, 2.35, 1.7, TRUE);
	else CAM::ATTACH_CAM_TO_ENTITY(camera, vehicle, 0, CAM_OFFSET_FORWARD, CAM_OFFSET_UP, TRUE);
	CAM::SET_CAM_FOV(camera, VERT_CAM_FOV);
	CAM::SET_CAM_ACTIVE(camera, TRUE);
	CAM::SET_CAM_ROT(camera, rotation.x, rotation.y, rotation.z, 0);
	CAM::SET_CAM_INHERIT_ROLL_VEHICLE(camera, TRUE);

    if (stationaryScene) {
        _setSpeed = 0;
    }

    CAM::RENDER_SCRIPT_CAMS(TRUE, FALSE, 0, TRUE, TRUE);

	AI::CLEAR_PED_TASKS(ped);
	if (_drivingMode >= 0 && !stationaryScene) {
        if (DRIVE_SPEC_AREA && !START_SPEC_AREA) {
            AI::TASK_VEHICLE_DRIVE_TO_COORD(ped, vehicle, dir.x, dir.y, dir.z, _setSpeed, Any(1.f), vehicleHash, _drivingMode, 50.f, true);
        }
        else {
            AI::TASK_VEHICLE_DRIVE_WANDER(ped, vehicle, _setSpeed, _drivingMode);
        }
        
    }

    //while (!CAM::IS_GAMEPLAY_CAM_RENDERING()) {
    //    camera = CAM::GET_RENDERING_CAM();// CAM::CREATE_CAM("DEFAULT_SCRIPTED_CAMERA", TRUE);
    //    if (strcmp(_vehicle, "packer") == 0) CAM::ATTACH_CAM_TO_ENTITY(camera, vehicle, 0, 2.35, 1.7, TRUE);
    //    else CAM::ATTACH_CAM_TO_ENTITY(camera, vehicle, 0, CAM_OFFSET_FORWARD, CAM_OFFSET_UP, TRUE);
    //    CAM::SET_CAM_FOV(camera, VERT_CAM_FOV);
    //    CAM::SET_CAM_ACTIVE(camera, TRUE);
    //    //CAM::SET_CAM_ROT(camera, rotation.x, rotation.y, rotation.z, 0);
    //    CAM::SET_CAM_INHERIT_ROLL_VEHICLE(camera, TRUE);
    //    CAM::SET_GAMEPLAY_CAM_RELATIVE_HEADING(0);
    //    CAM::SET_GAMEPLAY_CAM_RELATIVE_PITCH(0, 0x3F800000);//Constant taken from nativedb
    //}

    if (m_recordScenario) {
        UNK1::_SET_RECORDING_MODE(1);
    }
}

void Scenario::start(const Value& sc, const Value& dc) {
	if (running) return;

	//Parse options
	srand(std::time(NULL));
	parseScenarioConfig(sc, true);
	parseDatasetConfig(dc, true);

	//Build scenario
	buildScenario();

	running = true;
	lastSafetyCheck = std::clock();
}

void Scenario::config(const Value& sc, const Value& dc) {
	if (!running) return;

	running = false;

	//Parse options
	srand(std::time(NULL));
	parseScenarioConfig(sc, false);
	parseDatasetConfig(dc, false);

	//Build scenario
	buildScenario();

	running = true;
	lastSafetyCheck = std::clock();
}

void Scenario::run() {
	if (running) {
        if (m_recordScenario) {
            Vector3 rotation = ENTITY::GET_ENTITY_ROTATION(vehicle, 0);
            CAM::SET_CAM_ROT(camera, rotation.x, rotation.y, rotation.z, 0);
        }

		std::clock_t now = std::clock();

        if (SAME_TIME_OF_DAY) {
            TIME::SET_CLOCK_TIME(hour, minute, 0);
        }

        if (DRIVE_SPEC_AREA && !START_SPEC_AREA) {
            if (pow(currentPos.x - dir.x, 2) + pow(currentPos.y - dir.y, 2) < pow(50, 2))
            {
                std::vector<std::pair<float, float>> new_points = generate_n_random_points(
                    m_startArea, m_polyGrid, 1, 100, { { currentPos.x , currentPos.y } });
                dir.x = new_points[0].first;
                dir.y = new_points[0].second;

                AI::TASK_VEHICLE_DRIVE_TO_COORD(ped, vehicle, dir.x, dir.y, dir.z, _setSpeed, Any(1.f),
                    GAMEPLAY::GET_HASH_KEY((char*)_vehicle), _drivingMode, 1.f, true);
            }
            else if (!in_bounds(currentPos.x, currentPos.y, m_startArea, m_polyGrid))
            {
                std::vector<std::pair<float, float>> new_points = generate_n_random_points(
                    m_startArea, m_polyGrid, 1, 100, { { currentPos.x , currentPos.y } });
                dir.x = new_points[0].first;
                dir.y = new_points[0].second;

                AI::TASK_VEHICLE_DRIVE_TO_COORD(ped, vehicle, dir.x, dir.y, dir.z, _setSpeed, Any(1.f),
                    GAMEPLAY::GET_HASH_KEY((char*)_vehicle), _drivingMode, 1.f, true);
            }
        }

		if (_drivingMode < 0) {
			CONTROLS::_SET_CONTROL_NORMAL(27, 71, currentThrottle); //[0,1]
			CONTROLS::_SET_CONTROL_NORMAL(27, 72, currentBrake); //[0,1]
			CONTROLS::_SET_CONTROL_NORMAL(27, 59, currentSteering); //[-1,1]
		}
		
		float delay = ((float)(now - lastSafetyCheck)) / CLOCKS_PER_SEC;
		if (delay > 10) {
            //Need to delay first camera parameters being set so native functions return correct values
            if (!s_camParams.firstInit) {
                s_camParams.init = false;
                setCamParams();
                s_camParams.firstInit = true;
            }

			lastSafetyCheck = std::clock();
			//Avoid bad things such as getting killed by the police, robbed, dying in car accidents or other horrible stuff
			PLAYER::SET_EVERYONE_IGNORE_PLAYER(player, TRUE);
			PLAYER::SET_POLICE_IGNORE_PLAYER(player, TRUE);
			PLAYER::CLEAR_PLAYER_WANTED_LEVEL(player); // Never wanted

			// Put on seat belt
			PED::SET_PED_CONFIG_FLAG(ped, 32, FALSE);

			// Invincible vehicle
			VEHICLE::SET_VEHICLE_TYRES_CAN_BURST(vehicle, FALSE);
			VEHICLE::SET_VEHICLE_WHEELS_CAN_BREAK(vehicle, FALSE);
			VEHICLE::SET_VEHICLE_HAS_STRONG_AXLES(vehicle, TRUE);

			VEHICLE::SET_VEHICLE_CAN_BE_VISIBLY_DAMAGED(vehicle, FALSE);
			ENTITY::SET_ENTITY_INVINCIBLE(vehicle, TRUE);
			ENTITY::SET_ENTITY_PROOFS(vehicle, 1, 1, 1, 1, 1, 1, 1, 1);

			// Player invincible
			PLAYER::SET_PLAYER_INVINCIBLE(player, TRUE);

			// Driving characteristics
			PED::SET_DRIVER_AGGRESSIVENESS(ped, 0.0);
			PED::SET_DRIVER_ABILITY(ped, 100.0);

            //Create vehicles if it is a stationary scenario
            createVehicles();
		}
	}
	scriptWait(0);
}

void Scenario::stop() {
	if (!running) return;
	running = false;
	CAM::DESTROY_ALL_CAMS(TRUE);
	CAM::RENDER_SCRIPT_CAMS(FALSE, TRUE, 500, FALSE, FALSE);
	AI::CLEAR_PED_TASKS(ped);
	setCommands(0.0, 0.0, 0.0);
}

void Scenario::setCommands(float throttle, float brake, float steering) {
	currentThrottle = throttle;
	currentBrake = brake;
	currentSteering = steering;
}

StringBuffer Scenario::generateMessage() {
	StringBuffer buffer;
	buffer.Clear();
	Writer<StringBuffer> writer(buffer);

    if (m_recordScenario) {
        Vector3 rotation = ENTITY::GET_ENTITY_ROTATION(vehicle, 0);
        CAM::SET_CAM_ROT(camera, rotation.x, rotation.y, rotation.z, 0);
        return buffer;
    }
	
    log("About to pause game");
    GAMEPLAY::SET_GAME_PAUSED(true);

    //Need to ensure camera rotation aligns with vehicle after game is paused
    Vector3 rotation = ENTITY::GET_ENTITY_ROTATION(vehicle, 0);
    CAM::SET_CAM_ROT(camera, rotation.x, rotation.y, rotation.z, 0);
    Vector3 camRot = CAM::GET_CAM_ROT(camera, 0);

    //Time synchronization seems to be correct with 2 render calls
    CAM::RENDER_SCRIPT_CAMS(TRUE, FALSE, 0, FALSE, FALSE);
    scriptWait(0);
    CAM::RENDER_SCRIPT_CAMS(TRUE, FALSE, 0, FALSE, FALSE);
    scriptWait(0);
    CAM::RENDER_SCRIPT_CAMS(TRUE, FALSE, 0, FALSE, FALSE);
    scriptWait(0);

    ////Can check whether camera and vehicle are aligned
    //Vector3 camRot2 = CAM::GET_CAM_ROT(camera, 0);
    //std::ostringstream oss1;
    //oss1 << "entityRotation X: " << rotation.x << " Y: " << rotation.y << " Z: " << rotation.z <<
    //    "\n camRot X: " << camRot.x << " Y: " << camRot.y << " Z: " << camRot.z <<
    //    "\n camRot2 X: " << camRot2.x << " Y: " << camRot2.y << " Z: " << camRot2.z;
    //std::string str1 = oss1.str();
    //log(str1);

    log("Script cams rendered");
    screenCapturer->capture();
    log("Screen captured");

    //TODO pass this through
    bool depthMap = true;

    setCamParams();
    //setColorBuffer();
    if (depthMap) setDepthBuffer();
    if (depthMap) setStencilBuffer();
	if (trafficSigns); //TODO
	if (direction) setDirection();
	if (reward) setReward();
	if (throttle) setThrottle();
	if (brake) setBrake();
	if (steering) setSteering();
	if (drivingMode); //TODO

    if (!m_pObjDet) {
        m_pObjDet.reset(new ObjectDetection());
        m_pObjDet->initCollection(s_camParams.width, s_camParams.height, false);
    }
    FrameObjectInfo fObjInfo = m_pObjDet->generateMessage(depth_map, m_stencilBuffer);
    log("After generate msg", true);
    m_pObjDet->exportDetections();
    log("After export detections", true);
    std::ostringstream oss1;
    oss1 << "screencap length: " << screenCapturer->length << " width/height: " << screenCapturer->imageWidth << "/" << screenCapturer->imageHeight;
    std::string str1 = oss1.str();
    log(str1, true);
    //m_pObjDet->exportImage(screenCapturer->pixels);
    log("After exportImage", true);
    m_pObjDet->increaseIndex();
    log("After generate msg", true);
    d["index"] = fObjInfo.instanceIdx;

    GAMEPLAY::SET_GAME_PAUSED(false);

	d.Accept(writer);

	return buffer;
}

void Scenario::setThrottle(){
	d["throttle"] = getFloatValue(vehicle, 0x92C);
}

void Scenario::setBrake(){
	d["brake"] = getFloatValue(vehicle, 0x930);
}

void Scenario::setSteering(){
	d["steering"] = -getFloatValue(vehicle, 0x924) / 0.6981317008;
}

void Scenario::setDirection(){
	int direction;
	float distance;
	Vehicle temp_vehicle;
	Document::AllocatorType& allocator = d.GetAllocator();
	PATHFIND::GENERATE_DIRECTIONS_TO_COORD(dir.x, dir.y, dir.z, TRUE, &direction, &temp_vehicle, &distance);
	Value _direction(kArrayType);
	_direction.PushBack(direction, allocator).PushBack(distance, allocator);
	d["direction"] = _direction;
}

void Scenario::setReward() {
	d["reward"] = rewarder->computeReward(vehicle);
}

void Scenario::createVehicle(const char* model, float relativeForward, float relativeRight, float heading, int color, int color2) {
    Hash vehicleHash = GAMEPLAY::GET_HASH_KEY(const_cast<char*>(model));
    Vector3 pos;
    pos.x = currentPos.x + currentForwardVector.x * relativeForward + currentRightVector.x * relativeRight;
    pos.y = currentPos.y + currentForwardVector.y * relativeForward + currentRightVector.y * relativeRight;
    pos.z = currentPos.z + currentForwardVector.z * relativeForward + currentRightVector.z * relativeRight;
    STREAMING::REQUEST_MODEL(vehicleHash);
    while (!STREAMING::HAS_MODEL_LOADED(vehicleHash)) WAIT(0);
    Vehicle tempV = VEHICLE::CREATE_VEHICLE(vehicleHash, pos.x, pos.y, pos.z, heading, FALSE, FALSE);
    WAIT(0);
    if (color != -1) {
        VEHICLE::SET_VEHICLE_COLOURS(tempV, color, color2);
    }
    VEHICLE::SET_VEHICLE_ON_GROUND_PROPERLY(tempV);
    ENTITY::SET_ENTITY_AS_NO_LONGER_NEEDED(&tempV);
}

void Scenario::createPed(int model, float relativeForward, float relativeRight, float heading, int task) {
    //Ped hashes found at: https://www.se7ensins.com/forums/threads/request-pc-ped-hashes.1317848/
    Hash hash = 0x505603B9;// GAMEPLAY::GET_HASH_KEY(const_cast<char*>(model));
    Vector3 pos;
    pos.x = currentPos.x + currentForwardVector.x * relativeForward + currentRightVector.x * relativeRight;
    pos.y = currentPos.y + currentForwardVector.y * relativeForward + currentRightVector.y * relativeRight;
    pos.z = currentPos.z + currentForwardVector.z * relativeForward + currentRightVector.z * relativeRight;
    STREAMING::REQUEST_MODEL(hash);
    while (!STREAMING::HAS_MODEL_LOADED(hash)) WAIT(0);
    Ped temp = PED::CREATE_PED(4, hash, pos.x, pos.y, pos.z, heading, FALSE, FALSE);
    WAIT(0);
    AI::TASK_WANDER_STANDARD(ped, 10.0f, 10);
    if (task == 0) {
        AI::TASK_STAND_STILL(temp, -1);
    }
    else if (task == 1) {
        PED::SET_PED_PINNED_DOWN(temp, true, -1);
    }
    else if (task == 2) {
        AI::TASK_WRITHE(temp, player, 999999, false);
    }
    else if (task == 3) {
        PED::SET_PED_DUCKING(temp, true);
    }
    ENTITY::SET_ENTITY_AS_NO_LONGER_NEEDED(&temp);
}

void Scenario::createVehicles() {
    setPosition();
    if (stationaryScene && !vehicles_created) {
        log("Creating peds");
        for (int i = 0; i < pedsToCreate.size(); i++) {
            PedToCreate p = pedsToCreate[i];
            createPed(p.model, p.forward, p.right, p.heading, i);
        }
        log("Creating vehicles");
        for (int i = 0; i < vehiclesToCreate.size(); i++) {
            VehicleToCreate v = vehiclesToCreate[i];
            createVehicle(v.model.c_str(), v.forward, v.right, v.heading, v.color, v.color2);
        }
        vehicles_created = true;
    }
}

//Saves the position and vectors of the capture vehicle
void Scenario::setPosition() {
    //NOTE: The forward and right vectors are swapped (compared to native function labels) to keep consistency with coordinate system
    ENTITY::GET_ENTITY_MATRIX(vehicle, &currentForwardVector, &currentRightVector, &currentUpVector, &currentPos); //Blue or red pill
}

//TODO Calls to export_get_color_buffer are causing GTA to crash
void Scenario::setColorBuffer() {
    log("Before color buffer", true);
    int size = export_get_color_buffer((void**)&color_buf);
    log("After color buffer", true);
}

void Scenario::setStencilBuffer() {
    int size;
    std::string filename;
    log("About to get stencil buffer");
    size = export_get_stencil_buffer((void**)&m_stencilBuffer);
    log("After getting stencil buffer");
}

void Scenario::setDepthBuffer(bool prevDepth) {
    int size;
    std::string filename;
    std::string pcFilename;
    log("About to get depth buffer");
    size = export_get_depth_buffer((void**)&depth_map, false);

    //TODO Use stencil buffer
    log("After getting depth buffer");
}

void Scenario::drawBoxes(Vector3 BLL, Vector3 FUR, Vector3 dim, Vector3 upVector, Vector3 rightVector, Vector3 forwardVector, Vector3 position, int colour) {
    //log("Inside draw boxes");
    if (showBoxes) {
        log("Inside show boxes");
        Vector3 edge1 = BLL;
        Vector3 edge2;
        Vector3 edge3;
        Vector3 edge4;
        Vector3 edge5 = FUR;
        Vector3 edge6;
        Vector3 edge7;
        Vector3 edge8;

        int green = colour * 255;
        int blue = abs(colour - 1) * 255;

        edge2.x = edge1.x + 2 * dim.y*rightVector.x;
        edge2.y = edge1.y + 2 * dim.y*rightVector.y;
        edge2.z = edge1.z + 2 * dim.y*rightVector.z;

        edge3.x = edge2.x + 2 * dim.z*upVector.x;
        edge3.y = edge2.y + 2 * dim.z*upVector.y;
        edge3.z = edge2.z + 2 * dim.z*upVector.z;

        edge4.x = edge1.x + 2 * dim.z*upVector.x;
        edge4.y = edge1.y + 2 * dim.z*upVector.y;
        edge4.z = edge1.z + 2 * dim.z*upVector.z;

        edge6.x = edge5.x - 2 * dim.y*rightVector.x;
        edge6.y = edge5.y - 2 * dim.y*rightVector.y;
        edge6.z = edge5.z - 2 * dim.y*rightVector.z;

        edge7.x = edge6.x - 2 * dim.z*upVector.x;
        edge7.y = edge6.y - 2 * dim.z*upVector.y;
        edge7.z = edge6.z - 2 * dim.z*upVector.z;

        edge8.x = edge5.x - 2 * dim.z*upVector.x;
        edge8.y = edge5.y - 2 * dim.z*upVector.y;
        edge8.z = edge5.z - 2 * dim.z*upVector.z;

        GRAPHICS::DRAW_LINE(edge1.x, edge1.y, edge1.z, edge2.x, edge2.y, edge2.z, 0, green, blue, 200);
        GRAPHICS::DRAW_LINE(edge1.x, edge1.y, edge1.z, edge4.x, edge4.y, edge4.z, 0, green, blue, 200);
        GRAPHICS::DRAW_LINE(edge2.x, edge2.y, edge2.z, edge3.x, edge3.y, edge3.z, 0, green, blue, 200);
        GRAPHICS::DRAW_LINE(edge3.x, edge3.y, edge3.z, edge4.x, edge4.y, edge4.z, 0, green, blue, 200);

        GRAPHICS::DRAW_LINE(edge5.x, edge5.y, edge5.z, edge6.x, edge6.y, edge6.z, 0, green, blue, 200);
        GRAPHICS::DRAW_LINE(edge5.x, edge5.y, edge5.z, edge8.x, edge8.y, edge8.z, 0, green, blue, 200);
        GRAPHICS::DRAW_LINE(edge6.x, edge6.y, edge6.z, edge7.x, edge7.y, edge7.z, 0, green, blue, 200);
        GRAPHICS::DRAW_LINE(edge7.x, edge7.y, edge7.z, edge8.x, edge8.y, edge8.z, 0, green, blue, 200);

        GRAPHICS::DRAW_LINE(edge1.x, edge1.y, edge1.z, edge7.x, edge7.y, edge7.z, 0, green, blue, 200);
        GRAPHICS::DRAW_LINE(edge2.x, edge2.y, edge2.z, edge8.x, edge8.y, edge8.z, 0, green, blue, 200);
        GRAPHICS::DRAW_LINE(edge3.x, edge3.y, edge3.z, edge5.x, edge5.y, edge5.z, 0, green, blue, 200);
        GRAPHICS::DRAW_LINE(edge4.x, edge4.y, edge4.z, edge6.x, edge6.y, edge6.z, 0, green, blue, 200);
        WAIT(0);
    }
}

void Scenario::setCamParams() {
    //These values stay the same throughout a collection period
    if (!s_camParams.init) {
        s_camParams.nearClip = CAM::GET_CAM_NEAR_CLIP(camera);
        s_camParams.farClip = 10000;// CAM::GET_CAM_FAR_CLIP(camera);
        s_camParams.fov = CAM::GET_CAM_FOV(camera);
        s_camParams.ncHeight = 2 * s_camParams.nearClip * tan(s_camParams.fov / 2. * (PI / 180.)); // field of view is returned vertically
        s_camParams.ncWidth = s_camParams.ncHeight * GRAPHICS::_GET_SCREEN_ASPECT_RATIO(false);
        s_camParams.init = true;

        if (m_recordScenario) {
            std::ostringstream oss;
            oss << "NC, FC, FOV: " << s_camParams.nearClip << ", " << s_camParams.farClip << ", " << s_camParams.fov;
            std::string str = oss.str();
            log(str, true);
        }
    }

    //These values change frame to frame
    s_camParams.theta = CAM::GET_CAM_ROT(camera, 0);
    s_camParams.pos = CAM::GET_CAM_COORD(camera);

    std::ostringstream oss1;
    oss1 << "\ns_camParams.pos X: " << s_camParams.pos.x << " Y: " << s_camParams.pos.y << " Z: " << s_camParams.pos.z <<
        "\nvehicle.pos X: " << currentPos.x << " Y: " << currentPos.y << " Z: " << currentPos.z <<
        "\nfar: " << s_camParams.farClip << " nearClip: " << s_camParams.nearClip << " fov: " << s_camParams.fov <<
        "\nrotation gameplay: " << s_camParams.theta.x << " Y: " << s_camParams.theta.y << " Z: " << s_camParams.theta.z <<
        "\n AspectRatio: " << GRAPHICS::_GET_SCREEN_ASPECT_RATIO(false);
    std::string str1 = oss1.str();
    log(str1, true);

    //For measuring height of camera (LiDAR) to ground plane
    /*float groundZ;
    GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(s_camParams.pos.x, s_camParams.pos.y, s_camParams.pos.z, &(groundZ), 0);
    
    std::ostringstream oss;
    oss << "LiDAR height: " << s_camParams.pos.z - groundZ;
    std::string str = oss.str();
    log(str);*/
}