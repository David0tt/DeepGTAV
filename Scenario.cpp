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

const float PI = 3.14159265359;
const float VERT_CAM_FOV = 59; //In degrees
//Need to input the vertical FOV with GTA functions.
//90 degrees horizontal (KITTI) corresponds to 59 degrees vertical (https://www.gtaall.com/info/fov-calculator.html).
const float HOR_CAM_FOV = 90; //In degrees

const float CAM_OFFSET_FORWARD = 0.5;
const float CAM_OFFSET_UP = 0.8;

bool STATIONARY_SCEN = true;

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
	}
	else if (setDefaults) {
		x = 5000 * ((float)rand() / RAND_MAX) - 2500;
		y = 8000 * ((float)rand() / RAND_MAX) - 2000;
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
	else if (setDefaults) _vehicle = vehicleList[rand() % 3];

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
	
	if (!dc["frame"].IsNull()) {
		if (!dc["frame"][0].IsNull()) width = dc["frame"][0].GetInt();
		else if (setDefaults) width = _WIDTH_;

		if (!dc["frame"][1].IsNull()) height = dc["frame"][1].GetInt();
		else if (setDefaults) height = _HEIGHT_;
	}
	else if (setDefaults) {
		width = _WIDTH_;
		height = _HEIGHT_;
	}	

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
    if (!dc["pointclouds"].IsNull()) pointclouds = dc["pointclouds"].GetBool();
    else if (setDefaults) pointclouds = _POINTCLOUDS_;
    if (!dc["offscreen"].IsNull()) offscreen = dc["offscreen"].GetBool();
    else if (setDefaults) offscreen = _OFFSCREEN_;
    if (!dc["showBoxes"].IsNull()) showBoxes = dc["showBoxes"].GetBool();
    else if (setDefaults) showBoxes = _SHOWBOXES_;

    //Create camera intrinsics matrix
    calcCameraIntrinsics();

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

	screenCapturer = new ScreenCapturer(width, height);
}

void Scenario::buildScenario() {
	Vector3 pos, rotation;
	Hash vehicleHash;
	float heading;

	GAMEPLAY::SET_RANDOM_SEED(std::time(NULL));
	while (!PATHFIND::_0xF7B79A50B905A30D(-8192.0f, 8192.0f, -8192.0f, 8192.0f)) WAIT(0);
	PATHFIND::GET_CLOSEST_VEHICLE_NODE_WITH_HEADING(x, y, 0, &pos, &heading, 0, 0, 0);

	ENTITY::DELETE_ENTITY(&vehicle);
	vehicleHash = GAMEPLAY::GET_HASH_KEY((char*)_vehicle);
	STREAMING::REQUEST_MODEL(vehicleHash);
	while (!STREAMING::HAS_MODEL_LOADED(vehicleHash)) WAIT(0);
    if (STATIONARY_SCEN) {
        pos.x = 1000;
        pos.y = 0;
        pos.z = 0;
        heading = 0;
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

	rotation = ENTITY::GET_ENTITY_ROTATION(vehicle, 1);
	CAM::DESTROY_ALL_CAMS(TRUE);
	camera = CAM::CREATE_CAM("DEFAULT_SCRIPTED_CAMERA", TRUE);
	if (strcmp(_vehicle, "packer") == 0) CAM::ATTACH_CAM_TO_ENTITY(camera, vehicle, 0, 2.35, 1.7, TRUE);
	else CAM::ATTACH_CAM_TO_ENTITY(camera, vehicle, 0, CAM_OFFSET_FORWARD, CAM_OFFSET_UP, TRUE);
	CAM::SET_CAM_FOV(camera, VERT_CAM_FOV);
	CAM::SET_CAM_ACTIVE(camera, TRUE);
	CAM::SET_CAM_ROT(camera, rotation.x, rotation.y, rotation.z, 1);
	CAM::SET_CAM_INHERIT_ROLL_VEHICLE(camera, TRUE);

    if (STATIONARY_SCEN) {
        _setSpeed = 0;
    }

    CAM::RENDER_SCRIPT_CAMS(TRUE, FALSE, 0, TRUE, TRUE);

	AI::CLEAR_PED_TASKS(ped);
	if (_drivingMode >= 0 && !STATIONARY_SCEN) AI::TASK_VEHICLE_DRIVE_WANDER(ped, vehicle, _setSpeed, _drivingMode);
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
		std::clock_t now = std::clock();

		Vector3 rotation = ENTITY::GET_ENTITY_ROTATION(vehicle, 1);
		CAM::SET_CAM_ROT(camera, rotation.x, rotation.y, rotation.z, 1);

		if (_drivingMode < 0) {
			CONTROLS::_SET_CONTROL_NORMAL(27, 71, currentThrottle); //[0,1]
			CONTROLS::_SET_CONTROL_NORMAL(27, 72, currentBrake); //[0,1]
			CONTROLS::_SET_CONTROL_NORMAL(27, 59, currentSteering); //[-1,1]
		}
		
		float delay = ((float)(now - lastSafetyCheck)) / CLOCKS_PER_SEC;
		if (delay > 10) {
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

            //Setup LiDAR before collecting
            setupLiDAR();

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
	
	screenCapturer->capture();

    setIndex();
    setPosition();
	if (vehicles) setVehiclesList();
	if (peds) setPedsList();
	if (trafficSigns); //TODO
	if (direction) setDirection();
	if (reward) setReward();
	if (throttle) setThrottle();
	if (brake) setBrake();
	if (steering) setSteering();
	if (speed) setSpeed();
	if (yawRate) setYawRate();
	if (drivingMode); //TODO
	if (location) setLocation();
	if (time) setTime();
    if (pointclouds && lidar_initialized) collectLiDAR();
    setFocalLength();

	d.Accept(writer);

	return buffer;
}

//Returns the angle between a relative position vector and the forward vector (rotated about up axis)
float Scenario::observationAngle(Vector3 position) {
    float x1 = currentRightVector.x;
    float y1 = currentRightVector.y;
    float z1 = currentRightVector.z;
    float x2 = position.x;
    float y2 = position.y;
    float z2 = position.z;
    float xn = currentUpVector.x;
    float yn = currentUpVector.y;
    float zn = currentUpVector.z;

    float dot = x1 * x2 + y1 * y2 + z1 * z2;
    float det = x1 * y2*zn + x2 * yn*z1 + xn * y1*z2 - z1 * y2*xn - z2 * yn*x1 - zn * y1*x2;
    float observationAngle = atan2(det, dot);

    if (DEBUG_LOGGING) {
        std::ostringstream oss;
        oss << "Forward is: " << x1 << ", " << y1 << ", " << z1 <<
            "\nNormal is: " << x2 << ", " << y2 << ", " << z2 <<
            "\nPosition is: " << position.x << ", " << position.y << ", " << position.z << " and angle is: " << observationAngle;
        std::string str = oss.str();
        log(str);
    }

    return observationAngle;
}

void Scenario::drawVectorFromPosition(Vector3 vector, int blue, int green) {
    GRAPHICS::DRAW_LINE(currentPos.x, currentPos.y, currentPos.z, vector.x*1000 + currentPos.x, vector.y * 1000 + currentPos.y, vector.z * 1000 + currentPos.z, 0, green, blue, 200);
    WAIT(0);
}

//Saves the position and vectors of the capture vehicle
void Scenario::setPosition() {
    //NOTE: The forward and right vectors are swapped (compared to native function labels) to keep consistency with coordinate system
    ENTITY::GET_ENTITY_MATRIX(vehicle, &currentForwardVector, &currentRightVector, &currentUpVector, &currentPos); //Blue or red pill
}

bool Scenario::getEntityVector(Value &_entity, Document::AllocatorType& allocator, int entityID, Hash model, int classid) {
    bool success = false;

    Vector3 FUR; //Front Upper Right
    Vector3 BLL; //Back Lower Lelft
    Vector3 dim; //Vehicle dimensions
    Vector3 upVector, rightVector, forwardVector, position; //Vehicle position
    Vector3 min;
    Vector3 max;
    Vector3 speedVector;
    float heading, speed;

    bool isOnScreen = ENTITY::IS_ENTITY_ON_SCREEN(entityID);
    if (offscreen || isOnScreen) {
        //Check if it is in screen
        ENTITY::GET_ENTITY_MATRIX(entityID, &forwardVector, &rightVector, &upVector, &position); //Blue or red pill
        float distance = sqrt(SYSTEM::VDIST2(currentPos.x, currentPos.y, currentPos.z, position.x, position.y, position.z));
        if (distance < 150) {
            if (ENTITY::HAS_ENTITY_CLEAR_LOS_TO_ENTITY(vehicle, entityID, 19)) {
                success = true;
                //Check if we see it (not occluded)
                GAMEPLAY::GET_MODEL_DIMENSIONS(model, &min, &max);

                speedVector = ENTITY::GET_ENTITY_SPEED_VECTOR(entityID, false);
                speed = ENTITY::GET_ENTITY_SPEED(entityID);
                if (speed > 0) {
                    heading = GAMEPLAY::GET_HEADING_FROM_VECTOR_2D(speedVector.x - currentForwardVector.x, speedVector.y - currentForwardVector.y);
                }
                else {
                    heading = GAMEPLAY::GET_HEADING_FROM_VECTOR_2D(forwardVector.x - currentForwardVector.x, forwardVector.y - currentForwardVector.y);
                }

                //Calculate size
                dim.x = 0.5*(max.x - min.x);
                dim.y = 0.5*(max.y - min.y);
                dim.z = 0.5*(max.z - min.z);

                //Amount dimensions are offcenter
                Vector3 offcenter;
                offcenter.x = 0;// max.x + min.x;
                offcenter.y = 0;// max.y + min.y;
                offcenter.z = min.z; //KITTI position is at object ground plane

                Vector3 offcenterPosition = convertCoordinateSystem(offcenter, forwardVector, rightVector, upVector);

                //Seems like the offcenter is not actually correct
                //Update object position to be consistent with KITTI (symmetrical dimensions except for z which is ground)
                position.x = position.x + offcenterPosition.x;
                position.y = position.y + offcenterPosition.y;
                position.z = position.z + offcenterPosition.z;

                //Kitti dimensions
                float kittiHeight = 2 * dim.z;
                float kittiWidth = 2 * dim.x;
                float kittiLength = 2 * dim.y;
                
                if (abs(offcenter.y) > 0.5 && classid == 0) {
                    std::ostringstream oss2;
                    oss2 << "Instance Index: " << instance_index << " Dimensions are: " << dim.x << ", " << dim.y << ", " << dim.z;
                    oss2 << "\nMax: " << max.x << ", " << max.y << ", " << max.z;
                    oss2 << "\nMin: " << min.x << ", " << min.y << ", " << min.z;
                    oss2 << "\noffset: " << offcenter.x << ", " << offcenter.y << ", " << offcenter.z;
                    std::string str2 = oss2.str();
                    log(str2);
                }

                FUR.x = position.x + dim.y*forwardVector.x + dim.x*rightVector.x + dim.z*upVector.x;
                FUR.y = position.y + dim.y*forwardVector.y + dim.x*rightVector.y + dim.z*upVector.y;
                FUR.z = position.z + dim.y*forwardVector.z + dim.x*rightVector.z + dim.z*upVector.z;
                //GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(FUR.x, FUR.y, FUR.z, &(FUR.z), 0);
                //FUR.z += 2 * dim.z;

                BLL.x = position.x - dim.y*forwardVector.x - dim.x*rightVector.x - dim.z*upVector.x;
                BLL.y = position.y - dim.y*forwardVector.y - dim.x*rightVector.y - dim.z*upVector.y;
                BLL.z = position.z - dim.y*forwardVector.z - dim.x*rightVector.z - dim.z*upVector.z;
                //GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(BLL.x, BLL.y, 1000.0, &(BLL.z), 0);

                Vector3 relativePos;
                relativePos.x = position.x - currentPos.x;
                relativePos.y = position.y - currentPos.y;
                relativePos.z = position.z - currentPos.z;

                Vector3 kittiForwardVector = convertCoordinateSystem(forwardVector, currentForwardVector, currentRightVector, currentUpVector);
                float angle = -atan2(kittiForwardVector.y, kittiForwardVector.x);

                relativePos = convertCoordinateSystem(relativePos, currentForwardVector, currentRightVector, currentUpVector);

                //Update object position to be consistent with KITTI (symmetrical dimensions except for z which is ground)
                relativePos.y = relativePos.y - CAM_OFFSET_FORWARD;
                relativePos.z = relativePos.z - CAM_OFFSET_UP;

                //Convert to KITTI camera coordinates
                Vector3 kittiPos;
                kittiPos.x = relativePos.x;
                kittiPos.y = -relativePos.z;
                kittiPos.z = relativePos.y;

                Value _vector(kArrayType);
                _vector.PushBack(FUR.x - currentPos.x, allocator).PushBack(FUR.y - currentPos.y, allocator).PushBack(FUR.z - currentPos.z, allocator);
                _entity.AddMember("FUR", _vector, allocator);
                _vector.SetArray();
                _vector.PushBack(BLL.x - currentPos.x, allocator).PushBack(BLL.y - currentPos.y, allocator).PushBack(BLL.z - currentPos.z, allocator);
                _entity.AddMember("BLL", _vector, allocator).AddMember("speed", speed, allocator).AddMember("heading", heading, allocator).AddMember("classID", classid, allocator);
                _entity.AddMember("offscreen", offscreen, allocator);
                _vector.SetArray();
                _vector.PushBack(kittiHeight, allocator).PushBack(kittiWidth, allocator).PushBack(kittiLength, allocator);
                _entity.AddMember("dimensions", _vector, allocator);
                _vector.SetArray();
                _vector.PushBack(offcenter.z, allocator).PushBack(offcenter.x, allocator).PushBack(offcenter.y, allocator);
                _entity.AddMember("offcenter", _vector, allocator);
                _vector.SetArray();
                _vector.PushBack(kittiPos.x, allocator).PushBack(kittiPos.y, allocator).PushBack(kittiPos.z, allocator);
                _entity.AddMember("location", _vector, allocator);
                _entity.AddMember("rotation_y", angle, allocator);
                _entity.AddMember("alpha", angle, allocator);
                _entity.AddMember("entityID", entityID, allocator);
                _entity.AddMember("distance", distance, allocator);

                drawBoxes(BLL, FUR, dim, upVector, rightVector, forwardVector, position, 1);
            }
        }
    }

    return success;
}

void Scenario::setVehiclesList() {
    log("Setting vehicles list.");
	const int ARR_SIZE = 1024;
	Vehicle vehicles[ARR_SIZE];
	Value _vehicles(kArrayType);
	Document::AllocatorType& allocator = d.GetAllocator();

    Hash model;
	int classid;

	int count = worldGetAllVehicles(vehicles, ARR_SIZE);
	for (int i = 0; i < count; i++) {
		if (vehicles[i] == vehicle) continue; //Don't process own car!

        model = ENTITY::GET_ENTITY_MODEL(vehicles[i]);
        if (VEHICLE::IS_THIS_MODEL_A_CAR(model)) classid = 0;
        else if (VEHICLE::IS_THIS_MODEL_A_BIKE(model)) classid = 1;
        else if (VEHICLE::IS_THIS_MODEL_A_BICYCLE(model)) classid = 2;
        else if (VEHICLE::IS_THIS_MODEL_A_QUADBIKE(model)) classid = 3;
        else if (VEHICLE::IS_THIS_MODEL_A_BOAT(model)) classid = 4;
        else if (VEHICLE::IS_THIS_MODEL_A_PLANE(model)) classid = 5;
        else if (VEHICLE::IS_THIS_MODEL_A_HELI(model)) classid = 6;
        else if (VEHICLE::IS_THIS_MODEL_A_TRAIN(model)) classid = 7;
        else if (VEHICLE::_IS_THIS_MODEL_A_SUBMERSIBLE(model)) classid = 8;
        else classid = 9; //unknown (ufo?)

        Value _vehicle(kObjectType);
        bool success = getEntityVector(_vehicle, allocator, vehicles[i], model, classid);
        if (success) {
            _vehicles.PushBack(_vehicle, allocator);
        }
	}
			
	d["vehicles"] = _vehicles;
}

void Scenario::setPedsList(){
    log("Setting peds list.");
	const int ARR_SIZE = 1024;
	Ped peds[ARR_SIZE];
	Value _peds(kArrayType);
	Document::AllocatorType& allocator = d.GetAllocator();

	Hash model;
	int classid;

	int count = worldGetAllPeds(peds, ARR_SIZE);
	for (int i = 0; i < count; i++) {
		if (PED::IS_PED_IN_ANY_VEHICLE(peds[i], TRUE)) continue; //Don't process peds in vehicles!

        if (PED::GET_PED_TYPE(peds[i]) == 28) classid = 11; //animal
        else classid = 10;

        model = ENTITY::GET_ENTITY_MODEL(peds[i]);

        Value _ped(kObjectType);
        bool success = getEntityVector(_ped, allocator, peds[i], model, classid);
        if (success) {
            _peds.PushBack(_ped, allocator);
        }
	}		
	d["peds"] = _peds;
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

void Scenario::setSpeed(){
	d["speed"] = ENTITY::GET_ENTITY_SPEED(vehicle);
}

void Scenario::setYawRate(){
	Vector3 rates = ENTITY::GET_ENTITY_ROTATION_VELOCITY(vehicle);
	d["yawRate"] = rates.z*180.0 / 3.14159265359;
}

void Scenario::setLocation(){
	Document::AllocatorType& allocator = d.GetAllocator();
	Vector3 pos = ENTITY::GET_ENTITY_COORDS(vehicle, false);
	Value location(kArrayType);
	location.PushBack(pos.x, allocator).PushBack(pos.y, allocator).PushBack(pos.z, allocator);
	d["location"] = location;
}

void Scenario::setTime(){
	d["time"] = TIME::GET_CLOCK_HOURS();
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

void Scenario::setupLiDAR() {
    if (pointclouds && !lidar_initialized) //flag if activate the LiDAR
    {
        lidar.Init3DLiDAR_FOV(120.0f, 359.9f, 0.09f, 26.9f, 0.420f);
        lidar.AttachLiDAR2Camera(camera, ped);
        lidar_initialized = true;
    }
}

void Scenario::createVehicles() {
    setPosition();
    if (STATIONARY_SCEN && !vehicles_created) {
        log("Creating vehicle");
        Hash vehicleHash = GAMEPLAY::GET_HASH_KEY("benson");
        Vector3 pos;
        pos.x = currentPos.x + currentForwardVector.x * 20 + currentRightVector.x * 0;
        pos.y = currentPos.y + currentForwardVector.y * 20 + currentRightVector.y * 0;
        pos.z = currentPos.z + currentForwardVector.z * 20;
        STREAMING::REQUEST_MODEL(vehicleHash);
        while (!STREAMING::HAS_MODEL_LOADED(vehicleHash)) WAIT(0);
        Vehicle tempV = VEHICLE::CREATE_VEHICLE(vehicleHash, pos.x, pos.y, pos.z, 60, FALSE, FALSE);
        WAIT(0);
        VEHICLE::SET_VEHICLE_ON_GROUND_PROPERLY(vehicle);
        ENTITY::SET_ENTITY_AS_NO_LONGER_NEEDED(&tempV);
        vehicles_created = true;
    }
}

void Scenario::collectLiDAR() {
    /**********Debug code for trying to get LiDAR to work reliably past 30m
    //Collect array of nearby peds/vehicles so that LiDAR hits them
    const int numElements = 16;
    const int arrSize = numElements * 2 + 2;  //Start at index 2, and the odd elements are padding
    Ped pedsTemp[arrSize];
    Vehicle vehiclesTemp[arrSize];
    //0 index is the size of the array
    pedsTemp[0] = numElements;
    vehiclesTemp[0] = numElements;

    int count = PED::GET_PED_NEARBY_VEHICLES(PLAYER::PLAYER_PED_ID(), pedsTemp);

    for (int i = 0; i < count; ++i)
    {
        int offsettedID = i * 2 + 2;
        if (ENTITY::DOES_ENTITY_EXIST(pedsTemp[offsettedID]))
        {
            ENTITY::SET_PED_AS_NO_LONGER_NEEDED(&pedsTemp[offsettedID]);
            //ENTITY::SET_ENTITY_CAN_BE_DAMAGED(pedsTemp[offsettedID],false);
            //PED::EXPLODE_PED_HEAD(peds[offsettedID], 0x5FC3C11);
        }
    }

    count = PED::GET_PED_NEARBY_VEHICLES(PLAYER::PLAYER_PED_ID(), vehiclesTemp);

    for (int i = 0; i < count; ++i)
    {
        int offsettedID = i * 2 + 2;
        if (ENTITY::DOES_ENTITY_EXIST(vehiclesTemp[offsettedID]))
        {
            ENTITY::SET_VEHICLE_AS_NO_LONGER_NEEDED(&vehiclesTemp[offsettedID]);
            //ENTITY::SET_ENTITY_CAN_BE_DAMAGED(vehiclesTemp[offsettedID], false);
            //PED::EXPLODE_PED_HEAD(peds[offsettedID], 0x5FC3C11);
        }
    }
    */

    lidar.updateCurrentPosition(currentForwardVector, currentRightVector, currentUpVector);
    float * pointCloud = lidar.GetPointClouds(m_pointCloudSize);
    
    char format[] = "E:\\data\\velodyne\\%06d.bin";
    char filename[sizeof format + 100];
    sprintf(filename, format, instance_index);

    std::ofstream ofile(filename, std::ios::binary);
    ofile.write((char*)pointCloud, 4 * sizeof(float)*m_pointCloudSize);

    ++instance_index;
}

void Scenario::setIndex() {
    d["index"] = instance_index;
}

//Camera intrinsics are focal length, and center in horizontal (x) and vertical (y)
void Scenario::calcCameraIntrinsics() {
    float f = width / (2 * tan(HOR_CAM_FOV * PI / 360));
    float cx = width / 2;
    float cy = height / 2;

    intrinsics[0] = f;
    intrinsics[1] = cx;
    intrinsics[2] = cy;

    if (DEBUG_LOGGING) {
        std::ostringstream oss;
        oss << "Focal length is: " << f;
        std::string str = oss.str();
        log(str);
    }

    d["focalLen"] = f;
}

//Camera intrinsics are focal length, and center in horizontal (x) and vertical (y)
void Scenario::setFocalLength() {
    d["focalLen"] = intrinsics[0];
}

void Scenario::drawBoxes(Vector3 BLL, Vector3 FUR, Vector3 dim, Vector3 upVector, Vector3 rightVector, Vector3 forwardVector, Vector3 position, int colour) {
    //log("Inside draw boxes");
    if (showBoxes) {
        //log("Inside show boxes");
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