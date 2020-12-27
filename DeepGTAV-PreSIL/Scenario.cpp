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
#include <sstream>
#include "AreaRoaming.h"

// #include "base64.h"

// TODO remove
const float VERT_CAM_FOV = 59; //In degrees
//Need to input the vertical FOV with GTA functions.
//90 degrees horizontal (KITTI) corresponds to 59 degrees vertical (https://www.gtaall.com/info/fov-calculator.html).
const float HOR_CAM_FOV = 90; //In degrees

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
    if (!dc["positionScenario"].IsNull()) m_positionScenario = dc["positionScenario"].GetBool();
    else if (setDefaults) m_positionScenario = _POSITION_SCENARIO_;

    if (DRIVE_SPEC_AREA && !stationaryScene) {
        int startArea = 0;
        dir.x = s_locationBounds[0][0][startArea];
        dir.y = s_locationBounds[0][1][startArea];
        dir.z = 0.f;
        x = s_locationBounds[0][0][startArea];//1,2,3,4,5,6,7,8 are all good
        y = s_locationBounds[0][1][startArea];//1-0 was last one used for 'good' data
    }

    if (stationaryScene || TRUPERCEPT_SCENARIO) {
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
        vehicles_created = false;
    }


	exporter.parseDatasetConfig(dc, setDefaults);
	//exporter.camera = &camera;
	//exporter.cameraPositionOffset = &cameraPositionOffset;
	//exporter.cameraRotationOffset = &cameraRotationOffset;
	exporter.m_ownVehicle = &m_ownVehicle;
	//exporter.instance_index = &instance_index;
}

void Scenario::buildScenario() {
	Vector3 pos;
	Hash vehicleHash;
	float heading;

    if (!stationaryScene) {
        GAMEPLAY::SET_RANDOM_SEED(std::time(NULL));
        while (!PATHFIND::_0xF7B79A50B905A30D(-8192.0f, 8192.0f, -8192.0f, 8192.0f)) WAIT(0);
        PATHFIND::GET_CLOSEST_VEHICLE_NODE_WITH_HEADING(x, y, 0, &pos, &heading, 0, 0, 0);
    }

	ENTITY::DELETE_ENTITY(&m_ownVehicle);
	vehicleHash = GAMEPLAY::GET_HASH_KEY((char*)_vehicle);
	STREAMING::REQUEST_MODEL(vehicleHash);
	while (!STREAMING::HAS_MODEL_LOADED(vehicleHash)) WAIT(0);
    if (stationaryScene) {
        pos.x = x;
        pos.y = y;
        pos.z = z;
        heading = startHeading;
        std::ostringstream oss;
        oss << "Start heading: " << startHeading;
        std::string str = oss.str();
        log(str);
        vehicles_created = false;
    }
	m_ownVehicle = VEHICLE::CREATE_VEHICLE(vehicleHash, pos.x, pos.y, pos.z, heading, FALSE, FALSE);
	VEHICLE::SET_VEHICLE_ON_GROUND_PROPERLY(m_ownVehicle);

	while (!ENTITY::DOES_ENTITY_EXIST(ped)) {
		ped = PLAYER::PLAYER_PED_ID();
        WAIT(0);
	}

	player = PLAYER::PLAYER_ID();
	PLAYER::START_PLAYER_TELEPORT(player, pos.x, pos.y, pos.z, heading, 0, 0, 0);
	while (PLAYER::IS_PLAYER_TELEPORT_ACTIVE()) WAIT(0);

	PED::SET_PED_INTO_VEHICLE(ped, m_ownVehicle, -1);
	STREAMING::SET_MODEL_AS_NO_LONGER_NEEDED(vehicleHash);

	TIME::SET_CLOCK_TIME(hour, minute, 0);

	GAMEPLAY::SET_WEATHER_TYPE_NOW_PERSIST((char*)_weather);

	exporter.initialize();

    if (stationaryScene) {
        _setSpeed = 0;
    }

    CAM::RENDER_SCRIPT_CAMS(TRUE, FALSE, 0, TRUE, TRUE);

	AI::CLEAR_PED_TASKS(ped);
	if (_drivingMode >= 0 && !stationaryScene && !m_positionScenario) {
        if (DRIVE_SPEC_AREA && !START_SPEC_AREA) {
            AI::TASK_VEHICLE_DRIVE_TO_COORD(ped, m_ownVehicle, dir.x, dir.y, dir.z, _setSpeed, Any(1.f), vehicleHash, _drivingMode, 50.f, true);
        }
        else {
            AI::TASK_VEHICLE_DRIVE_WANDER(ped, m_ownVehicle, _setSpeed, _drivingMode);
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

	running = true;
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

	//Parse options
	srand(std::time(NULL));
	parseScenarioConfig(sc, false);
	parseDatasetConfig(dc, false);

	//Build scenario
	buildScenario();

	lastSafetyCheck = std::clock();
}

void Scenario::run() {
	if (running) {

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

                AI::TASK_VEHICLE_DRIVE_TO_COORD(ped, m_ownVehicle, dir.x, dir.y, dir.z, _setSpeed, Any(1.f),
                    GAMEPLAY::GET_HASH_KEY((char*)_vehicle), _drivingMode, 1.f, true);
            }
            else if (!in_bounds(currentPos.x, currentPos.y, m_startArea, m_polyGrid))
            {
                std::vector<std::pair<float, float>> new_points = generate_n_random_points(
                    m_startArea, m_polyGrid, 1, 100, { { currentPos.x , currentPos.y } });
                dir.x = new_points[0].first;
                dir.y = new_points[0].second;

                AI::TASK_VEHICLE_DRIVE_TO_COORD(ped, m_ownVehicle, dir.x, dir.y, dir.z, _setSpeed, Any(1.f),
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
                exporter.setCamParams();
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
			VEHICLE::SET_VEHICLE_TYRES_CAN_BURST(m_ownVehicle, FALSE);
			VEHICLE::SET_VEHICLE_WHEELS_CAN_BREAK(m_ownVehicle, FALSE);
			VEHICLE::SET_VEHICLE_HAS_STRONG_AXLES(m_ownVehicle, TRUE);

			VEHICLE::SET_VEHICLE_CAN_BE_VISIBLY_DAMAGED(m_ownVehicle, FALSE);
			ENTITY::SET_ENTITY_INVINCIBLE(m_ownVehicle, TRUE);
			ENTITY::SET_ENTITY_PROOFS(m_ownVehicle, 1, 1, 1, 1, 1, 1, 1, 1);

			// Player invincible
			PLAYER::SET_PLAYER_INVINCIBLE(player, TRUE);

			// Driving characteristics
			PED::SET_DRIVER_AGGRESSIVENESS(ped, 0.0);
			PED::SET_DRIVER_ABILITY(ped, 100.0);
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
	return exporter.generateMessage();
}



//TODO remove
void Scenario::setRecording_active(bool x) {
	exporter.setRecording_active(x);
}

void Scenario::goToLocation(float x, float y, float z, float speed) {
	Hash vehicleHash;
	vehicleHash = GAMEPLAY::GET_HASH_KEY((char*)_vehicle);

	AI::CLEAR_PED_TASKS(ped);
	AI::TASK_VEHICLE_DRIVE_TO_COORD(ped, m_ownVehicle, x, y, z, speed, Any(1.f), vehicleHash, _drivingMode, 2.f, true);
}

void Scenario::teleportToLocation(float x, float y, float z) {
	//Hash vehicleHash;
	//float heading;

	//ENTITY::DELETE_ENTITY(&m_ownVehicle);
	//vehicleHash = GAMEPLAY::GET_HASH_KEY((char*)_vehicle);
	//STREAMING::REQUEST_MODEL(vehicleHash);

	//m_ownVehicle = VEHICLE::CREATE_VEHICLE(vehicleHash, x, y, z, heading, FALSE, FALSE);
	////VEHICLE::SET_VEHICLE_ON_GROUND_PROPERLY(m_ownVehicle);

	//while (!ENTITY::DOES_ENTITY_EXIST(ped)) {
	//	ped = PLAYER::PLAYER_PED_ID();
	//	WAIT(0);
	//}

	//player = PLAYER::PLAYER_ID();


	//PLAYER::START_PLAYER_TELEPORT(player, x, y, z, heading, 0, 0, 0);
	//while (PLAYER::IS_PLAYER_TELEPORT_ACTIVE()) WAIT(0);

	//PED::SET_PED_INTO_VEHICLE(ped, m_ownVehicle, -1);
	//STREAMING::SET_MODEL_AS_NO_LONGER_NEEDED(vehicleHash);


	//ENTITY::SET_ENTITY_COORDS(m_ownVehicle, x, y, z, 0, 0, 1);
	ENTITY::SET_ENTITY_COORDS_NO_OFFSET(m_ownVehicle, x, y, z, 0, 0, 1);
	ENTITY::SET_ENTITY_HAS_GRAVITY(m_ownVehicle, false);

	bool isStrong = true;
	int forceFlags = 0u;
	if (isStrong) forceFlags |= (1u << 0);
	//Set first bit	
	//LAST BOOL HAS TO BE FALSE (SCRIPT STOPS RUNNING)
	bool isDirRel = false;
	bool isMassRel = false;
	ENTITY::APPLY_FORCE_TO_ENTITY_CENTER_OF_MASS(m_ownVehicle, forceFlags, 0, 0, 5000, FALSE, isDirRel, isMassRel, FALSE);
	//ENTITY::APPLY_FORCE_TO_ENTITY_CENTER_OF_MASS(m_ownVehicle, 0, 0, 0, 50, false, BOOL p6, BOOL p7, BOOL p8);
	//ENTITY::APPLY_FORCE_TO_ENTITY(Entity entity, int forceType, float x, float y, float z, float xRot, float yRot, float zRot, int p8, BOOL isRel, BOOL ignoreUpVec, BOOL p11, BOOL p12, BOOL p13);

	// TODO try to set vehicle controls (e.g. rotors half speed at start

	//SIMULATE_PLAYER_INPUT_GAIT(Player player, float amount, int gaitType, float speed, BOOL p4, BOOL p5) { invoke<Void>(0x477D5D63E63ECA5D, player, amount, gaitType, speed, p4, p5); } // 0x477D5D63E63ECA5D 0x0D77CC34
	//static void RESET_PLAYER_INPUT_GAIT(Player player)

}

void Scenario::setCameraPositionAndRotation(float x, float y, float z, float rot_x, float rot_y, float rot_z) {
	exporter.setCameraPositionAndRotation(x, y, z, rot_x, rot_y, rot_z);
}


static int bike_num = 0;

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

    //if (VEHICLE::IS_THIS_MODEL_A_BICYCLE(vehicleHash) || VEHICLE::IS_THIS_MODEL_A_BIKE(vehicleHash)) {
    //    log("Trying to set ped on bike", true);
    //    Hash hash = 0x505603B9;// GAMEPLAY::GET_HASH_KEY(const_cast<char*>(model));
    //    STREAMING::REQUEST_MODEL(hash);
    //    Ped tempP = PED::CREATE_PED(4, hash, pos.x, pos.y, pos.z, heading, FALSE, FALSE);
    //    WAIT(0);
    //    if (bike_num == 0) {
    //        bike_num++;
    //        AI::TASK_ENTER_VEHICLE(tempP, tempV, 0, -1, 2.0f, 16, 0);
    //    }
    //    else {
    //        AI::TASK_VEHICLE_DRIVE_WANDER(tempP, tempV, 2.0f, 16777216);
    //    }
    //}

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
    ENTITY::SET_ENTITY_AS_NO_LONGER_NEEDED(&temp);
}

void Scenario::createVehicles() {
    setPosition();
    if ((stationaryScene || TRUPERCEPT_SCENARIO) && !vehicles_created) {
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


void Scenario::setWeather(const char* weather) {
	GAMEPLAY::SET_WEATHER_TYPE_NOW_PERSIST((char*)weather);
}

void Scenario::setClockTime(int hour, int minute, int second) {
	TIME::SET_CLOCK_TIME(hour, minute, second);
}

//Saves the position and vectors of the capture vehicle
void Scenario::setPosition() {
    //NOTE: The forward and right vectors are swapped (compared to native function labels) to keep consistency with coordinate system
    ENTITY::GET_ENTITY_MATRIX(m_ownVehicle, &currentForwardVector, &currentRightVector, &currentUpVector, &currentPos); //Blue or red pill
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

