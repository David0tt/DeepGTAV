#include "DataExport.h"

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



extern "C" {
	__declspec(dllimport) int export_get_depth_buffer(void** buf);
	__declspec(dllexport) int export_get_color_buffer(void** buf);
	__declspec(dllexport) int export_get_stencil_buffer(void** buf);
}


const float VERT_CAM_FOV = 59; //In degrees




void DataExport::initialize() {
	Vector3 rotation;

	rotation = ENTITY::GET_ENTITY_ROTATION(*m_ownVehicle, 0);
	CAM::DESTROY_ALL_CAMS(TRUE);
	camera = CAM::CREATE_CAM("DEFAULT_SCRIPTED_CAMERA", TRUE);
	//if (strcmp(_vehicle, "packer") == 0) CAM::ATTACH_CAM_TO_ENTITY(camera, vehicle, 0, 2.35, 1.7, TRUE);
	//else CAM::ATTACH_CAM_TO_ENTITY(camera, vehicle, 0, CAM_OFFSET_FORWARD, CAM_OFFSET_UP, TRUE);
	CAM::SET_CAM_FOV(camera, VERT_CAM_FOV);
	CAM::SET_CAM_ACTIVE(camera, TRUE);


	// TODO CANGED
	CAM::SET_CAM_ROT(camera, rotation.x, rotation.y, rotation.z, 0);
	//CAM::SET_CAM_ROT(camera, rotation.x, rotation.y, 2, 0);

	// pos = CAM::GET_CAM_COORD()
	//Vector3 camPos;
	//camPos = CAM::GET_CAM_COORD(camera);
	//CAM::SET_CAM_COORD(camera, camPos.x + 40.0, camPos.y + 40.0, camPos.z + 40.0);


	CAM::SET_CAM_INHERIT_ROLL_VEHICLE(camera, TRUE);


}


//void DataExport::setCapturedData() {
//
//}







void DataExport::parseDatasetConfig(const Value& dc, bool setDefaults) {
	if (DEBUG_MODE) log("DataExport::parseDatasetConifg");


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

	if (dc["reward"].IsArray()) {
		if (dc["reward"][0].IsFloat() && dc["reward"][1].IsFloat()) {
			rewarder = new GeneralRewarder((char*)(GetCurrentModulePath() + "paths.xml").c_str(), dc["reward"][0].GetFloat(), dc["reward"][1].GetFloat());
			reward = true;
		}
		else if (setDefaults) reward = _REWARD_;
	}
	else if (setDefaults) reward = _REWARD_;


	if (!dc["startIndex"].IsNull()) {
		instance_index = dc["startIndex"].GetInt();
	}
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

	buildJSONObject();

}




void DataExport::buildJSONObject() {
	if (DEBUG_MODE) log("DataExport::buildJSONObject");
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
	if (time) d.AddMember("time", a, allocator);
	d.AddMember("index", 0, allocator);
	d.AddMember("focalLen", 0.0, allocator);
	d.AddMember("curPosition", a, allocator);
	d.AddMember("seriesIndex", a, allocator);
	d.AddMember("bbox2d", a, allocator);
	d.AddMember("HeightAboveGround", 0.0, allocator);
	d.AddMember("CameraAngle", a, allocator);
	d.AddMember("CameraPosition", a, allocator);

	screenCapturer = new ScreenCapturer(s_camParams.width, s_camParams.height);

}


void DataExport::setRenderingCam(Vehicle v, int height, int length) {
	if (DEBUG_MODE) log("DataExport::setRenderingCam");
	Vector3 position;
	Vector3 fVec, rVec, uVec;
	Vector3 rotation = ENTITY::GET_ENTITY_ROTATION(v, 0);
	ENTITY::GET_ENTITY_MATRIX(v, &fVec, &rVec, &uVec, &position);

	Vector3 offset;
	offset.x = 0;
	offset.y = length / 2;
	offset.z = height;
	Vector3 offsetWorld = camToWorld(offset, fVec, rVec, uVec);
	//Since it's offset need to subtract the cam position
	offsetWorld.x -= s_camParams.pos.x;
	offsetWorld.y -= s_camParams.pos.y;
	offsetWorld.z -= s_camParams.pos.z;

	GAMEPLAY::SET_TIME_SCALE(0.0f);
	GAMEPLAY::SET_GAME_PAUSED(false);
	GAMEPLAY::SET_TIME_SCALE(0.0f);

	//TODO fix pointer billiard, after making DataExport the owner of the camera
	CAM::SET_CAM_COORD(camera, position.x + offsetWorld.x + cameraPositionOffset.x, position.y + offsetWorld.y + cameraPositionOffset.y, position.z + offsetWorld.z + cameraPositionOffset.z);
	CAM::SET_CAM_ROT(camera, rotation.x + cameraRotationOffset.x, rotation.y + cameraRotationOffset.y, rotation.z + cameraRotationOffset.z, 0);


	scriptWait(0);
	GAMEPLAY::SET_GAME_PAUSED(true);

	std::ostringstream oss;
	oss << "EntityID/rotation/position: " << v << "\n" <<
		position.x << ", " << position.y << ", " << position.z <<
		"\n" << rotation.x << ", " << rotation.y << ", " << rotation.z <<
		"\nOffset: " << offset.x << ", " << offset.y << ", " << offset.z <<
		"\nOffsetworld: " << offsetWorld.x << ", " << offsetWorld.y << ", " << offsetWorld.z;
	log(oss.str(), true);
}



void DataExport::setCameraPositionAndRotation(float x, float y, float z, float rot_x, float rot_y, float rot_z) {
	cameraPositionOffset.x = x;
	cameraPositionOffset.y = y;
	cameraPositionOffset.z = z;
	cameraRotationOffset.x = rot_x;
	cameraRotationOffset.y = rot_y;
	cameraRotationOffset.z = rot_z;
}



StringBuffer DataExport::generateMessage() {
	if (DEBUG_MODE) log("DataExport::GenerateMessage");
	StringBuffer buffer;
	buffer.Clear();
	Writer<StringBuffer> writer(buffer);


	log("About to pause game");
	GAMEPLAY::SET_GAME_PAUSED(true);
	GAMEPLAY::SET_TIME_SCALE(0.0f);

	setRenderingCam((*m_ownVehicle), CAM_OFFSET_UP, CAM_OFFSET_FORWARD);

	////Can check whether camera and vehicle are aligned
	//Vector3 camRot2 = CAM::GET_CAM_ROT(camera, 0);
	//std::ostringstream oss1;
	//oss1 << "entityRotation X: " << rotation.x << " Y: " << rotation.y << " Z: " << rotation.z <<
	//    "\n camRot X: " << camRot.x << " Y: " << camRot.y << " Z: " << camRot.z <<
	//    "\n camRot2 X: " << camRot2.x << " Y: " << camRot2.y << " Z: " << camRot2.z;
	//std::string str1 = oss1.str();
	//log(str1);



	log("Script cams rendered");
	capture();
	log("Screen captured");


	int depthSize = -1;

	if (recording_active) {
		//TODO pass this through
		bool depthMap = true;

		setCamParams();
		//setColorBuffer();
		depthSize = setDepthBuffer();
		if (depthMap) setStencilBuffer();
	}


	// Setting bufffers for the Server
	// Those were the original commands from DeepGTAV
	// They each need Scenario::setVehicleList(), Scenario::setPedsList() etc.
	// Those commands would set the JSON document, e.g. d["vehicles"] = _vehicles;

	// Those functionalities have been somehow moved to ObjectDetection.cpp, e.g. ObjectDetection::setVehiclesList()
	// To fix the JSON / Server those have to be implemented again to correctly set e.g. d["vehicles"] = _vehicles
	// A quick fix would be to copy the old Definitions from DeepGTAV, but I don't know if there would be side effects
	// The correct solution would be to integrate the functionalities of e.g. Scenario::setVehiclesList() and ObjectDetection::setVehiclesList()
	// into one.
	//
	// For now i only implement the messages I need and have the rest commented out.

	// if (vehicles) setVehiclesList();
	// if (peds) setPedsList();
	// if (trafficSigns); //TODO
	//if (direction) setDirection(); // TODO add again
	if (reward) exportReward();
	if (throttle) exportThrottle();
	if (brake) exportBrake();
	if (steering) exportSteering();
	if (speed) exportSpeed();
	if (yawRate) exportYawRate();
	if (drivingMode); //TODO
	if (location) exportLocation();
	if (time) exportTime();
	exportHeightAboveGround();
	exportCameraPosition();
	exportCameraAngle();





	if (!m_pObjDet) {
		m_pObjDet.reset(new ObjectDetection());
		m_pObjDet->initCollection(s_camParams.width, s_camParams.height, false, instance_index);
	}
	if (depthSize != -1) {
		FrameObjectInfo fObjInfo = m_pObjDet->generateMessage(depth_map, m_stencilBuffer);
		m_pObjDet->exportDetections(fObjInfo);

		const std::string detections = m_pObjDet->exportDetectionsString(fObjInfo);

		// set BBoxes to send as JSON
		Document::AllocatorType& allocator = d.GetAllocator();
		Value bbox2d(kArrayType);
		
		bbox2d.SetString(StringRef(detections.c_str()));
		d["bbox2d"] = bbox2d;


		BYTE* data = screenCapturer->pixels;
		m_pObjDet->exportImage(data);

		d["index"] = fObjInfo.instanceIdx;

		////Create vehicles if it is a stationary scenario
		//createVehicles();

		//if (GENERATE_SECONDARY_PERSPECTIVES) {
		//	generateSecondaryPerspectives();
		//}

		//For testing to ensure secondary ownvehicle aligns with main perspective
		//generateSecondaryPerspective(m_pObjDet->m_ownVehicleObj);

		m_pObjDet->increaseIndex();
	}
	else {
		log("ERROR: Depth buffer could not be properly set!!!!!!!!!!!!!!!!!!!!!!", true);
	}
	GAMEPLAY::SET_GAME_PAUSED(false);
	GAMEPLAY::SET_TIME_SCALE(1.0f);

	d.Accept(writer);

	return buffer;
}



void DataExport::setCamParams() {
	if (DEBUG_MODE) log("DataExport::setCamParams");
	//These values stay the same throughout a collection period
	if (!s_camParams.init) {
		s_camParams.nearClip = CAM::GET_CAM_NEAR_CLIP(camera);
		s_camParams.farClip = CAM::GET_CAM_FAR_CLIP(camera);
		s_camParams.fov = CAM::GET_CAM_FOV(camera);
		s_camParams.ncHeight = 2 * s_camParams.nearClip * tan(s_camParams.fov / 2. * (PI / 180.)); // field of view is returned vertically
		s_camParams.ncWidth = s_camParams.ncHeight * GRAPHICS::_GET_SCREEN_ASPECT_RATIO(false);
		s_camParams.init = true;

		//if (m_recordScenario) {
		//	float gameFC = CAM::GET_CAM_FAR_CLIP(camera);
		//	std::ostringstream oss;
		//	oss << "NC, FC (gameFC), FOV: " << s_camParams.nearClip << ", " << s_camParams.farClip << " (" << gameFC << "), " << s_camParams.fov;
		//	std::string str = oss.str();
		//	log(str, true);
		//}
	}

	//These values change frame to frame
	s_camParams.theta = CAM::GET_CAM_ROT(camera, 0);
	s_camParams.pos = CAM::GET_CAM_COORD(camera);

	//std::ostringstream oss1;
	//oss1 << "\ns_camParams.pos X: " << s_camParams.pos.x << " Y: " << s_camParams.pos.y << " Z: " << s_camParams.pos.z <<
	//	"\nvehicle.pos X: " << currentPos.x << " Y: " << currentPos.y << " Z: " << currentPos.z <<
	//	"\nfar: " << s_camParams.farClip << " nearClip: " << s_camParams.nearClip << " fov: " << s_camParams.fov <<
	//	"\nrotation gameplay: " << s_camParams.theta.x << " Y: " << s_camParams.theta.y << " Z: " << s_camParams.theta.z <<
	//	"\n AspectRatio: " << GRAPHICS::_GET_SCREEN_ASPECT_RATIO(false);
	//std::string str1 = oss1.str();
	//log(str1);

	//For optimizing 3d to 2d and unit vector to 2d calculations
	s_camParams.eigenPos = Eigen::Vector3f(s_camParams.pos.x, s_camParams.pos.y, s_camParams.pos.z);
	s_camParams.eigenRot = Eigen::Vector3f(s_camParams.theta.x, s_camParams.theta.y, s_camParams.theta.z);
	s_camParams.eigenTheta = (PI / 180.0) * s_camParams.eigenRot;
	s_camParams.eigenCamDir = rotate(WORLD_NORTH, s_camParams.eigenTheta);
	s_camParams.eigenCamUp = rotate(WORLD_UP, s_camParams.eigenTheta);
	s_camParams.eigenCamEast = rotate(WORLD_EAST, s_camParams.eigenTheta);
	s_camParams.eigenClipPlaneCenter = s_camParams.eigenPos + s_camParams.nearClip * s_camParams.eigenCamDir;
	s_camParams.eigenCameraCenter = -s_camParams.nearClip * s_camParams.eigenCamDir;

	//For measuring height of camera (LiDAR) to ground plane
	/*float groundZ;
	GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(s_camParams.pos.x, s_camParams.pos.y, s_camParams.pos.z, &(groundZ), 0);

	std::ostringstream oss;
	oss << "LiDAR height: " << s_camParams.pos.z - groundZ;
	std::string str = oss.str();
	log(str);*/
}



void DataExport::capture() {
	if (DEBUG_MODE) log("DataExport::capture");
	//Time synchronization seems to be correct with 2 render calls
	CAM::RENDER_SCRIPT_CAMS(TRUE, FALSE, 0, FALSE, FALSE);
	scriptWait(0);
	CAM::RENDER_SCRIPT_CAMS(TRUE, FALSE, 0, FALSE, FALSE);
	scriptWait(0);
	CAM::RENDER_SCRIPT_CAMS(TRUE, FALSE, 0, FALSE, FALSE);
	scriptWait(0);
	screenCapturer->capture();
}


void DataExport::setRecording_active(bool x) {
	recording_active = x;
}


////Generate a secondary perspective for all nearby vehicles
//void DataExport::generateSecondaryPerspectives() {
//	for (ObjEntity v : m_pObjDet->m_nearbyVehicles) {
//		if (VEHICLE::IS_THIS_MODEL_A_CAR(v.model)) {
//			generateSecondaryPerspective(v);
//		}
//	}
//	m_pObjDet->m_nearbyVehicles.clear();
//}
//
//void DataExport::generateSecondaryPerspective(ObjEntity vInfo) {
//	setRenderingCam(vInfo.entityID, vInfo.height, vInfo.length);
//
//	//GAMEPLAY::SET_GAME_PAUSED(true);
//	capture();
//
//	setCamParams();
//	setDepthBuffer();
//	setStencilBuffer();
//
//	FrameObjectInfo fObjInfo = m_pObjDet->generateMessage(depth_map, m_stencilBuffer, vInfo.entityID);
//	m_pObjDet->exportDetections(fObjInfo, &vInfo);
//	std::string filename = m_pObjDet->getStandardFilename("image_2", ".png");
//	m_pObjDet->exportImage(screenCapturer->pixels, filename);
//
//	//GAMEPLAY::SET_GAME_PAUSED(false);
//}




/*
 ********************************************************************
 Set Individual JSON fields
 ********************************************************************
*/

void DataExport::exportThrottle() {
	d["throttle"] = getFloatValue(*m_ownVehicle, 0x92C);
}

void DataExport::exportBrake() {
	d["brake"] = getFloatValue(*m_ownVehicle, 0x930);
}

void DataExport::exportSteering() {
	d["steering"] = -getFloatValue(*m_ownVehicle, 0x924) / 0.6981317008;
}

void DataExport::exportSpeed() {
	d["speed"] = ENTITY::GET_ENTITY_SPEED(*m_ownVehicle);
}

void DataExport::exportYawRate() {
	Vector3 rates = ENTITY::GET_ENTITY_ROTATION_VELOCITY(*m_ownVehicle);
	d["yawRate"] = rates.z*180.0 / 3.14159265359;
}

void DataExport::exportLocation() {
	Document::AllocatorType& allocator = d.GetAllocator();
	Vector3 pos = ENTITY::GET_ENTITY_COORDS(*m_ownVehicle, false);
	Value location(kArrayType);
	location.PushBack(pos.x, allocator).PushBack(pos.y, allocator).PushBack(pos.z, allocator);
	d["location"] = location;
}

void DataExport::exportTime() {
	Document::AllocatorType& allocator = d.GetAllocator();
	Value time(kArrayType);
	time.PushBack(TIME::GET_CLOCK_HOURS(), allocator).PushBack(TIME::GET_CLOCK_MINUTES(), allocator).PushBack(TIME::GET_CLOCK_SECONDS(), allocator);
	d["time"] = time;
}

void DataExport::exportHeightAboveGround() {
	d["HeightAboveGround"] = ENTITY::GET_ENTITY_HEIGHT_ABOVE_GROUND(*m_ownVehicle);
}

//void DataExport::setDirection() {
//	int direction;
//	float distance;
//	Vehicle temp_vehicle;
//	Document::AllocatorType& allocator = d.GetAllocator();
//	PATHFIND::GENERATE_DIRECTIONS_TO_COORD(dir.x, dir.y, dir.z, TRUE, &direction, &temp_vehicle, &distance);
//	Value _direction(kArrayType);
//	_direction.PushBack(direction, allocator).PushBack(distance, allocator);
//	d["direction"] = _direction;
//}

void DataExport::exportReward() {
	d["reward"] = rewarder->computeReward(*m_ownVehicle);
}

void DataExport::exportCameraPosition() {
	Document::AllocatorType& allocator = d.GetAllocator();
	Vector3 pos = CAM::GET_CAM_COORD(camera);
	Value position(kArrayType);
	position.PushBack(pos.x, allocator).PushBack(pos.y, allocator).PushBack(pos.z, allocator);
	d["CameraPosition"] = position;
}

void DataExport::exportCameraAngle() {
	Document::AllocatorType& allocator = d.GetAllocator();
	Vector3 ang = CAM::GET_CAM_ROT(camera, 0);
	Value angles(kArrayType);
	angles.PushBack(ang.x, allocator).PushBack(ang.y, allocator).PushBack(ang.z, allocator);
	d["CameraAngle"] = angles;
}

//void DataExport::exportWeather() {
//	d["Weather"] = ...;
//}


void DataExport::setColorBuffer() {
	if (DEBUG_MODE) log("DataExport::setColorBuffer");
	log("Before color buffer", true);
	int size = export_get_color_buffer((void**)&color_buf);
	log("After color buffer", true);
}

void DataExport::setStencilBuffer() {
	if (DEBUG_MODE) log("DataExport::setStencilBuffer");
	log("About to get stencil buffer");
	int size = export_get_stencil_buffer((void**)&m_stencilBuffer);
	log("After getting stencil buffer");
}

int DataExport::setDepthBuffer(bool prevDepth) {
	if (DEBUG_MODE) log("DataExport::setDepthBuffer");
	log("About to get depth buffer");
	int size = export_get_depth_buffer((void**)&depth_map);

	std::ostringstream oss;
	oss << "Depth buffer size: " << size;
	log(oss.str(), true);

	log("After getting depth buffer");
	return size;
}

