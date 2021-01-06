#include "..\ObjectDetIncludes.h"
#include <Eigen/Core>
#include "Constants.h"

#include <time.h>

#pragma once

static char* logDir = getenv("DEEPGTAV_LOG_FILE");

//TODO rename/refactor
static bool LOGGING = DEBUG_MODE;
static bool DEBUG_LOGGING = DEBUG_MODE;


static void log(std::string str, bool override = false) {
    if ((override || LOGGING) && logDir != NULL) {
        FILE* f = fopen(logDir, "a");
		if (DEBUG_LOG_WITH_TIME) {
			time_t rawtime;
			time(&rawtime);
			fprintf(f, "[%.19s] %s", ctime(&rawtime), str.c_str());
		}
		else {
			fprintf(f, str.c_str());
		}
        fprintf(f, "\n");
        fclose(f);
    }
}

// Converts a vector 'vec' into the coordinate system with the specified unit vectors in terms of the original coordinate system
static Vector3 convertCoordinateSystem(Vector3 vec, Vector3 forwardVector, Vector3 rightVector, Vector3 upVector) {
    Vector3 newVec;

    newVec.x = vec.x*rightVector.x + vec.y*rightVector.y + vec.z*rightVector.z;
    newVec.y = vec.x*forwardVector.x + vec.y*forwardVector.y + vec.z*forwardVector.z;
    newVec.z = vec.x*upVector.x + vec.y*upVector.y + vec.z*upVector.z;

    return newVec;
}

struct BBox2D {
    float left;
    float top;
    float right;
    float bottom;

    float width() {
        return right - left;
    }

    float height() {
        return bottom - top;
    }

    float posX() {
        return left + width() / 2;
    }

    float posY() {
        return top + height() / 2;
    }

    BBox2D() :
        left(s_camParams.width),
        right(0),
        top(s_camParams.height),
        bottom(0)
    {};
};

struct VehicleToCreate {
    std::string model;
    float forward;
    float right;
    float heading;
    int color;
    int color2;

    VehicleToCreate(std::string _model, float _forward, float _right, float _heading,
        int _color, int _color2) :
        model(_model),
        forward(_forward),
        right(_right),
        heading(_heading),
        color(_color),
        color2(_color2)
    {
    }

    VehicleToCreate(){
    }
};

struct PedToCreate {
    int model;
    float forward;
    float right;
    float heading;
};

static Vector3 subtractVector(Vector3 first, Vector3 subtract) {
    Vector3 diff;
    diff.x = first.x - subtract.x;
    diff.y = first.y - subtract.y;
    diff.z = first.z - subtract.z;
    return diff;
}

static Eigen::Vector3f rotate(Eigen::Vector3f a, Eigen::Vector3f theta)
{
    Eigen::Vector3f d;

    d(0) = (float)cos((double)theta(2))*((float)cos((double)theta(1))*a(0) + (float)sin((double)theta(1))*((float)sin((double)theta(0))*a(1) + (float)cos((double)theta(0))*a(2))) - (float)sin((double)theta(2))*((float)cos((double)theta(0))*a(1) - (float)sin((double)theta(0))*a(2));
    d(1) = (float)sin((double)theta(2))*((float)cos((double)theta(1))*a(0) + (float)sin((double)theta(1))*((float)sin((double)theta(0))*a(1) + (float)cos((double)theta(0))*a(2))) + (float)cos((double)theta(2))*((float)cos((double)theta(0))*a(1) - (float)sin((double)theta(0))*a(2));
    d(2) = -(float)sin((double)theta(1))*a(0) + (float)cos((double)theta(1))*((float)sin((double)theta(0))*a(1) + (float)cos((double)theta(0))*a(2));

    return d;
}

static Eigen::Vector2f get_2d_from_3d(const Eigen::Vector3f& vertex, bool draw_debug = false) {
    // Inspired by Artur Filopowicz: Video Games for Autonomous Driving: https://github.com/arturf1/GTA5-Scripts/blob/master/Annotator.cs#L379

    if (draw_debug)
    {
        auto cam_dir_line_end = s_camParams.eigenCamDir + s_camParams.eigenPos;

        GRAPHICS::DRAW_LINE(s_camParams.eigenPos.x(), s_camParams.eigenPos.y(), s_camParams.eigenPos.z(), cam_dir_line_end.x(), cam_dir_line_end.y(), cam_dir_line_end.z(), 0, 255, 0, 200);
    }

    const Eigen::Vector3f near_clip_to_target = vertex - s_camParams.eigenClipPlaneCenter; // del

    Eigen::Vector3f camera_to_target = near_clip_to_target - s_camParams.eigenCameraCenter; // Total distance - subtracting a negative to add clip distance

    if (draw_debug)
    {
        auto cam_up_end_line = s_camParams.eigenCamUp + s_camParams.eigenPos;
        GRAPHICS::DRAW_LINE(s_camParams.eigenPos.x(), s_camParams.eigenPos.y(), s_camParams.eigenPos.z(), cam_up_end_line.x(), cam_up_end_line.y(), cam_up_end_line.z(), 100, 100, 255, 200);
        auto cam_east_end_line = s_camParams.eigenCamUp + s_camParams.eigenPos;
        GRAPHICS::DRAW_LINE(s_camParams.eigenPos.x(), s_camParams.eigenPos.y(), s_camParams.eigenPos.z(), cam_east_end_line.x(), cam_east_end_line.y(), cam_east_end_line.z(), 100, 100, 255, 200);
        auto del_draw = s_camParams.eigenPos + near_clip_to_target;
        GRAPHICS::DRAW_LINE(s_camParams.eigenClipPlaneCenter.x(), s_camParams.eigenClipPlaneCenter.y(), s_camParams.eigenClipPlaneCenter.z(), del_draw.x(), del_draw.y(), del_draw.z(), 255, 255, 100, 255);
        auto viewerDistDraw = s_camParams.eigenPos + camera_to_target;
        GRAPHICS::DRAW_LINE(s_camParams.eigenPos.x(), s_camParams.eigenPos.y(), s_camParams.eigenPos.z(), viewerDistDraw.x(), viewerDistDraw.y(), viewerDistDraw.z(), 255, 100, 100, 255);
    }
    Eigen::Vector3f camera_to_target_unit_vector = camera_to_target * (1. / camera_to_target.norm()); // Unit vector in direction of plane / line intersection

    double view_plane_dist = s_camParams.nearClip / s_camParams.eigenCamDir.dot(camera_to_target_unit_vector);

    Eigen::Vector3f up3d, forward3d, right3d;
    up3d = rotate(WORLD_UP, s_camParams.eigenRot);
    right3d = rotate(WORLD_EAST, s_camParams.eigenRot);
    forward3d = rotate(WORLD_NORTH, s_camParams.eigenRot);
    Eigen::Vector3f new_origin = s_camParams.eigenClipPlaneCenter + (s_camParams.ncHeight / 2.) * s_camParams.eigenCamUp - (s_camParams.ncWidth / 2.) * s_camParams.eigenCamEast;

    if (draw_debug)
    {
        auto top_right = new_origin + s_camParams.ncWidth * right3d;

        GRAPHICS::DRAW_LINE(new_origin.x(), new_origin.y(), new_origin.z(), top_right.x(), top_right.y(), top_right.z(), 100, 255, 100, 255);

        auto bottom_right = top_right - s_camParams.ncHeight * up3d;

        GRAPHICS::DRAW_LINE(bottom_right.x(), bottom_right.y(), bottom_right.z(), top_right.x(), top_right.y(), top_right.z(), 100, 255, 100, 255);

        auto bottom_left = bottom_right - s_camParams.ncWidth * right3d;

        GRAPHICS::DRAW_LINE(bottom_right.x(), bottom_right.y(), bottom_right.z(), bottom_left.x(), bottom_left.y(), bottom_left.z(), 100, 255, 100, 255);
        GRAPHICS::DRAW_LINE(bottom_left.x(), bottom_left.y(), bottom_left.z(), new_origin.x(), new_origin.y(), new_origin.z(), 100, 255, 100, 255);
    }
    Eigen::Vector2f ret;

    bool use_artur_method = true;

    if (use_artur_method)
    {
        Eigen::Vector3f view_plane_point = view_plane_dist * camera_to_target_unit_vector + s_camParams.eigenCameraCenter;
        view_plane_point = (view_plane_point + s_camParams.eigenClipPlaneCenter) - new_origin;
        double viewPlaneX = view_plane_point.dot(s_camParams.eigenCamEast) / s_camParams.eigenCamEast.dot(s_camParams.eigenCamEast);
        double viewPlaneZ = view_plane_point.dot(s_camParams.eigenCamUp) / s_camParams.eigenCamUp.dot(s_camParams.eigenCamUp);
        double screenX = viewPlaneX / (double)s_camParams.ncWidth;
        double screenY = -viewPlaneZ / (double)s_camParams.ncHeight;
        ret = { screenX, screenY };
    }
    else
    {
        auto intersection = s_camParams.eigenPos + view_plane_dist * camera_to_target_unit_vector;
        auto center_to_intersection = s_camParams.eigenClipPlaneCenter - intersection;
        auto x_dist = center_to_intersection.dot(right3d);
        auto z_dist = center_to_intersection.dot(up3d);
        auto screen_x = 1. - (s_camParams.ncWidth / 2. + x_dist) / s_camParams.ncWidth;
        auto screen_y = (s_camParams.ncHeight / 2. + z_dist) / s_camParams.ncHeight;
        ret = { screen_x, screen_y };
    }
    return ret;
}

static const std::vector<Eigen::Vector3f> coefficients = {
    { -0.5, -0.5,-0.5 },
{ 0.5, -0.5,-0.5 },
{ 0.5,  0.5, 0.5 },
{ -0.5,  0.5,-0.5 },
{ -0.5, -0.5, 0.5 },
{ 0.5, -0.5, 0.5 },
{ 0.5,  0.5, 0.5 },
{ -0.5,  0.5, 0.5 }
};

static Vector3 getUnitVector(Vector3 vector) {
    float distance = sqrt(SYSTEM::VDIST2(0, 0, 0, vector.x, vector.y, vector.z));
    Vector3 unitVec;
    unitVec.x = vector.x / distance;
    unitVec.y = vector.y / distance;
    unitVec.z = vector.z / distance;
    return unitVec;
}

static Vector3 subtractVecs(Vector3 first, Vector3 subtract) {
    Vector3 difference;
    difference.x = first.x - subtract.x;
    difference.y = first.y - subtract.y;
    difference.z = first.z - subtract.z;
    return difference;
}

static float dotProd(Vector3 first, Vector3 sec) {
    return (first.x * sec.x + first.y * sec.y + first.z * sec.z);
}

static Vector3 camToWorld(Vector3 relPos, Vector3 camForward, Vector3 camRight, Vector3 camUp) {
    Vector3 worldX; worldX.x = 1; worldX.y = 0; worldX.z = 0;
    Vector3 worldY; worldY.x = 0; worldY.y = 1; worldY.z = 0;
    Vector3 worldZ; worldZ.x = 0; worldZ.y = 0; worldZ.z = 1;
    Vector3 xVectorCam = convertCoordinateSystem(worldX, camForward, camRight, camUp);
    Vector3 yVectorCam = convertCoordinateSystem(worldY, camForward, camRight, camUp);
    Vector3 zVectorCam = convertCoordinateSystem(worldZ, camForward, camRight, camUp);

    Vector3 worldPos = convertCoordinateSystem(relPos, yVectorCam, xVectorCam, zVectorCam);
    worldPos.x += s_camParams.pos.x;
    worldPos.y += s_camParams.pos.y;
    worldPos.z += s_camParams.pos.z;

    return worldPos;
}