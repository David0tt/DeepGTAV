#include "..\ObjectDetIncludes.h"
#include <Eigen/Core>
#include "Constants.h"

#pragma once

static char* logDir = getenv("DEEPGTAV_LOG_FILE");
static bool LOGGING = false;
static bool DEBUG_LOGGING = false;
static void log(std::string str, bool override = false) {
    if ((override || LOGGING) && logDir != NULL) {
        FILE* f = fopen(logDir, "a");
        fprintf(f, str.c_str());
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

static Eigen::Vector2f get_2d_from_3d(const Eigen::Vector3f& vertex, const Eigen::Vector3f& cam_coords, const Eigen::Vector3f& cam_rotation, float cam_near_clip, float cam_field_of_view, bool draw_debug = false) {
    // Inspired by Artur Filopowicz: Video Games for Autonomous Driving: https://github.com/arturf1/GTA5-Scripts/blob/master/Annotator.cs#L379

    Eigen::Vector3f theta = (PI / 180.0) * cam_rotation;
    auto cam_dir = rotate(WORLD_NORTH, theta);
    if (draw_debug)
    {
        auto cam_dir_line_end = cam_dir + cam_coords;

        GRAPHICS::DRAW_LINE(cam_coords.x(), cam_coords.y(), cam_coords.z(), cam_dir_line_end.x(), cam_dir_line_end.y(), cam_dir_line_end.z(), 0, 255, 0, 200);
    }
    auto clip_plane_center = cam_coords + cam_near_clip * cam_dir;
    auto camera_center = -cam_near_clip * cam_dir;
    auto near_clip_height = 2 * cam_near_clip * tan(cam_field_of_view / 2. * (3.14159 / 180.)); // field of view is returned vertically
    auto near_clip_width = near_clip_height * GRAPHICS::_GET_SCREEN_ASPECT_RATIO(false);

    const Eigen::Vector3f cam_up = rotate(WORLD_UP, theta);
    const Eigen::Vector3f cam_east = rotate(WORLD_EAST, theta);
    const Eigen::Vector3f near_clip_to_target = vertex - clip_plane_center; // del

    Eigen::Vector3f camera_to_target = near_clip_to_target - camera_center; // Total distance - subtracting a negative to add clip distance

    if (draw_debug)
    {
        auto cam_up_end_line = cam_up + cam_coords;
        GRAPHICS::DRAW_LINE(cam_coords.x(), cam_coords.y(), cam_coords.z(), cam_up_end_line.x(), cam_up_end_line.y(), cam_up_end_line.z(), 100, 100, 255, 200);
        auto cam_east_end_line = cam_up + cam_coords;
        GRAPHICS::DRAW_LINE(cam_coords.x(), cam_coords.y(), cam_coords.z(), cam_east_end_line.x(), cam_east_end_line.y(), cam_east_end_line.z(), 100, 100, 255, 200);
        auto del_draw = cam_coords + near_clip_to_target;
        GRAPHICS::DRAW_LINE(clip_plane_center.x(), clip_plane_center.y(), clip_plane_center.z(), del_draw.x(), del_draw.y(), del_draw.z(), 255, 255, 100, 255);
        auto viewerDistDraw = cam_coords + camera_to_target;
        GRAPHICS::DRAW_LINE(cam_coords.x(), cam_coords.y(), cam_coords.z(), viewerDistDraw.x(), viewerDistDraw.y(), viewerDistDraw.z(), 255, 100, 100, 255);
    }
    Eigen::Vector3f camera_to_target_unit_vector = camera_to_target * (1. / camera_to_target.norm()); // Unit vector in direction of plane / line intersection

    double view_plane_dist = cam_near_clip / cam_dir.dot(camera_to_target_unit_vector);

    Eigen::Vector3f up3d, forward3d, right3d;
    up3d = rotate(WORLD_UP, cam_rotation);
    right3d = rotate(WORLD_EAST, cam_rotation);
    forward3d = rotate(WORLD_NORTH, cam_rotation);
    Eigen::Vector3f new_origin = clip_plane_center + (near_clip_height / 2.) * cam_up - (near_clip_width / 2.) * cam_east;

    if (draw_debug)
    {
        auto top_right = new_origin + near_clip_width * right3d;

        GRAPHICS::DRAW_LINE(new_origin.x(), new_origin.y(), new_origin.z(), top_right.x(), top_right.y(), top_right.z(), 100, 255, 100, 255);



        auto bottom_right = top_right - near_clip_height * up3d;

        GRAPHICS::DRAW_LINE(bottom_right.x(), bottom_right.y(), bottom_right.z(), top_right.x(), top_right.y(), top_right.z(), 100, 255, 100, 255);



        auto bottom_left = bottom_right - near_clip_width * right3d;

        GRAPHICS::DRAW_LINE(bottom_right.x(), bottom_right.y(), bottom_right.z(), bottom_left.x(), bottom_left.y(), bottom_left.z(), 100, 255, 100, 255);
        GRAPHICS::DRAW_LINE(bottom_left.x(), bottom_left.y(), bottom_left.z(), new_origin.x(), new_origin.y(), new_origin.z(), 100, 255, 100, 255);
    }
    Eigen::Vector2f ret;

    bool use_artur_method = true;

    if (use_artur_method)
    {
        Eigen::Vector3f view_plane_point = view_plane_dist * camera_to_target_unit_vector + camera_center;
        view_plane_point = (view_plane_point + clip_plane_center) - new_origin;
        double viewPlaneX = view_plane_point.dot(cam_east) / cam_east.dot(cam_east);
        double viewPlaneZ = view_plane_point.dot(cam_up) / cam_up.dot(cam_up);
        double screenX = viewPlaneX / near_clip_width;
        double screenY = -viewPlaneZ / near_clip_height;
        ret = { screenX, screenY };
    }
    else
    {
        auto intersection = cam_coords + view_plane_dist * camera_to_target_unit_vector;
        auto center_to_intersection = clip_plane_center - intersection;
        auto x_dist = center_to_intersection.dot(right3d);
        auto z_dist = center_to_intersection.dot(up3d);
        auto screen_x = 1. - (near_clip_width / 2. + x_dist) / near_clip_width;
        auto screen_y = (near_clip_height / 2. + z_dist) / near_clip_height;
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