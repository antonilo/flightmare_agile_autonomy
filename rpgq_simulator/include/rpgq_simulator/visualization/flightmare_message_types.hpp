#pragma once

/**
 * @file   jsonMessageSpec.hpp
 * @author Winter Guerra
 * @brief  Defines the json structure of messages going to and from Unity.
 */

// Message/state struct definition

#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Dense>

#include "rpgq_simulator/tools/json.hpp"

using json = nlohmann::json;

namespace RPGQ {
namespace FlightmareTypes {
typedef uint64_t USecs;
typedef uint64_t ImgID;

typedef size_t SceneID;
const SceneID SCENE_WAREHOUSE = 0;
const SceneID SCENE_NATUREFOREST = 3;
const SceneID SCENE_EMPTYFOREST = 4;
const SceneID SCENE_GRANDCANYON = 5;
const SceneID SCENE_WASTELAND = 6;
const SceneID SCENE_JAPANESE = 7;
}  // namespace FlightmareTypes

namespace Simulator {
// Unity Camera, should not be used alone.
// has to be attached on a vehicle.
struct Camera_t {
  std::string ID;
  // frame Metadata
  // int64_t ntime; // deprecated
  int channels{3};
  int width{1024};
  int height{768};
  double fov{70.0f};
  double depth_scale{0.20};  // 0.xx corresponds to xx cm resolution
  // metadata
  bool is_depth{false};
  int output_index{0};
  //
  std::vector<std::string> post_processing;
  // Transformation matrix from camera to vehicle body 4 x 4
  // use 1-D vector for json convention
  std::vector<double> T_BC{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

struct Lidar_t {
  std::string ID;
  int num_beams{10};
  double max_distance{10.0};
  double start_scan_angle{-M_PI / 2};
  double end_scan_angle{M_PI / 2};
  // Transformation matrix from lidar to vehicle body 4 x 4
  // use 1-D vector for json convention
  std::vector<double> T_BS{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

// Unity Vechicle, e.g., quadrotor
struct Vehicle_t {
  std::string ID;
  // unity coordinate system left-handed, y up
  std::vector<double> position{0.0, 0.0, 0.0};
  // unity quaternion (x, y, z, w)
  std::vector<double> rotation{0.0, 0.0, 0.0, 1.0};
  std::vector<double> size{1.0, 1.0, 1.0};  // scale
  // sensors attached on the vehicle
  std::vector<Camera_t> cameras;
  std::vector<Lidar_t> lidars;
  // collision check
  bool has_collision_check = true;
};

// Unity object, e.g., gate, light, etc...
struct Object_t {
  std::string ID;
  std::string prefab_ID;
  // unity coordinate system left hand, y up
  std::vector<double> position{0.0, 0.0, 0.0};
  // unity quaternion (x, y, z, w)
  std::vector<double> rotation{0.0, 0.0, 0.0, 1.0};
  std::vector<double> size{1.0, 1.0, 1.0};  // scale
};

struct SettingsMessage_t {
  // scene/render settings
  size_t scene_id = FlightmareTypes::SCENE_WAREHOUSE;

  //
  std::vector<Vehicle_t> vehicles;
  std::vector<Object_t> objects;
  //
};

struct PubMessage_t {
  FlightmareTypes::USecs ntime{0};
  std::vector<Vehicle_t> vehicles;
  std::vector<Object_t> objects;
};

struct PointCloudMessage_t {
  int scene_id{0};
  std::vector<double> bounding_box_scale{100.0, 100.0, 100.0};
  std::vector<double> bounding_box_origin{0.0, 0.0, 0.0};
  double resolution_z{0.05};
  double ground_z_limit{0.2};
  double resolution_above_ground{0.15};
  double resolution_below_ground{0.3};
  std::string path{"point_clouds_data/"};
  std::string file_name{"default"};
  double unity_ground_offset{0.3};
};

struct TreeMessage_t {
  std::vector<double> bounding_area{25.0, 25.0};
  std::vector<double> bounding_origin{0.0, 0.0};
  double density{4.0};
  int seed{-1};
};

struct ObjectMessage_t {
  std::vector<std::string> name = {"Cube"};  // Cube, Sphere, Cylinder
  std::vector<double> bounding_area{25.0, 25.0, 25.0};
  std::vector<double> bounding_origin{0.0, 0.0, 0.0};
  double density{4.0};
  double rand_size{0.0};
  std::vector<double> scale_min{
      0.1,
      0.3,
      2.0,
  };
  std::vector<double> scale_max{1.0, 0.3, 10.0};

  std::vector<double> rot_min{0.0, 0.0, 0.0};
  std::vector<double> rot_max{360.0, 360.0, 360.0};
  int seed{-1};
};

struct FixRatioObjectMessage_t {
  std::string name = "Cube";  // Cube, Sphere, Cylinder
  std::vector<double> bounding_area{25.0, 25.0};
  std::vector<double> bounding_origin{0.0, 0.0};
  double density{4.0};
  double scale_min{0.1};
  double scale_max{2.0};
  double rot_min{0.0};  // rotation around z-axix
  double rot_max{360.0};
  int seed{-1};
};

struct LightMessage_t {
  double red{0};
  double green{0};
  double blue{0};
  double intensity{1.0};
};

//
struct Sub_Vehicle_t {
  bool collision;
  std::vector<double> lidar_ranges;
};

struct SubMessage_t {
  //
  FlightmareTypes::USecs ntime{0};
  //
  std::vector<Sub_Vehicle_t> sub_vehicles;
};

/*********************
 * JSON constructors *
 *********************/
// Camera_t
inline void to_json(json &j, const Camera_t &o) {
  j = json{{"ID", o.ID},
           {"channels", o.channels},
           {"width", o.width},
           {"height", o.height},
           {"fov", o.fov},
           {"T_BC", o.T_BC},
           {"isDepth", o.is_depth},
           {"post_processing", o.post_processing},
           {"depthScale", o.depth_scale},
           {"outputIndex", o.output_index}};
}

// Lidar
inline void to_json(json &j, const Lidar_t &o) {
  j = json{{"ID", o.ID},
           {"num_beams", o.num_beams},
           {"max_distance", o.max_distance},
           {"start_angle", o.start_scan_angle},
           {"end_angle", o.end_scan_angle},
           {"T_BS", o.T_BS}};
}
// Vehicle_t
inline void to_json(json &j, const Vehicle_t &o) {
  j = json{{"ID", o.ID},
           {"position", o.position},
           {"rotation", o.rotation},
           {"size", o.size},
           {"cameras", o.cameras},
           {"lidars", o.lidars},
           {"hasCollisionCheck", o.has_collision_check}};
}

// Object_t
inline void to_json(json &j, const Object_t &o) {
  j = json{{"ID", o.ID},
           {"prefabID", o.prefab_ID},
           {"position", o.position},
           {"rotation", o.rotation},
           {"size", o.size}};
}

// Setting messages, pub to unity
inline void to_json(json &j, const SettingsMessage_t &o) {
  j = json{{"scene_id", o.scene_id},
           {"vehicles", o.vehicles},
           {"objects", o.objects}};
}

// Publish messages to unity
inline void to_json(json &j, const PubMessage_t &o) {
  j = json{
      {"ntime", o.ntime}, {"vehicles", o.vehicles}, {"objects", o.objects}};
}

inline void to_json(json &j, const PointCloudMessage_t &o) {
  j = json{{"scene_id", o.scene_id},
           {"bounding_box_scale", o.bounding_box_scale},
           {"bounding_box_origin", o.bounding_box_origin},
           {"resolution_z", o.resolution_z},
           {"ground_z_limit", o.ground_z_limit},
           {"resolution_above_ground", o.resolution_above_ground},
           {"resolution_below_ground", o.resolution_below_ground},
           {"path", o.path},
           {"file_name", o.file_name},
           {"unity_ground_offset", o.unity_ground_offset}};
}

inline void to_json(json &j, const TreeMessage_t &o) {
  j = json{{"bounding_area", o.bounding_area},
           {"bounding_origin", o.bounding_origin},
           {"density", o.density},
           {"seed", o.seed}};
}

inline void to_json(json &j, const ObjectMessage_t &o) {
  j = json{{"bounding_area", o.bounding_area},
           {"bounding_origin", o.bounding_origin},
           {"density", o.density},
           {"rand_size", o.rand_size},
           {"seed", o.seed},
           {"name", o.name},
           {"scale_min", o.scale_min},
           {"scale_max", o.scale_max},
           {"rot_min", o.rot_min},
           {"rot_max", o.rot_max}};
}

inline void to_json(json &j, const FixRatioObjectMessage_t &o) {
  j = json{{"bounding_area", o.bounding_area},
           {"bounding_origin", o.bounding_origin},
           {"density", o.density},
           {"seed", o.seed},
           {"name", o.name},
           {"scale_min", o.scale_min},
           {"scale_max", o.scale_max},
           {"rot_min", o.rot_min},
           {"rot_max", o.rot_max}};
}

inline void to_json(json &j, const LightMessage_t &o) {
  j = json{{"red", o.red},
           {"green", o.green},
           {"blue", o.blue},
           {"intensity", o.intensity}};
}

// Publish messages to unity
inline void from_json(const json &j, Sub_Vehicle_t &o) {
  o.collision = j.at("collision").get<bool>();
  o.lidar_ranges = j.at("lidar_ranges").get<std::vector<double>>();
}

// json to our sub message data type
// note: pub_vechicles is defined in Unity which corresponding
// to our sub_vehicles in ROS.
inline void from_json(const json &j, SubMessage_t &o) {
  o.ntime = j.at("ntime").get<FlightmareTypes::USecs>();
  o.sub_vehicles = j.at("pub_vehicles").get<std::vector<Sub_Vehicle_t>>();
}

// Struct for outputting parsed received messages to handler functions
struct RenderMessage_t {
  SubMessage_t sub_msg;
  std::vector<cv::Mat> images;
};

}  // namespace Simulator
}  // namespace RPGQ