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

namespace RPGQ
{
  namespace Simulator
  {
     namespace FlightmareTypes
     {
       typedef uint64_t USecs;
     }

    // Unity Camera, should not be used alone.
    // has to be attached on a vehicle.
    struct Camera_t
    {
      std::string ID;
      // frame Metadata
      // int64_t ntime; // deprecated
      int channels{3};
      int width{1024};
      int height{768};
      double fov{70.0f};
      double depth_scale{0.20}; // 0.xx corresponds to xx cm resolution
      // metadata
      bool is_depth{false};
      int output_index{0};
      //
      std::vector<std::string> post_processing;
      // Transformation matrix from camera to vehicle body 4 x 4
      // use 1-D vector for json convention
      std::vector<double> T_BC { 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0};
    };

    // Unity Vechicle, e.g., quadrotor
    struct Vehicle_t
    {
      std::string ID;
      // unity coordinate system left-handed, y up
      std::vector<double> position{0.0, 0.0, 0.0};
      // unity quaternion (x, y, z, w)
      std::vector<double> rotation{0.0, 0.0, 0.0, 1.0};
      std::vector<double> size{1.0, 1.0, 1.0}; // scale
      // sensors attached on the vehicle
      std::vector<Camera_t> cameras;
      // collision check
      bool has_collision_check = true;
    };

    // Unity object, e.g., gate, light, etc...
    struct Object_t
    {
      std::string ID;
      std::string prefab_ID;
      // unity coordinate system left hand, y up
      std::vector<double> position{0.0, 0.0, 0.0};
      // unity quaternion (x, y, z, w)
      std::vector<double> rotation{0.0, 0.0, 0.0, 1.0};
      std::vector<double> size{1.0, 1.0, 1.0}; // scale
    };

    struct SettingsMessage_t
    {
      // scene/render settings
      bool scene_is_internal{true};
      std::string scene_filename;

      // ONLY Support one vehicle for now....
      std::vector<Vehicle_t> vehicles;
      std::vector<Object_t> objects;
    };

    struct PubMessage_t
    {
      FlightmareTypes::USecs ntime{0};
      std::vector<Vehicle_t> vehicles;
      std::vector<Object_t> objects;
    };

    struct SubMessage_t
    {
      //
      FlightmareTypes::USecs ntime{0};
      int camWidth;
      int camHeight;
      double camDepthScale;
      // Object state update
      std::vector<std::string> cameraIDs;
      std::vector<int> channels;

      // collision detector and raycaster update
      bool has_vehicle_collision;
      float lidar_return;
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
               {"outputIndex", o.output_index}
      };
    }

    // Vehicle_t
    inline void to_json(json &j, const Vehicle_t &o) {
      j = json{{"ID", o.ID},
               {"position", o.position},
               {"rotation", o.rotation},
               {"size", o.size},
               {"cameras", o.cameras},
               {"hasCollisionCheck", o.has_collision_check}
      };
    }

    // Object_t
    inline void to_json(json &j, const Object_t &o) {
      j = json{{"ID", o.ID},
               {"prefabID", o.prefab_ID},
               {"position", o.position},
               {"rotation", o.rotation},
               {"size", o.size}
      };
    }

    // Setting messages, pub to unity
    inline void to_json(json &j, const SettingsMessage_t &o) {
      j = json{{"sceneIsInternal", o.scene_is_internal},
               {"sceneFilename", o.scene_filename},
               {"vehicles", o.vehicles},
               {"objects", o.objects}
      };
    }

    // Publish messages to unity
    inline void to_json(json &j, const PubMessage_t &o) {
      j = json{{"ntime", o.ntime},
               {"vehicles", o.vehicles},
               {"objects", o.objects}
      };
    }

    // Subscribe message from unity
    inline void from_json(const json &j, SubMessage_t &o) {
      o.ntime = j.at("ntime").get<FlightmareTypes::USecs>();
      o.camWidth = j.at("camWidth").get<int>();
      o.camHeight = j.at("camHeight").get<int>();
      o.cameraIDs = j.at("cameraIDs").get<std::vector<std::string>>();
      o.channels = j.at("channels").get<std::vector<int>>();
      o.has_vehicle_collision = j.at("hasVehicleCollision").get<bool>();
      o.lidar_return = j.at("lidarReturn").get<float>();
    }

    // Struct for outputting parsed received messages to handler functions
    struct RenderMessage_t
    {
      SubMessage_t sub_msg;
      std::vector<cv::Mat> images;
    };

  } // namespace Simulator
} // namespace RPGQ