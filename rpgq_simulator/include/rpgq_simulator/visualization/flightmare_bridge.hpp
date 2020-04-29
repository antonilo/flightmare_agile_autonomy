#pragma once

#include <eigen3/Eigen/Eigen>
#include <fstream>
#include <chrono>
#include <unordered_map>
#include <map>

// Include ZMQ bindings for comms with Unity.
#include <iostream>
#include <string>
#include <zmqpp/zmqpp.hpp>

#include "rpgq_simulator/visualization/flightmare_message_types.hpp"
#include "rpgq_simulator/framework/unity/unity_object.h"
#include "rpgq_simulator/framework/objects/base_object.h"
#include "rpgq_simulator/implementation/unity/unity_gate.h"
#include "rpgq_simulator/implementation/objects/quadrotor_vehicle/quadrotor_vehicle.h"
#include "rpgq_simulator/implementation/objects/quadrotor_vehicle/quad_and_lidar.h"
#include "rpgq_simulator/implementation/objects/quadrotor_vehicle/quad_and_rgb_camera.h"
#include "rpgq_simulator/implementation/objects/quadrotor_vehicle/quad_and_stereo_rgb_camera.h"

using json = nlohmann::json;

namespace RPGQ {
namespace Simulator {
  
class FlightmareBridge {
  public:
    // constructor & destructor
    FlightmareBridge();
    ~FlightmareBridge() {};

    // public set functions
    void setScene(const size_t & scene_id) { 
      settings_.scene_id = scene_id;
    };

    // public add functions
    void addObject(std::shared_ptr<UnityObject> object);
    void addQuad(std::shared_ptr<QuadrotorVehicle> quad);
    void addQuadLidar(std::shared_ptr<QuadLidar> quad_lidar);
    void addQuadRGB(std::shared_ptr<QuadRGBCamera> quad_rbg);
    void addQuadStereoRGB(std::shared_ptr<QuadStereoRGBCamera> quad_stereo_rbg);

    // public connect function
    bool connectUnity();
    void disconnectUnity();
    
    // public get functions
    void getRender(const FlightmareTypes::ImgID & ntime);
    FlightmareTypes::ImgID handleOutput(RenderMessage_t & output);

    static std::shared_ptr<FlightmareBridge> getInstance(void) {
      static std::shared_ptr<FlightmareBridge> fmb = std::make_shared<FlightmareBridge>();
      return fmb;
    };

    // public auxiliary functions
    std::vector<double> matrix44ROS2Unity(const Eigen::Matrix4d & ros_tranformation);
    std::vector<double> positionROS2Unity(const Eigen::Vector3d & ros_position);
    std::vector<double> rotationROS2Unity(const Eigen::Quaterniond & ros_quatenion);
    std::vector<double> sizeROS2Unity(const Eigen::Vector3d &  ros_size);

    //
    void setPubPort(const std::string & pub_port){ pub_port_ = pub_port; }
    void setSubPort(const std::string & sub_port){ sub_port_ = sub_port; }

    void initializeConnections(void);

  private:
    // TODO
    SettingsMessage_t settings_;
    PubMessage_t pub_msg_;
    //
    std::unordered_map<std::string, std::shared_ptr<UnityObject>> unity_objects_;
    std::unordered_map<std::string, std::shared_ptr<QuadrotorVehicle>> unity_vehicles_;
    std::unordered_map<std::string, std::shared_ptr<RGBCamera>> unity_cameras_;
    std::unordered_map<std::string, std::shared_ptr<Lidar>> unity_lidars_;

    // ZMQ variables and functions
    std::string client_address_{"tcp://*"};
    std::string pub_port_{"10253"};
    std::string sub_port_{"10254"};
    zmqpp::context context_;
    zmqpp::socket pub_{context_, zmqpp::socket_type::publish};
    zmqpp::socket sub_{context_, zmqpp::socket_type::subscribe};
    void sendInitialSettings(void);
    bool handleSettings(void);

    // timing variables
    int64_t num_frames_ = 0;
    int64_t last_downloaded_utime_ = 0;
    int64_t last_download_debug_utime_ = 0;
    int64_t u_packet_latency_ = 0;

    // connecting symbols
    bool unity_ready_{false};

    // auxiliary variables
    std::vector<uint8_t> input_buffer_;

    // Ensure that input buffer can handle the incoming message.
  //      inline void ensureBufferIsAllocated(SubMessage_t renderMetadata){
  //        // Check that buffer size is correct
  //        uint64_t requested_buffer_size = renderMetadata.camWidth * renderMetadata.camHeight * 3;
  //        // Resize if necessary
  //        if (input_buffer_.size() != requested_buffer_size)
  //        {
  //          input_buffer_.resize(requested_buffer_size);
  //        }
  //      };
};

} // namespace Simulator
} // namespace RPGQ