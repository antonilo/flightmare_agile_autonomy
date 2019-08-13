#pragma once

#include <eigen3/Eigen/Eigen>
#include <fstream>
#include <chrono>
#include <unordered_map>

// Include ZMQ bindings for comms with Unity.
#include <iostream>
#include <string>
#include <zmqpp/zmqpp.hpp>

#include "rpgq_simulator/visualization/flightmare_message_types.hpp"
#include "rpgq_simulator/framework/unity/unity_object.h"
#include "rpgq_simulator/implementation/objects/quadrotor_vehicle/quad_and_rgb_camera.h"
#include "rpgq_simulator/implementation/objects/quadrotor_vehicle/quadrotor_vehicle.h"

using json = nlohmann::json;

namespace RPGQ
{
  namespace Simulator
  {
    class FlightmareBridge
    {
     public:
      // constructor & destructor
      FlightmareBridge();
      ~FlightmareBridge() {};

      // public set functions
      void setScene(const std::string scene_name) {
        settings_.scene_filename = scene_name;
      };

      // public add functions
      void addObject(std::shared_ptr<UnityObject> object);
      void addQuadRGB(std::shared_ptr<QuadRGBCamera> quad_rbg);

      void updateVehiclePoses(const FlightmareTypes::USecs ntime,
        std::shared_ptr<QuadRGBCamera> quad_rgb,
        size_t vehicle_idx);
      void updateObjectPoses(const FlightmareTypes::USecs ntime,
        std::shared_ptr<UnityObject> object,
        size_t object_idx);

      // public connect function
      bool connectUnity();
      // public get functions
      void getRender(void);
      void handleImages(std::shared_ptr<QuadRGBCamera> quad_rbg);

      static std::shared_ptr<FlightmareBridge> getInstance(void) {
        static std::shared_ptr<FlightmareBridge> fmb = std::make_shared<FlightmareBridge>();
        return fmb;
      };

      // public auxiliary functions
      std::vector<double> matrix44ROS2Unity(const Eigen::Matrix4d & ros_tranformation);
      std::vector<double> positionROS2Unity(const Eigen::Vector3d & ros_position);
      std::vector<double> rotationROS2Unity(const Eigen::Quaterniond & ros_quatenion);
      std::vector<double> sizeROS2Unity(const Eigen::Vector3d &  ros_size);

     private:
      // TODO
      SettingsMessage_t settings_;
      PubMessage_t pub_msg_;
      SubMessage_t sub_msg_;

      // ZMQ variables and functions
      std::string client_address_{"tcp://*"};
      std::string pub_port_{"10253"};
      std::string sub_port_{"10254"};
      zmqpp::context context_;
      zmqpp::socket pub_{context_, zmqpp::socket_type::publish};
      zmqpp::socket sub_{context_, zmqpp::socket_type::subscribe};
      void initializeConnections(void);
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
      inline void ensureBufferIsAllocated(SubMessage_t renderMetadata){
        // Check that buffer size is correct
        uint64_t requested_buffer_size = renderMetadata.camWidth * renderMetadata.camHeight * 3;
        // Resize if necessary
        if (input_buffer_.size() != requested_buffer_size)
        {
          input_buffer_.resize(requested_buffer_size);
        }
      };
    };
  } // namespace Simulator
} // namespace RPGQ