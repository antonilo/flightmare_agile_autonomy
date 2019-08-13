#include "rpgq_simulator/visualization/flightmare_bridge.hpp"

namespace RPGQ
{
  namespace Simulator
  {
    // constructor
    FlightmareBridge::FlightmareBridge()
    {
      // initialize ZMQ connection
      initializeConnections();
    }

    bool FlightmareBridge::connectUnity()
    {
      if (!unity_ready_)
      {
        // initialize Scene settings
        sendInitialSettings();
        // check if setting is done
        unity_ready_ = handleSettings();
      }
      return unity_ready_;
    }

    std::vector<double> FlightmareBridge::positionROS2Unity(
      const Eigen::Vector3d & ros_pos)
    {
      // tranform position from ROS coordinate system (right hand)
      // to Unity coordinate system (left hand)
      std::vector<double> unity_pos{ros_pos[0], ros_pos[2], ros_pos[1]};
      return unity_pos;
    }

    std::vector<double> FlightmareBridge::rotationROS2Unity(
      const Eigen::Quaterniond & ros_quat)
    {
      // tranform rotation (quaternion) from ROS coordinate system (right hand)
      // to Unity coordinate system (left hand)
      static Eigen::Matrix3d T;
      T(0, 0) = 1.0; T(0, 1) = 0.0; T(0, 2) = 0.0;
      T(1, 0) = 0.0; T(1, 1) = 0.0; T(1, 2) = 1.0;
      T(2, 0) = 0.0; T(2, 1) = 1.0; T(2, 2) = 0.0;
      Eigen::Matrix3d R_unity = T * ros_quat.toRotationMatrix() * T.transpose();
      Eigen::Quaterniond q_unity(R_unity);
      std::vector<double> unity_quat{q_unity.x(), q_unity.y(), q_unity.z(), q_unity.w()};
      return unity_quat;
    }

    std::vector<double> FlightmareBridge::matrix44ROS2Unity(
      const Eigen::Matrix4d & ros_trans)
    {
      static Eigen::Matrix4d T;
      T(0, 0) = 1.0; T(0, 1) = 0.0; T(0, 2) = 0.0; T(0, 3) = 0.0;
      T(1, 0) = 0.0; T(1, 1) = 0.0; T(1, 2) = 1.0; T(1, 3) = 0.0;
      T(2, 0) = 0.0; T(2, 1) = 1.0; T(2, 2) = 0.0; T(2, 3) = 0.0;
      T(3, 0) = 0.0; T(3, 1) = 0.0; T(3, 2) = 0.0; T(3, 3) = 1.0;
      Eigen::Matrix4d TRAN_unity = T * ros_trans * T.transpose();
      std::vector<double> tran_unity;
      for (int i=0; i<4; ++i){
        for (int j=0; j<4; ++j){
          tran_unity.push_back(TRAN_unity(i, j));
        }
      }
      return tran_unity;
    }

    std::vector<double> FlightmareBridge::sizeROS2Unity(
      const Eigen::Vector3d & ros_size)
    {
      // 3D size in right hand coordinate system to left hand coordinate system.
      std::vector<double> unity_size{ros_size[0], ros_size[2], ros_size[1]};
      return unity_size;
    }

    //
    void FlightmareBridge::addQuadRGB(std::shared_ptr<QuadRGBCamera> quad_rgb)
    {
      Vehicle_t vehicle_t;
      std::shared_ptr<QuadrotorVehicle> quad = quad_rgb->GetQuad();
      vehicle_t.ID = quad->GetID();
      vehicle_t.position = positionROS2Unity(quad->GetPos());
      vehicle_t.rotation = rotationROS2Unity(quad->GetQuat());
      vehicle_t.size = sizeROS2Unity(quad->GetSize());

      std::shared_ptr<RGBCamera> cam = quad_rgb->GetRGBCamera();
      Camera_t camera_t;
      camera_t.ID = cam->GetID();
      camera_t.T_BC = matrix44ROS2Unity(cam->GetRelPose());
      camera_t.channels = cam->GetChannel();
      camera_t.width = cam->GetWidth();
      camera_t.height = cam->GetHeight();
      camera_t.fov = cam->GetFov();
      camera_t.depth_scale = cam->GetDepthScale();
      camera_t.is_depth = false;
      camera_t.output_index = 0;
      vehicle_t.cameras.push_back(camera_t);
      //
      settings_.vehicles.push_back(vehicle_t);
      pub_msg_.vehicles.push_back(vehicle_t);
    }

    //
    void FlightmareBridge::addObject(std::shared_ptr<UnityObject> obj)
    {
      Object_t object_t;
      object_t.ID = obj->GetID();
      object_t.prefab_ID = obj->GetPrefabID();
      object_t.position = positionROS2Unity(obj->GetPos());
      object_t.rotation = rotationROS2Unity(obj->GetQuat());
      object_t.size = sizeROS2Unity(obj->GetSize());
      //
      settings_.objects.push_back(object_t);
      pub_msg_.objects.push_back(object_t);
    }

    void FlightmareBridge::updateVehiclePoses(const FlightmareTypes::USecs ntime,
      std::shared_ptr<QuadRGBCamera> quad_rgb,
      size_t vehicle_idx)
    {
      std::shared_ptr<QuadrotorVehicle> quad = quad_rgb->GetQuad();
      pub_msg_.ntime = ntime;
      pub_msg_.vehicles[vehicle_idx].position = positionROS2Unity(quad->GetPos());
      pub_msg_.vehicles[vehicle_idx].rotation = rotationROS2Unity(quad->GetQuat());
    }

    void FlightmareBridge::updateObjectPoses(
      const FlightmareTypes::USecs ntime,
      std::shared_ptr<UnityObject> object,
      size_t object_idx)
    {
      pub_msg_.ntime = ntime;
      pub_msg_.objects[object_idx].position = positionROS2Unity(object->GetPos());
      pub_msg_.objects[object_idx].rotation = rotationROS2Unity(object->GetQuat());
    }

    // public get functions
    void FlightmareBridge::getRender(void)
    {
      // create new message object
      zmqpp::message msg;
      // add topic header
      msg << "Pose";
      // create JSON object for pose update and append
      json json_msg = pub_msg_;
      msg << json_msg.dump();
      // send message without blocking
      pub_.send(msg, true);
    }

    // ZMQ functions
    void FlightmareBridge::initializeConnections()
    {
      std::printf("Initializing ZMQ connections...\n");

      // create and bind an upload socket
      pub_.set(zmqpp::socket_option::send_high_water_mark, 6);
      pub_.bind(client_address_ + ":" + pub_port_);

      // create and bind a download_socket
      sub_.set(zmqpp::socket_option::receive_high_water_mark, 6);
      sub_.bind(client_address_ + ":" + sub_port_);
      sub_.subscribe("");
      std::printf("Initializing ZMQ done...\n");
    }

    // Send settings to the Unity
    void FlightmareBridge::sendInitialSettings()
    {
      // create new message object
      zmqpp::message msg;
      // add topic header
      msg << "Pose";
      // create JSON object for initial settings
      json json_mesg = settings_;
      msg << json_mesg.dump();
      // send message without blocking
      pub_.send(msg, true);
    };

    bool FlightmareBridge::handleSettings()
    {
      // create new message object
      zmqpp::message msg;

      bool done = false;
      // Unpack message metadata
      if (sub_.receive(msg, true)) {
        std::string metadata_string = msg.get(0);
        // Parse metadata
        done = json::parse(metadata_string).at("ready").get<bool>();
      }
      return done;
    }

    void FlightmareBridge::handleImages(std::shared_ptr<QuadRGBCamera> quad_rgb)
    {
      RenderMessage_t  output;
      // create new message object
      zmqpp::message msg;
      sub_.receive(msg);
      // unpack message metadata
      std::string json_sub_msg = msg.get(0);
      // parse metadata
      SubMessage_t sub_msg = json::parse(json_sub_msg);

      ros::Time image_timestamp;
      image_timestamp = image_timestamp.fromNSec(sub_msg.ntime);

      ensureBufferIsAllocated(sub_msg);

      output.images.resize(sub_msg.cameraIDs.size());

      for (uint i=0; i < sub_msg.cameraIDs.size(); ++i)
      {
        // Reshape the received image
        // Calculate how long the casted and reshaped image will be.
        uint32_t imageLen = sub_msg.camWidth * sub_msg.camHeight * sub_msg.channels[i];
        // Get raw image bytes from ZMQ message.
        // WARNING: This is a zero-copy operation that also casts the input to an array of unit8_t.
        // when the message is deleted, this pointer is also dereferenced.
        const uint8_t* imageData;
        msg.get(imageData, i+1);
        // ALL images comes as 3-channel RGB images from Unity. Calculate the row length
        uint32_t bufferRowLength = sub_msg.camWidth * sub_msg.channels[i];

        // Pack image into cv::Mat
        cv::Mat new_image = cv::Mat(sub_msg.camHeight, sub_msg.camWidth, CV_MAKETYPE(CV_8U, sub_msg.channels[i]));
        memcpy(new_image.data, imageData, imageLen);
        // Flip image since OpenCV origin is upper left, but Unity's is lower left.
        cv::flip(new_image, new_image, 0);

        // Tell OpenCv that the input is RGB.
        if (sub_msg.channels[i]==3){
          cv::cvtColor(new_image, new_image, CV_RGB2BGR);
        }
        output.images.at(i) = new_image;
      }
      output.sub_msg = sub_msg;
      num_frames_++;

       std::shared_ptr<RGBCamera> cam = quad_rgb->GetRGBCamera();
       cam->feedImageQueue(image_timestamp, output.images.at(0));
    }
  } // namespace Simulator
} // namespace RPGQ
