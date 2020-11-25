#include "rpgq_simulator/visualization/flightmare_bridge.hpp"

namespace RPGQ {
namespace Simulator {

// constructor
FlightmareBridge::FlightmareBridge() {
  // initialize ZMQ connection
  // initializeConnections();
}

bool FlightmareBridge::connectUnity() {
  if (!unity_ready_) {
    // initialize Scene settings
    sendInitialSettings();
    // check if setting is done
    unity_ready_ = handleSettings();
  }
  return unity_ready_;
}

void FlightmareBridge::disconnectUnity() {
  unity_ready_ = false;
  // create new message object
  std::string client_address_dis_{"tcp://*"};
  std::string pub_port_dis_{"10255"};
  zmqpp::context context_dis_;
  zmqpp::socket pub_dis_{context_dis_, zmqpp::socket_type::publish};
  pub_dis_.set(zmqpp::socket_option::send_high_water_mark, 6);
  pub_dis_.bind(client_address_dis_ + ":" + pub_port_dis_);

  // wait until publisher is properly connected
  usleep(1000000);
  zmqpp::message msg_dis_;
  ROS_INFO("Disconnect from Unity!");
  msg_dis_ << "DISCONNECT";
  pub_dis_.send(msg_dis_, true);

  RPGQ::FlightmareTypes::ImgID send_id = 1;
  getRender(send_id);

  pub_.close();
  sub_.close();
  pub_dis_.close();
}

std::vector<double> FlightmareBridge::positionROS2Unity(
    const Eigen::Vector3d &ros_pos) {
  // tranform position from ROS coordinate system (right-handed)
  // to Unity coordinate system (left-handed)
  std::vector<double> unity_pos{ros_pos[0], ros_pos[2], ros_pos[1]};
  return unity_pos;
}

std::vector<double> FlightmareBridge::rotationROS2Unity(
    const Eigen::Quaterniond &ros_quat) {
  // tranform rotation (quaternion) from ROS coordinate system (right hand)
  // to Unity coordinate system (left hand)
  static Eigen::Matrix3d T;
  T(0, 0) = 1.0;
  T(0, 1) = 0.0;
  T(0, 2) = 0.0;
  T(1, 0) = 0.0;
  T(1, 1) = 0.0;
  T(1, 2) = 1.0;
  T(2, 0) = 0.0;
  T(2, 1) = 1.0;
  T(2, 2) = 0.0;
  Eigen::Matrix3d R_unity = T * ros_quat.toRotationMatrix() * T.transpose();
  Eigen::Quaterniond q_unity(R_unity);
  std::vector<double> unity_quat{q_unity.x(), q_unity.y(), q_unity.z(),
                                 q_unity.w()};
  return unity_quat;
}

std::vector<double> FlightmareBridge::matrix44ROS2Unity(
    const Eigen::Matrix4d &ros_trans) {
  static Eigen::Matrix4d T;
  T(0, 0) = 1.0;
  T(0, 1) = 0.0;
  T(0, 2) = 0.0;
  T(0, 3) = 0.0;
  T(1, 0) = 0.0;
  T(1, 1) = 0.0;
  T(1, 2) = 1.0;
  T(1, 3) = 0.0;
  T(2, 0) = 0.0;
  T(2, 1) = 1.0;
  T(2, 2) = 0.0;
  T(2, 3) = 0.0;
  T(3, 0) = 0.0;
  T(3, 1) = 0.0;
  T(3, 2) = 0.0;
  T(3, 3) = 1.0;
  Eigen::Matrix4d TRAN_unity = T * ros_trans * T.transpose();
  std::vector<double> tran_unity;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      tran_unity.push_back(TRAN_unity(i, j));
    }
  }
  return tran_unity;
}

std::vector<double> FlightmareBridge::sizeROS2Unity(
    const Eigen::Vector3d &ros_size) {
  // 3D size from right hand coordinate system to left hand coordinate system.
  std::vector<double> unity_size{ros_size[0], ros_size[2], ros_size[1]};
  return unity_size;
}

void FlightmareBridge::addQuad(std::shared_ptr<QuadrotorVehicle> quad) {
  // add quadrotor that does not have attached camera
  Vehicle_t vehicle_t;
  // retreive vehicle information from simulation
  vehicle_t.ID = quad->GetID();
  vehicle_t.position = positionROS2Unity(quad->GetPos());
  vehicle_t.rotation = rotationROS2Unity(quad->GetQuat());
  vehicle_t.size = sizeROS2Unity(quad->GetSize());
  //
  settings_.vehicles.push_back(vehicle_t);
  pub_msg_.vehicles.push_back(vehicle_t);
  //
  unity_vehicles_.emplace(vehicle_t.ID, quad);
}

void FlightmareBridge::addQuadLidar(std::shared_ptr<QuadLidar> quad_lidar) {
  std::cout << "Adding a Quadrotor with LiDAR" << std::endl;
  // add quadrotor that does not have attached camera
  Vehicle_t vehicle_t;
  std::shared_ptr<QuadrotorVehicle> quad = quad_lidar->GetQuad();
  vehicle_t.ID = quad->GetID();
  vehicle_t.position = positionROS2Unity(quad->GetPos());
  vehicle_t.rotation = rotationROS2Unity(quad->GetQuat());
  vehicle_t.size = sizeROS2Unity(quad->GetSize());
  //
  Lidar_t lidar_t;
  std::shared_ptr<Lidar> lidar = quad_lidar->GetLidar();
  lidar_t.ID = lidar->GetID();
  lidar_t.num_beams = lidar->GetBeams();
  lidar_t.start_scan_angle = lidar->GetStartScanAngle();
  lidar_t.max_distance = lidar->GetMaxDistance();
  lidar_t.end_scan_angle = lidar->GetEndScanAngle();
  lidar_t.T_BS = matrix44ROS2Unity(lidar->GetRelPose());
  vehicle_t.lidars.push_back(lidar_t);
  //
  settings_.vehicles.push_back(vehicle_t);
  pub_msg_.vehicles.push_back(vehicle_t);
  //
  unity_vehicles_.emplace(vehicle_t.ID, quad);
  unity_lidars_.emplace(vehicle_t.ID + "_" + lidar_t.ID, lidar);
}

//
void FlightmareBridge::addQuadRGB(std::shared_ptr<QuadRGBCamera> quad_rgb) {
  std::cout << "Adding a Quadrotor with RGB Camera" << std::endl;
  // add quadrotor that has an RGB camera attached.
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
  camera_t.post_processing = cam->GetPostProcessing();
  camera_t.is_depth = false;
  camera_t.output_index = 0;
  vehicle_t.cameras.push_back(camera_t);
  //
  settings_.vehicles.push_back(vehicle_t);
  pub_msg_.vehicles.push_back(vehicle_t);
  //
  unity_vehicles_.emplace(vehicle_t.ID, quad);
  unity_cameras_.emplace(vehicle_t.ID + "_" + camera_t.ID, cam);
}

//
void FlightmareBridge::addQuadStereoRGB(
    std::shared_ptr<QuadStereoRGBCamera> stereo_quad_rgb) {
  // add quadrotor that has an RGB camera attached.
  Vehicle_t vehicle_t;
  std::shared_ptr<QuadrotorVehicle> quad = stereo_quad_rgb->GetQuad();
  vehicle_t.ID = quad->GetID();
  vehicle_t.position = positionROS2Unity(quad->GetPos());
  vehicle_t.rotation = rotationROS2Unity(quad->GetQuat());
  vehicle_t.size = sizeROS2Unity(quad->GetSize());

  std::shared_ptr<RGBCamera> left_cam = stereo_quad_rgb->GetLeftRGBCamera();
  Camera_t left_camera_t;
  left_camera_t.ID = left_cam->GetID();
  left_camera_t.T_BC = matrix44ROS2Unity(left_cam->GetRelPose());
  left_camera_t.channels = left_cam->GetChannel();
  left_camera_t.width = left_cam->GetWidth();
  left_camera_t.height = left_cam->GetHeight();
  left_camera_t.fov = left_cam->GetFov();
  left_camera_t.depth_scale = left_cam->GetDepthScale();
  left_camera_t.post_processing = left_cam->GetPostProcessing();
  left_camera_t.is_depth = false;
  left_camera_t.output_index = 0;
  vehicle_t.cameras.push_back(left_camera_t);

  std::shared_ptr<RGBCamera> right_cam = stereo_quad_rgb->GetRightRGBCamera();
  Camera_t right_camera_t;
  right_camera_t.ID = right_cam->GetID();
  right_camera_t.T_BC = matrix44ROS2Unity(right_cam->GetRelPose());
  right_camera_t.channels = right_cam->GetChannel();
  right_camera_t.width = right_cam->GetWidth();
  right_camera_t.height = right_cam->GetHeight();
  right_camera_t.fov = right_cam->GetFov();
  right_camera_t.depth_scale = right_cam->GetDepthScale();
  right_camera_t.post_processing = right_cam->GetPostProcessing();
  right_camera_t.is_depth = false;
  right_camera_t.output_index = 1;
  vehicle_t.cameras.push_back(right_camera_t);
  //
  settings_.vehicles.push_back(vehicle_t);
  pub_msg_.vehicles.push_back(vehicle_t);
  //
  unity_vehicles_.emplace(vehicle_t.ID, quad);
  unity_cameras_.emplace(vehicle_t.ID + "_" + left_camera_t.ID, left_cam);
  unity_cameras_.emplace(vehicle_t.ID + "_" + right_camera_t.ID, right_cam);
}

//
void FlightmareBridge::addObject(std::shared_ptr<UnityObject> obj) {
  Object_t object_t;
  object_t.ID = obj->GetID();
  object_t.prefab_ID = obj->GetPrefabID();
  object_t.position = positionROS2Unity(obj->GetPos());
  object_t.rotation = rotationROS2Unity(obj->GetQuat());
  object_t.size = sizeROS2Unity(obj->GetSize());
  //
  settings_.objects.push_back(object_t);
  pub_msg_.objects.push_back(object_t);
  //
  unity_objects_.emplace(object_t.ID, obj);
}

// public get functions
void FlightmareBridge::getRender(const FlightmareTypes::ImgID &ntime) {
  pub_msg_.ntime = ntime;
  for (size_t idx = 0; idx < pub_msg_.vehicles.size(); idx++) {
    Vehicle_t vehicle_i = pub_msg_.vehicles[idx];
    std::shared_ptr<QuadrotorVehicle> quad = unity_vehicles_[vehicle_i.ID];
    pub_msg_.vehicles[idx].position = positionROS2Unity(quad->GetPos());
    pub_msg_.vehicles[idx].rotation = rotationROS2Unity(quad->GetQuat());
  }

  for (size_t idx = 0; idx < pub_msg_.objects.size(); idx++) {
    Object_t object_i = pub_msg_.objects[idx];
    std::shared_ptr<UnityObject> object = unity_objects_[object_i.ID];
    pub_msg_.objects[idx].position = positionROS2Unity(object->GetPos());
    pub_msg_.objects[idx].rotation = rotationROS2Unity(object->GetQuat());
  }
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
void FlightmareBridge::initializeConnections() {
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
void FlightmareBridge::sendInitialSettings() {
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

bool FlightmareBridge::handleSettings() {
  // create new message object
  zmqpp::message msg;

  bool done = false;
  // Unpack message metadata
  if (sub_.receive(msg, true)) {
    std::string metadata_string = msg.get(0);
    // std::cout << metadata_string << std::endl;
    // Parse metadata
    if (json::parse(metadata_string).size() > 1) {
      return false;  // hack
    }
    done = json::parse(metadata_string).at("ready").get<bool>();
  }
  std::cout << "\rReady ? : " << done << std::flush << std::endl;
  return done;
}

FlightmareTypes::ImgID FlightmareBridge::handleOutput(RenderMessage_t &output) {
  // create new message object
  zmqpp::message msg;
  sub_.receive(msg);
  // unpack message metadata
  std::string json_sub_msg = msg.get(0);
  // parse metadata
  SubMessage_t sub_msg = json::parse(json_sub_msg);

  ros::Time timestamp;
  timestamp = timestamp.fromNSec(sub_msg.ntime);

  // ensureBufferIsAllocated(sub_msg);
  size_t image_i = 1;
  size_t camera_i = 0;
  for (int i = 0; i < settings_.vehicles.size(); i++) {
    // update vehicle collision flag
    Vehicle_t vehicle = settings_.vehicles[i];
    std::shared_ptr<QuadrotorVehicle> quad = unity_vehicles_[vehicle.ID];
    quad->SetCollision(sub_msg.sub_vehicles[i].collision);

    // feed lidar ranges
    if (unity_lidars_.size() > 0) {
      // TODO: For now we only consider the first lidar attached on the vehicle
      std::shared_ptr<Lidar> lidar =
          unity_lidars_[vehicle.ID + "_" + vehicle.lidars[0].ID];
      std::vector<double> lidar_ranges;
      for (auto lidar_range_i : sub_msg.sub_vehicles[i].lidar_ranges) {
        lidar_ranges.push_back(lidar_range_i);
      }
      lidar->FeedLidarQueue(timestamp, lidar_ranges);
    }

    // feed image data to RGB camera
    for (const auto &cam : vehicle.cameras) {
      std::unordered_map<std::string, cv::Mat> images;
      std::string camera_ID = vehicle.ID + "_" + cam.ID;
      std::shared_ptr<RGBCamera> rgb_camera = unity_cameras_[camera_ID];

      uint32_t imageLen = cam.width * cam.height * cam.channels;
      // Get raw image bytes from ZMQ message.
      // WARNING: This is a zero-copy operation that also casts the input to an
      // array of unit8_t. when the message is deleted, this pointer is also
      // dereferenced.
      const uint8_t *imageData;
      msg.get(imageData, image_i);
      image_i = image_i + 1;
      //
      uint32_t bufferRowLength = cam.width * cam.channels;
      // Pack image into cv::Mat
      cv::Mat new_image =
          cv::Mat(cam.height, cam.width, CV_MAKETYPE(CV_8U, cam.channels));
      memcpy(new_image.data, imageData, imageLen);
      // Flip image since OpenCV origin is upper left, but Unity's is lower
      // left.
      cv::flip(new_image, new_image, 0);

      // Tell OpenCv that the input is RGB.
      if (cam.channels == 3) {
        cv::cvtColor(new_image, new_image, CV_RGB2BGR);
      }
      images.emplace("rgb", new_image);
      size_t post_press_idx = 0;
      for (const auto &pp : cam.post_processing) {
        if (post_press_idx == 0) {
          // depth
          uint32_t image_len = cam.width * cam.height * 4;
          // Get raw image bytes from ZMQ message.
          // WARNING: This is a zero-copy operation that also casts the input to
          // an array of unit8_t. when the message is deleted, this pointer is
          // also dereferenced.
          const uint8_t *image_data;
          msg.get(image_data, image_i);
          image_i = image_i + 1;
          // Pack image into cv::Mat
          cv::Mat pp_image = cv::Mat(cam.height, cam.width, CV_32FC1);
          memcpy(pp_image.data, image_data, image_len);
          // Flip image since OpenCV origin is upper left, but Unity's is lower
          // left.
          pp_image = pp_image * (1000.f);
          cv::flip(pp_image, pp_image, 0);

          // unity_quadrotors_[idx]
          //     ->getCameras()[cam.output_index]
          //     ->feedImageQueue(layer_idx, new_image);
          images.emplace(pp, pp_image);

        } else {
          //
          const uint8_t *pp_imageData;
          msg.get(pp_imageData, image_i);
          image_i = image_i + 1;
          //
          uint32_t pp_bufferRowLength = cam.width * cam.channels;
          // Pack image into cv::Mat
          cv::Mat pp_image =
              cv::Mat(cam.height, cam.width, CV_MAKETYPE(CV_8U, cam.channels));
          memcpy(pp_image.data, pp_imageData, imageLen);
          // Flip image since OpenCV origin is upper left, but Unity's is lower
          // left.
          cv::flip(pp_image, pp_image, 0);

          // Tell OpenCv that the input is RGB.
          if (cam.channels) {
            cv::cvtColor(pp_image, pp_image, CV_RGB2BGR);
          }
          images.emplace(pp, pp_image);
        }

        post_press_idx += 1;
      }
      //
      rgb_camera->FeedImageQueue(timestamp, images);
    }
  }
  //
  return sub_msg.ntime;
}

}  // namespace Simulator
}  // namespace RPGQ
