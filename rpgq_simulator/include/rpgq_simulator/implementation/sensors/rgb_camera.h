#pragma once

#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

// rpgq rpgq_common
#include <rpgq_common/parameters/parameters.h>

// rpgq rpgq_simulator
#include <rpgq_simulator/framework/sensors/base_sensor.h>

// standard library
#include <memory>
#include <random>
#include <mutex>
#include <deque>
#include <unordered_map>

// others
#include <opencv2/core.hpp>
#include <eigen3/Eigen/Dense>

namespace RPGQ
{
  namespace Simulator
  {
    namespace RGBCameraTypes
    {
      typedef float Intensity_t;
      typedef cv::Mat_<Intensity_t> Image_t;
      typedef cv::Mat_<Intensity_t> Depthmap_t;
      typedef cv::Mat_<cv::Vec<Intensity_t, 2>> OpticFlow_t;

      typedef Eigen::Matrix4d Mat4_t;
      typedef Eigen::Vector3d Vec3_t;

      typedef std::function<Eigen::Vector3d()> GetPos_t;
      typedef std::function<Eigen::Vector3d()> GetVel_t;
      typedef std::function<Eigen::Vector3d()> GetAcc_t;
      typedef std::function<Eigen::Quaterniond()> GetQuat_t;
      typedef std::function<Eigen::Vector3d()> GetOmega_t;
      typedef std::function<Eigen::Vector3d()> GetPsi_t;

      const std::string RGB = "rgb";
      // image post processing
      typedef std::string PostProcessingID;
      const PostProcessingID Depth = "depth";
      const PostProcessingID OpticalFlow = "optical_flow";
      const PostProcessingID ObjectSegment = "object_segment"; // object segmentation
      const PostProcessingID CategorySegment = "category_segment"; // category segmentation
    }
    // standard camera
    class RGBCamera : public BaseSensor
    {
     public:
      // parameters
      enum
      {
        updateFrequency = 0, // [Hz]
        channels = 1, // [N]
        width = 2, // [pixels]
        height = 3, // [pixels]
        fov = 4,
        depth_scale = 5,
        NUM_PARAMS
      };

      RGBCamera(SensorID id,
       const Node* prevSimNode,
       Parameters<NUM_PARAMS> params = Parameters<NUM_PARAMS>("simulator/sensors/rgb_camera"));
      ~RGBCamera();

      void SetPoseCallbacks(RGBCameraTypes::GetPos_t GetPos,
                            RGBCameraTypes::GetVel_t GetVel,
                            RGBCameraTypes::GetQuat_t GetQuat,
                            RGBCameraTypes::GetOmega_t GetOmega)
      {
        GetPos_ = GetPos;
        GetVel_ = GetVel;
        GetQuat_ = GetQuat;
        GetOmega_ = GetOmega;
      };

      // Set relative pose
      void SetRelPose(Eigen::Vector3d &B_r_BC, Eigen::Matrix3d &R_BC){
        B_r_BC_ = B_r_BC;
        T_BC_.block<3,3>(0,0) = R_BC;
        T_BC_.block<3,1>(0,3) = B_r_BC_;
        T_BC_.row(3) << 0.0, 0.0, 0.0, 1.0;
      };

      void feedImageQueue(const ros::Time & img_timestamp,
        std::unordered_map<RGBCameraTypes::PostProcessingID, cv::Mat> & images);
      // public get functions
      RGBCameraTypes::Mat4_t GetRelPose(void) {return T_BC_;};

      int GetChannel(void) { return channels_; };
      int GetWidth(void) { return width_; };
      int GetHeight(void) { return height_; };
      double GetFov(void) {return fov_; };
      double GetDepthScale(void) { return depth_scale_; };
      std::vector<std::string> GetPostProcessing(void){
        std::vector<std::string> post_processing;
        for (const auto & pp : post_processing_){
          if (pp.second){
            post_processing.push_back(pp.first);
          }
        }
        return post_processing;
      };

      sensor_msgs::CameraInfo GetCameraInfo(void);

      void SetChanel(const int & channels) { channels_ = channels;};
      void SetWidth(const int & width) {width_ = width; };
      void SetHeight(const int & height) {height_ = height; };
      void SetFov(const double & fov) {fov_ = fov; };
      void SetDepthScale(const double & depth_scale)
          {depth_scale_ = depth_scale;};
      void SetPostProcessing(const std::vector<std::string> & post_processing){
        for (const auto & pp : post_processing){
          if (pp=="depth")
            post_processing_[RGBCameraTypes::Depth] = true;
          if (pp=="optical_flow")
            post_processing_[RGBCameraTypes::OpticalFlow] = true;
          if (pp=="object_segment")
            post_processing_[RGBCameraTypes::ObjectSegment] = true;
          if (pp=="category_segment")
            post_processing_[RGBCameraTypes::CategorySegment] = true;
        }
      };
      void EnableDepth(const bool & on){
        if (on) {
          post_processing_[RGBCameraTypes::Depth] = true;
        } else {
          post_processing_[RGBCameraTypes::Depth] = false;
        }
      };
      void EnableOpticalFlow(const bool & on){
        if (on){
          post_processing_[RGBCameraTypes::OpticalFlow] = true;
        } else {
          post_processing_[RGBCameraTypes::OpticalFlow] = false;
        }
      };
      void EnableObjectSegment(const bool & on){
        if (on){
          post_processing_[RGBCameraTypes::ObjectSegment] = true;
        } else {
          post_processing_[RGBCameraTypes::ObjectSegment] = false;
        }
      };
      void EnableCategorySegment(const bool & on){
        if (on){
          post_processing_[RGBCameraTypes::CategorySegment] = true;
        } else {
          post_processing_[RGBCameraTypes::CategorySegment] = false;
        }
      };


     private:
      // simulate sensor
      void RunSimulation_(void);
      USecs UpdateSamplingInterval(void);

      // publishers
      image_transport::Publisher imgPub_;
      image_transport::Publisher depthmapPub_;
      image_transport::Publisher opticFlowPub_;
      image_transport::Publisher objSegmentPub_;
      image_transport::Publisher categorySegmentPub_;
      ros::Publisher cameraInfoPub_;

      Eigen::Vector3d B_r_BC_;
      RGBCameraTypes::Mat4_t T_BC_;

      // rgb camera "callback" functions
      RGBCameraTypes::GetPos_t GetPos_;
      RGBCameraTypes::GetVel_t GetVel_;
      RGBCameraTypes::GetQuat_t GetQuat_;
      RGBCameraTypes::GetOmega_t GetOmega_;
      std::vector<RGBCameraTypes::GetPos_t> GetPosObj_;
      std::vector<RGBCameraTypes::GetVel_t> GetVelObj_;
      std::vector<RGBCameraTypes::GetQuat_t> GetQuatObj_;
      std::vector<RGBCameraTypes::GetOmega_t> GetOmegaObj_;

      // rgb camera parameters
      int channels_;
      int width_;
      int height_;
      double fov_;
      double depth_scale_;
      std::unordered_map<RGBCameraTypes::PostProcessingID, bool> post_processing_= {
        {RGBCameraTypes::Depth, false},
        {RGBCameraTypes::OpticalFlow, false},
        {RGBCameraTypes::ObjectSegment, false},
        {RGBCameraTypes::CategorySegment, false}
      };

      //
      std::mutex image_queue_mutex_;
      std::deque<cv::Mat> image_queue_;

      // auxiliary variables, mainly for unity setup
      bool is_depth{false};
    };
  } // namespace Simulator
} // namespace RPGQ