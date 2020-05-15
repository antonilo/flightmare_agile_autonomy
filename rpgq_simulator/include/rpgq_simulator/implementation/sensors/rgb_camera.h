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
        typedef int8_t Intensity_t;
        typedef cv::Mat Image_t;

        struct RGBImage_t
        {
          Image_t image;
          USecs elapsed_useconds;
        };
        struct Depthmap_t
        {
          Image_t image;
          USecs elapsed_useconds;
        };
        struct Segement_t
        {
          Image_t image;
          USecs elapsed_useconds;
        };

        struct OpticFlow_t
        {
          Image_t image;
          USecs elapsed_useconds;
        };

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

      void FeedImageQueue(const ros::Time & img_timestamp,
        std::unordered_map<RGBCameraTypes::PostProcessingID, cv::Mat> & images);

      // public get functions
      RGBCameraTypes::Mat4_t GetRelPose(void) {return T_BC_;};
      int GetChannel(void) { return channels_; };
      int GetWidth(void) { return width_; };
      int GetHeight(void) { return height_; };
      double GetFov(void) {return fov_; };
      double GetDepthScale(void) { return depth_scale_; };
      std::vector<std::string> GetPostProcessing(void);
      sensor_msgs::CameraInfo GetCameraInfo(const USecs & elapsed_useconds);

      // get image functions
      void GetRGBImage(cv::Mat & rgb_img);
      void GetDepthmap(cv::Mat & depth_map);
      void GetOpticalFlow(cv::Mat & optical_flow);
      void GetObjSegment(cv::Mat & obj_segment);
      void GetCatSegment(cv::Mat & cat_segment);
      
      // public set functions
      void SetChanel(const int & channels) { channels_ = channels;};
      void SetWidth(const int & width) {width_ = width; };
      void SetHeight(const int & height) {height_ = height; };
      void SetFov(const double & fov) {fov_ = fov; };
      void SetDepthScale(const double & depth_scale)
      {
        depth_scale_ = depth_scale;
      };
      void SetPostProcessing(const std::vector<std::string> & post_processing);
      void EnableDepth(const bool & on);
      void EnableOpticalFlow(const bool & on);
      void EnableObjectSegment(const bool & on);
      void EnableCategorySegment(const bool & on);

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

      // publish functions
      void PublishImage();
      void PublishDepthmap();
      void PublishOpticFlow(); // const RGBCameraTypes::OpticFlow_t& optic_flow);
      void PublishObjSegment();
      void PublishCatSegment();

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

      // camera relative pose with respect to the vehicle body
      Eigen::Vector3d B_r_BC_;
      RGBCameraTypes::Mat4_t T_BC_;

      //
      std::unordered_map<RGBCameraTypes::PostProcessingID, bool> post_processing_;

      // image data buffer
      std::mutex queue_mutex_;
      int queue_size_ = 1;
      std::deque<RGBCameraTypes::RGBImage_t> image_queue_;
      std::deque<RGBCameraTypes::Depthmap_t>  depth_queue_;
      std::deque<RGBCameraTypes::OpticFlow_t> optical_flow_queue_;
      std::deque<RGBCameraTypes::Segement_t>  obj_seg_queue_;
      std::deque<RGBCameraTypes::Segement_t>  category_seg_queue_;

      // auxiliary variables, mainly for unity setup
      bool is_depth{false};
    };
  } // namespace Simulator
} // namespace RPGQ