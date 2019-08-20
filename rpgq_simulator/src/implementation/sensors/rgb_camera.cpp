#include <rpgq_simulator/implementation/sensors/rgb_camera.h>

//// ROS
//#include "sensor_msgs/CameraInfo.h"

namespace RPGQ
{
  namespace Simulator
  {
    RGBCamera::RGBCamera(SensorID id, const Node* prevSimNode,
      Parameters<NUM_PARAMS> params):
      BaseSensor(id, prevSimNode, SECS_TO_USECS(1.0/params.GetDouble(RGBCamera::updateFrequency)))
    {
      // set up publisher
      image_transport::ImageTransport it(pnh_);
      imgPub_ = it.advertise(GetSimTreePath(simNode_) + "/image", 1);
      depthmapPub_ = it.advertise(GetSimTreePath(simNode_) + "/depth", 1);
      opticFlowPub_ = it.advertise(GetSimTreePath(simNode_) + "/optical_flow", 1);
      objSegmentPub_ = it.advertise(GetSimTreePath(simNode_) + "/object_segment", 1);
      categorySegmentPub_ = it.advertise(GetSimTreePath(simNode_) + "/category_segment", 1);
      cameraInfoPub_ = pnh_.advertise<sensor_msgs::CameraInfo>(GetSimTreePath(simNode_) + "/camera_info", 1);

      // general rbg camera variables
      channels_ = params.GetInt(RGBCamera::channels);
      width_ = params.GetInt(RGBCamera::width);
      height_ = params.GetInt(RGBCamera::height);
      fov_ = params.GetDouble(RGBCamera::fov);
      depth_scale_ = params.GetDouble(RGBCamera::depth_scale);

      // image post processing
    }

    sensor_msgs::CameraInfo RGBCamera::GetCameraInfo(void)
    {
      sensor_msgs::CameraInfo camera_info;
      camera_info = {};
      camera_info.width = width_;
      camera_info.height = height_;
      camera_info.distortion_model = "plum_bob";

      float f = (camera_info.height / 2.0) / tan((M_PI * (fov_ / 180.0)) / 2.0);
      float cx = camera_info.width / 2.0;
      float cy = camera_info.height / 2.0;
      float tx = 0.0;
      float ty = 0.0;
      camera_info.D = {0.0, 0.0, 0.0, 0.0, 0.0};
      camera_info.K = {f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0};
      camera_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
      camera_info.P = {f, 0.0, cx, tx, 0.0, f, cy, ty, 0.0, 0.0, 1.0, 0.0};
      return camera_info;
    }

    void RGBCamera::RunSimulation_()
    {

    }

    USecs RGBCamera::UpdateSamplingInterval(void)
    {

    }

    RGBCamera::~RGBCamera()
    {
      imgPub_.shutdown();
      depthmapPub_.shutdown();
      opticFlowPub_.shutdown();
    }

    void RGBCamera::feedImageQueue(const ros::Time & img_timestamp,
      std::unordered_map<RGBCameraTypes::PostProcessingID, cv::Mat> & images)
    {
      {
        cv::Mat rgb_image = images[RGBCameraTypes::RGB];
        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(),
          "bgr8", rgb_image).toImageMsg();
        img_msg->header.stamp = img_timestamp;
        img_msg->header.frame_id = GetSimTreePath(simNode_) + "/image";
        imgPub_.publish(img_msg);
      }
      if (post_processing_[RGBCameraTypes::Depth])
      {
        cv::Mat depth_image = images[RGBCameraTypes::Depth];
        sensor_msgs::ImagePtr depth_img_msg = cv_bridge::CvImage(std_msgs::Header(),
          "bgr8", depth_image).toImageMsg();
        depth_img_msg->header.stamp = img_timestamp;
        depth_img_msg->header.frame_id = GetSimTreePath(simNode_) + "/depth";
        depthmapPub_.publish(depth_img_msg);
      }
      if (post_processing_[RGBCameraTypes::OpticalFlow])
      {
        cv::Mat opticalflow_image = images[RGBCameraTypes::OpticalFlow];
        sensor_msgs::ImagePtr opticalflow_img_msg = cv_bridge::CvImage(std_msgs::Header(),
          "bgr8", opticalflow_image).toImageMsg();
        opticalflow_img_msg->header.stamp = img_timestamp;
        opticalflow_img_msg->header.frame_id = GetSimTreePath(simNode_) + "/optical_flow";
        opticFlowPub_.publish(opticalflow_img_msg);
      }
      if (post_processing_[RGBCameraTypes::ObjectSegment])
      {
        cv::Mat objseg_image = images[RGBCameraTypes::ObjectSegment];
        sensor_msgs::ImagePtr objseg_img_msg = cv_bridge::CvImage(std_msgs::Header(),
          "bgr8", objseg_image).toImageMsg();
        objseg_img_msg->header.stamp = img_timestamp;
        objseg_img_msg->header.frame_id = GetSimTreePath(simNode_) + "/object_segment";
        objSegmentPub_.publish(objseg_img_msg);
      }
      if (post_processing_[RGBCameraTypes::CategorySegment])
      {
        cv::Mat catseg_image = images[RGBCameraTypes::CategorySegment];
        sensor_msgs::ImagePtr catseg_img_msg = cv_bridge::CvImage(std_msgs::Header(),
          "bgr8", catseg_image).toImageMsg();
        catseg_img_msg->header.stamp = img_timestamp;
        catseg_img_msg->header.frame_id = GetSimTreePath(simNode_) + "/category_segment";
        categorySegmentPub_.publish(catseg_img_msg);
      }

    }

  } // namespace Simulator
} // namespace RPGQ