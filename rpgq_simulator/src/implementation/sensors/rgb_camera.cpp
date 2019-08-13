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

      cameraInfoPub_ = pnh_.advertise<sensor_msgs::CameraInfo>(GetSimTreePath(simNode_) + "/camera_info", 1);

      // general rbg camera variables
      channels_ = params.GetInt(RGBCamera::channels);
      width_ = params.GetInt(RGBCamera::width);
      height_ = params.GetInt(RGBCamera::height);
      fov_ = params.GetDouble(RGBCamera::fov);
      depth_scale_ = params.GetDouble(RGBCamera::depth_scale);
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
      const cv::Mat & image)
    {
      sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
      img_msg->header.stamp = img_timestamp;
      img_msg->header.frame_id = GetSimTreePath(simNode_) + "/image";
      imgPub_.publish(img_msg);
    }

  } // namespace Simulator
} // namespace RPGQ