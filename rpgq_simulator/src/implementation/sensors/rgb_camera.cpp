#include <rpgq_simulator/implementation/sensors/rgb_camera.h>

//// ROS
// others
#include <opencv2/highgui.hpp>

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
    }

    sensor_msgs::CameraInfo RGBCamera::GetCameraInfo(const USecs & elapsed_useconds)
    {
      sensor_msgs::CameraInfo camera_info;
      camera_info.header.stamp.fromNSec(1000*elapsed_useconds);;
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

    void RGBCamera::RunSimulation_(void)
    {
      // std::cout << "Simulating RGB Camera" << std::endl;
      PublishImage();
      if (post_processing_[RGBCameraTypes::Depth])
      {
        PublishDepthmap();
      }
      if (post_processing_[RGBCameraTypes::ObjectSegment])
      {
        PublishObjSegment();
      }
      if (post_processing_[RGBCameraTypes::CategorySegment])
      {
        PublishCatSegment();
      }
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

    void RGBCamera::PublishImage()
    {
      if (!image_queue_.empty())
      {
        RGBCameraTypes::RGBImage_t rgb_image = image_queue_.front();
        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(),
          "bgr8", rgb_image.image).toImageMsg();
        img_msg->header.stamp.fromNSec(1000*rgb_image.elapsed_useconds);
        imgPub_.publish(img_msg);

        sensor_msgs::CameraInfo camera_info = GetCameraInfo(rgb_image.elapsed_useconds);
        cameraInfoPub_.publish(camera_info);
        //
        image_queue_.pop_front();
//
//        static int counter = 0;
//        counter++;
//        if (counter >= 25)
//        {
//          char filename[80], counterString[10];
//          std::strcpy(filename, "/home/sysadmin/Desktop/Images/Multi/");
//          std::sprintf(counterString, "%05d", counter-25);
//          std::strcat(filename, counterString);
//          std::strcat(filename, ".png");
//          cv::imwrite(filename, rgb_image.image);
//        }

      }
    }

    void RGBCamera::PublishDepthmap()
    {
      if (!depth_queue_.empty())
      {
        RGBCameraTypes::Depthmap_t depth_map = depth_queue_.front();
        sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(),
          "bgr8", depth_map.image).toImageMsg();
        depth_msg->header.stamp.fromNSec(1000*depth_map.elapsed_useconds);
        depthmapPub_.publish(depth_msg);
        depth_queue_.pop_front();
//        char filename[80], counterString[10];

//        static int counter = 0;
//        std::strcpy(filename, "/home/sysadmin/Desktop/Images/Depth/");
//        std::sprintf(counterString, "%05d", counter++);
//        std::strcat(filename, counterString);
//        std::strcat(filename, ".png");
//        cv::imwrite(filename, depth_map.image);
      }
    }

//    void RGBCamera::PublishOpticFlow(const RGBCameraTypes::OpticFlow_t &optic_flow)
//    {
//
//    }

    void RGBCamera::PublishObjSegment()
    {
      if (!obj_seg_queue_.empty())
      {
        RGBCameraTypes::Segement_t obj_seg = obj_seg_queue_.front();
        sensor_msgs::ImagePtr obj_seg_msg = cv_bridge::CvImage(std_msgs::Header(),
          "bgr8", obj_seg.image).toImageMsg();
        obj_seg_msg->header.stamp.fromNSec(1000*obj_seg.elapsed_useconds);
        objSegmentPub_.publish(obj_seg_msg);
        obj_seg_queue_.pop_front();

//        static int counter = 0;
//        char filename[80], counterString[10];
//        std::strcpy(filename, "/home/sysadmin/Desktop/Images/ObjSeg/");
//        std::sprintf(counterString, "%05d", counter++);
//        std::strcat(filename, counterString);
//        std::strcat(filename, ".png");
//        cv::imwrite(filename, obj_seg.image);
      }
    }

    void RGBCamera::PublishCatSegment()
    {
      if (!category_seg_queue_.empty())
      {
        RGBCameraTypes::Segement_t category_seg = category_seg_queue_.front();
        sensor_msgs::ImagePtr category_seg_msg = cv_bridge::CvImage(std_msgs::Header(),
          "bgr8", category_seg.image).toImageMsg();
        category_seg_msg->header.stamp.fromNSec(1000*category_seg.elapsed_useconds);
        categorySegmentPub_.publish(category_seg_msg);
        category_seg_queue_.pop_front();

//        static int counter = 0;
//        char filename[80], counterString[10];
//        std::strcpy(filename, "/home/sysadmin/Desktop/Images/CatSeg/");
//        std::sprintf(counterString, "%05d", counter++);
//        std::strcat(filename, counterString);
//        std::strcat(filename, ".png");
//        cv::imwrite(filename, category_seg.image);
      }
    }

    void RGBCamera::FeedImageQueue(const ros::Time & img_timestamp,
      std::unordered_map<RGBCameraTypes::PostProcessingID, cv::Mat> & images)
    {
      {
        queue_mutex_.lock();
        RGBCameraTypes::RGBImage_t rgb_image;
        rgb_image.image = images[RGBCameraTypes::RGB];
        rgb_image.elapsed_useconds = ROSTIME_TO_USECS(img_timestamp);
        image_queue_.push_back(rgb_image);
        queue_mutex_.unlock();
      }
      if (post_processing_[RGBCameraTypes::Depth])
      {
        queue_mutex_.lock();
        RGBCameraTypes::Depthmap_t depth_image;
        depth_image.image = images[RGBCameraTypes::Depth];
        depth_image.elapsed_useconds = ROSTIME_TO_USECS(img_timestamp);
        depth_queue_.push_back(depth_image);
        queue_mutex_.unlock();
      }
//      if (post_processing_[RGBCameraTypes::OpticalFlow])
//      {
//        queue_mutex_.lock();
//        RGBCameraTypes::OpticFlow_t optical_flow_image = images[RGBCameraTypes::OpticalFlow];
//        optical_flow_queue_.push_back(optical_flow_image);
//        queue_mutex_.unlock();
//      }
      if (post_processing_[RGBCameraTypes::ObjectSegment])
      {
        queue_mutex_.lock();
        RGBCameraTypes::Segement_t obj_seg_image;
        obj_seg_image.image = images[RGBCameraTypes::ObjectSegment];
        obj_seg_image.elapsed_useconds = ROSTIME_TO_USECS(img_timestamp);
        obj_seg_queue_.push_back(obj_seg_image);
        queue_mutex_.unlock();
      }
      if (post_processing_[RGBCameraTypes::CategorySegment])
      {
        queue_mutex_.lock();
        RGBCameraTypes::Segement_t category_seg_image;
        category_seg_image.image = images[RGBCameraTypes::CategorySegment];
        category_seg_image.elapsed_useconds = ROSTIME_TO_USECS(img_timestamp);
        category_seg_queue_.push_back(category_seg_image);
        queue_mutex_.unlock();
      }

      // hack
      RunSimulation_();
    }

  } // namespace Simulator
} // namespace RPGQ