#pragma once

/*************************************************************
/* This is mainly copied from Henri's event camera simulator:
/*
/* https://github.com/uzh-rpg/rpg_event_camera_simulator
/*************************************************************/

// ros
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>

// rpgq rpgq_simulator
#include <rpgq_simulator/framework/sensors/base_sensor.h>
#include <rpgq_simulator/implementation/sensors/event_camera/event_camera_types.h>
#include <rpgq_simulator/implementation/sensors/event_camera/renderer.h>

// standard library
#include <memory>
#include <random>

namespace RPGQ
{
    namespace Simulator
    {
        class EventCamera : public BaseSensor
        {
        public:
            EventCamera(SensorID id, const Node* prevSimNode, USecs maxSimUSecsInterval);
            ~EventCamera();

            // add objects
            void AddObjectModelAndCallbacks(const std::string &path,
                                EventCameraTypes::GetPos_t GetPos,
                                EventCameraTypes::GetVel_t GetVel,
                                EventCameraTypes::GetQuat_t GetQuat,
                                EventCameraTypes::GetOmega_t GetOmega);

            // public set functions
            void SetSceneModel(const std::string &path)
            {
                renderer_.SetSceneModel(path);
                sceneInitialized_ = true;
            };
            void SetPoseCallbacks(EventCameraTypes::GetPos_t GetPos,
                                EventCameraTypes::GetVel_t GetVel,
                                EventCameraTypes::GetQuat_t GetQuat,
                                EventCameraTypes::GetOmega_t GetOmega)
            {
                GetPos_ = GetPos;
                GetVel_ = GetVel;
                GetQuat_ = GetQuat;
                GetOmega_ = GetOmega;
            };
            void SetRelPose(Eigen::Vector3d &B_r_BC, Eigen::Matrix3d &R_BC)
            {
                B_r_BC_ = B_r_BC;
                T_BC_.block<3,3>(0,0) = R_BC;
                T_BC_.block<3,1>(0,3) = B_r_BC_;
                T_BC_.row(3) << 0.0, 0.0, 0.0, 1.0;
            };

            // public get functions
            EventCameraTypes::Mat4_t GetRelPose(void) {return T_BC_;};

        private:
            // simulate sensor
            void RunSimulation_(void);
            USecs UpdateSamplingInterval(void);

            // publishers
            image_transport::Publisher framePub_;
            image_transport::Publisher depthmapPub_;
            image_transport::Publisher opticFlowPub_;
            ros::Publisher eventsPub_;
            ros::Publisher imuPub_;
            ros::Publisher cameraInfoPub_;
            void PublishFrame(const EventCameraTypes::Image_t& image);
            void PublishDepthmap(const EventCameraTypes::Depthmap_t& depthmap);
            void PublishOpticFlow(const EventCameraTypes::OpticFlow_t& opticFlow);
            void PublishEvents(const EventCameraTypes::Events& events);
            void PublishIMU(void);
            void PublishCameraInfo(void);

            // general event camera variables
            bool initialized_;
            bool sceneInitialized_;
            Renderer renderer_;
            cv::Size imageSize_;
            std::shared_ptr<EventCameraTypes::Image_t> image_;
            std::shared_ptr<EventCameraTypes::Depthmap_t> depthmap_;
            std::shared_ptr<EventCameraTypes::OpticFlow_t> opticFlow_;
            EventCameraTypes::Image_t previousImage_;
            EventCameraTypes::Image_t triggerImage_;
            USecs previousImageTime_;
            uint8_t numObjects_;
            std::default_random_engine gen_;
            std::normal_distribution<EventCameraTypes::Intensity_t> sensorNoise_;

            // event camera parameters
            double gaussianBlurSigma_;        
            double adaptiveSamplingLambda_;
            bool useLogImage_;
            bool usePredictedBrightnessChange_;
            EventCameraTypes::Intensity_t logEps_;
            EventCameraTypes::Intensity_t intensityTol_;
            EventCameraTypes::Intensity_t Cp_, Cn_, sigmaCp_, sigmaCn_;
            USecs dusecsFrame_;
            int32_t imageWidth_;
            int32_t imageHeight_;
            double fx_;
            double fy_;
            double cx_;
            double cy_;
            Eigen::Vector3d B_r_BC_;
            EventCameraTypes::Mat4_t T_BC_;

            // event camera "callback" functions
            EventCameraTypes::GetPos_t GetPos_;
            EventCameraTypes::GetVel_t GetVel_;
            EventCameraTypes::GetQuat_t GetQuat_;
            EventCameraTypes::GetOmega_t GetOmega_;
            std::vector<EventCameraTypes::GetPos_t> GetPosObj_;
            std::vector<EventCameraTypes::GetVel_t> GetVelObj_;
            std::vector<EventCameraTypes::GetQuat_t> GetQuatObj_;
            std::vector<EventCameraTypes::GetOmega_t> GetOmegaObj_;

            // simulation timing variables
            USecs usecsNextTFrame_, usecsNextTFlow_;
            USecs usecsNextCameraInfo_, usecsCameraInfoInterval_;

            // auxiliary functions
            int colorwheel_[55][3];
            void GaussianBlur(EventCameraTypes::Image_t& image, double sigma);
            std::pair<EventCameraTypes::Intensity_t,EventCameraTypes::Intensity_t> ComputeMaximumPredictedAbsBrightnessChange(const EventCameraTypes::OpticFlow_t &opticFlow, const EventCameraTypes::Image_t& image);
            double ComputeMaximumOpticFlowMagnitude(const EventCameraTypes::OpticFlow_t& opticFlow) const;
            EventCameraTypes::Events ComputeEvents(const EventCameraTypes::Image_t& image, const USecs t);
        };

    } // namespace Simulator
} // namespace RPGQ
