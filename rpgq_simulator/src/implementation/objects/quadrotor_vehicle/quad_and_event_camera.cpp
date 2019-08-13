#include <rpgq_simulator/implementation/objects/quadrotor_vehicle/quad_and_event_camera.h>

// rpgq rpgq_simulator
#include <rpgq_simulator/implementation/sensors/event_camera.h>
#include <rpgq_simulator/implementation/sensors/imu.h>

namespace RPGQ
{
    namespace Simulator
    {
        // constructor
        QuadEventCamera::QuadEventCamera(ObjectID id, const Node* prevSimNode, USecs maxSimUSecsInterval):
            CompositeObject(ID::Object::QuadrotorEventCamera, prevSimNode, maxSimUSecsInterval)
        {
            // add children
            quadPtr_ = std::make_shared<QuadrotorVehicle>(id, &simNode_, maxSimUSecsInterval);
            AddChild(quadPtr_);

            // add sensor
            eventCameraPtr_ = std::make_shared<EventCamera>(ID::Sensor::EventCamera, quadPtr_->GetSimNode(), 33333); // simulate at least every 33333us -> 30Hz        
            eventCameraPtr_->SetPoseCallbacks(std::bind(&QuadrotorDynamics::GetPos, quadPtr_->GetDynamics()),
                                            std::bind(&QuadrotorDynamics::GetVel, quadPtr_->GetDynamics()),
                                            std::bind(&QuadrotorDynamics::GetQuat, quadPtr_->GetDynamics()),
                                            std::bind(&QuadrotorDynamics::GetOmega, quadPtr_->GetDynamics()));
            Eigen::Vector3d B_r_BC(0.0, 0.0, 0.0);
            Eigen::Matrix3d R_BC = Eigen::AngleAxisd(1.7177715175,
              Eigen::Vector3d(-0.862856209461017,0.357406744336593,-0.357406744336593)).toRotationMatrix(); // forward looking between (B_e_X and B_e_Y)
            eventCameraPtr_->SetRelPose(B_r_BC, R_BC);
            AddSensor(eventCameraPtr_);

            imuPtr_ = std::make_shared<IMU>(ID::Sensor::IMU, eventCameraPtr_->GetSimNode()); // default 1kHz -> simulate every 1000us
            imuPtr_->SetCallbackFunctions(std::bind(&QuadrotorDynamics::GetAcc, quadPtr_->GetDynamics()),
                                        std::bind(&QuadrotorDynamics::GetQuat, quadPtr_->GetDynamics()),
                                        std::bind(&QuadrotorDynamics::GetOmega, quadPtr_->GetDynamics()),
                                        std::bind(&QuadrotorDynamics::GetPsi, quadPtr_->GetDynamics()));
            EventCameraTypes::Mat4_t T_BC = eventCameraPtr_->GetRelPose();
            Eigen::Vector3d B_r_BS = T_BC.block<3,1>(0,3);
            Eigen::Matrix3d R_SB = (T_BC.block<3,3>(0,0)).transpose();
            imuPtr_->SetRelPose(B_r_BS, R_SB);
            AddSensor(imuPtr_);
        }


        // add objects
        void QuadEventCamera::AddObject(std::shared_ptr<BaseObject> object,
                                                const std::string &objectModelPath,
                                                EventCameraTypes::GetPos_t GetPos,
                                                EventCameraTypes::GetVel_t GetVel,
                                                EventCameraTypes::GetQuat_t GetQuat,
                                                EventCameraTypes::GetOmega_t GetOmega)
        {
            // add object to composite
            AddChild(object);

            // add object to event camera
            eventCameraPtr_->AddObjectModelAndCallbacks(objectModelPath, GetPos, GetVel, GetQuat, GetOmega);
        }


        // set functions
        void QuadEventCamera::SetSceneModel(const std::string &path)
        {
            eventCameraPtr_->SetSceneModel(path);
        };

    } // namespace Simulator
} // namespace RPGQ