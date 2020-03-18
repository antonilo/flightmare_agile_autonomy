#include <rpgq_simulator/implementation/objects/quadrotor_vehicle/quad_and_stereo_event_camera.h>

// rpgq rpgq_simulator
#include <rpgq_simulator/implementation/sensors/event_camera.h>
#include <rpgq_simulator/implementation/sensors/imu.h>

namespace RPGQ
{
    namespace Simulator
    {
        // constructor
        QuadStereoEventCamera::QuadStereoEventCamera(ObjectID id, const Node* prevSimNode, USecs maxSimUSecsInterval):
            CompositeObject(ID::Object::QuadrotorStereoEventCamera, prevSimNode, maxSimUSecsInterval)
        {
            // add children
            quadPtr_ = std::make_shared<QuadrotorVehicle>(id, &simNode_,
              maxSimUSecsInterval);
            AddChild(quadPtr_);

            // add sensor
            {
                leftEventCameraPtr_ = std::make_shared<EventCamera>(ID::Sensor::EventCamera + "_left", quadPtr_->GetSimNode(), 33333); // simulate at least every 33333us -> 30Hz        
                leftEventCameraPtr_->SetPoseCallbacks(std::bind(&QuadrotorDynamics::GetPos, quadPtr_->GetDynamics()),
                                                std::bind(&QuadrotorDynamics::GetVel, quadPtr_->GetDynamics()),
                                                std::bind(&QuadrotorDynamics::GetQuat, quadPtr_->GetDynamics()),
                                                std::bind(&QuadrotorDynamics::GetOmega, quadPtr_->GetDynamics()));
                Eigen::Vector3d B_r_BC(-0.10*std::sqrt(2.0)/2.0, 0.10*std::sqrt(2.0)/2.0, 0.0);
                Eigen::Matrix3d R_BC = Eigen::AngleAxisd(1.7177715175,
                                                        Eigen::Vector3d(-0.862856209461017,0.357406744336593,-0.357406744336593)).toRotationMatrix(); // forward looking between (B_e_X and B_e_Y)
                leftEventCameraPtr_->SetRelPose(B_r_BC, R_BC);
                AddSensor(leftEventCameraPtr_);
            }


            {
                rightEventCameraPtr_ = std::make_shared<EventCamera>(ID::Sensor::EventCamera + "_right", quadPtr_->GetSimNode(), 33333); // simulate at least every 33333us -> 30Hz        
                rightEventCameraPtr_->SetPoseCallbacks(std::bind(&QuadrotorDynamics::GetPos, quadPtr_->GetDynamics()),
                                                std::bind(&QuadrotorDynamics::GetVel, quadPtr_->GetDynamics()),
                                                std::bind(&QuadrotorDynamics::GetQuat, quadPtr_->GetDynamics()),
                                                std::bind(&QuadrotorDynamics::GetOmega, quadPtr_->GetDynamics()));
                Eigen::Vector3d B_r_BC(0.10*std::sqrt(2.0)/2.0, -0.10*std::sqrt(2.0)/2.0, 0.0);
                Eigen::Matrix3d R_BC = Eigen::AngleAxisd(1.7177715175,
                                                        Eigen::Vector3d(-0.862856209461017,0.357406744336593,-0.357406744336593)).toRotationMatrix(); // forward looking between (B_e_X and B_e_Y)
                rightEventCameraPtr_->SetRelPose(B_r_BC, R_BC);
                AddSensor(rightEventCameraPtr_);
            }


            {
                leftImuPtr_ = std::make_shared<IMU>(ID::Sensor::IMU, leftEventCameraPtr_->GetSimNode()); // default 1kHz -> simulate every 1000us
                leftImuPtr_->SetCallbackFunctions(std::bind(&QuadrotorDynamics::GetAcc, quadPtr_->GetDynamics()),
                                            std::bind(&QuadrotorDynamics::GetQuat, quadPtr_->GetDynamics()),
                                            std::bind(&QuadrotorDynamics::GetOmega, quadPtr_->GetDynamics()),
                                            std::bind(&QuadrotorDynamics::GetPsi, quadPtr_->GetDynamics()));
                EventCameraTypes::Mat4_t T_BC = leftEventCameraPtr_->GetRelPose();
                Eigen::Vector3d B_r_BS = T_BC.block<3,1>(0,3);
                Eigen::Matrix3d R_SB = (T_BC.block<3,3>(0,0)).transpose();
                leftImuPtr_->SetRelPose(B_r_BS, R_SB);
                AddSensor(leftImuPtr_);
            }


            {
                rightImuPtr_ = std::make_shared<IMU>(ID::Sensor::IMU, rightEventCameraPtr_->GetSimNode()); // default 1kHz -> simulate every 1000us
                rightImuPtr_->SetCallbackFunctions(std::bind(&QuadrotorDynamics::GetAcc, quadPtr_->GetDynamics()),
                                            std::bind(&QuadrotorDynamics::GetQuat, quadPtr_->GetDynamics()),
                                            std::bind(&QuadrotorDynamics::GetOmega, quadPtr_->GetDynamics()),
                                            std::bind(&QuadrotorDynamics::GetPsi, quadPtr_->GetDynamics()));
                EventCameraTypes::Mat4_t T_BC = rightEventCameraPtr_->GetRelPose();
                Eigen::Vector3d B_r_BS = T_BC.block<3,1>(0,3);
                Eigen::Matrix3d R_SB = (T_BC.block<3,3>(0,0)).transpose();
                rightImuPtr_->SetRelPose(B_r_BS, R_SB);
                AddSensor(rightImuPtr_);
            }
        }


        // add objects
        void QuadStereoEventCamera::AddObject(std::shared_ptr<BaseObject> object,
                                                const std::string &objectModelPath,
                                                EventCameraTypes::GetPos_t GetPos,
                                                EventCameraTypes::GetVel_t GetVel,
                                                EventCameraTypes::GetQuat_t GetQuat,
                                                EventCameraTypes::GetOmega_t GetOmega)
        {
            // add object to composite
            AddChild(object);

            // add object to event camera
            leftEventCameraPtr_->AddObjectModelAndCallbacks(objectModelPath, GetPos, GetVel, GetQuat, GetOmega);
            rightEventCameraPtr_->AddObjectModelAndCallbacks(objectModelPath, GetPos, GetVel, GetQuat, GetOmega);
        }


        // set functions
        void QuadStereoEventCamera::SetSceneModel(const std::string &path)
        {
            leftEventCameraPtr_->SetSceneModel(path);
            rightEventCameraPtr_->SetSceneModel(path);
        };

    } // namespace Simulator
} // namespace RPGQ