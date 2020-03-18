#include <rpgq_simulator/implementation/objects/quadrotor_vehicle/quad_and_stereo_rgb_camera.h>
//
#include <rpgq_simulator/implementation/sensors/rgb_camera.h>
#include <rpgq_simulator/implementation/sensors/imu.h>

namespace RPGQ
{
  namespace Simulator
  {
    // constructor
    QuadStereoRGBCamera::QuadStereoRGBCamera(ObjectID id, const Node *prevSimNode, USecs maxSimUSecsInterval) :
      CompositeObject(ID::Object::QuadrotorStereoRGBCamera, prevSimNode, maxSimUSecsInterval)
    {
      // add quadrotor
      quadPtr_ = std::make_shared<QuadrotorVehicle>(id, &simNode_,
        maxSimUSecsInterval);
      AddChild(quadPtr_);

      // add left rgb camera
      {
        leftRGBCameraPtr_ = std::make_shared<RGBCamera>(ID::Sensor::RGBCamera + "_left",
                                                        quadPtr_->GetSimNode());
        leftRGBCameraPtr_->SetPoseCallbacks(std::bind(&QuadrotorDynamics::GetPos, quadPtr_->GetDynamics()),
                                        std::bind(&QuadrotorDynamics::GetVel, quadPtr_->GetDynamics()),
                                        std::bind(&QuadrotorDynamics::GetQuat, quadPtr_->GetDynamics()),
                                        std::bind(&QuadrotorDynamics::GetOmega, quadPtr_->GetDynamics()));
        Eigen::Vector3d B_r_BC(-0.10*std::sqrt(2.0)/2.0, 0.10*std::sqrt(2.0)/2.0, 0.3);
        Eigen::Matrix3d R_BC;
        R_BC << 1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0;
        leftRGBCameraPtr_->SetRelPose(B_r_BC, R_BC);
        AddSensor(leftRGBCameraPtr_);
      }
      // add right rgb camera
      {
        rightRGBCameraPtr_ = std::make_shared<RGBCamera>(ID::Sensor::RGBCamera + "_right",
                                                         quadPtr_->GetSimNode());
        rightRGBCameraPtr_->SetPoseCallbacks(std::bind(&QuadrotorDynamics::GetPos, quadPtr_->GetDynamics()),
                                            std::bind(&QuadrotorDynamics::GetVel, quadPtr_->GetDynamics()),
                                            std::bind(&QuadrotorDynamics::GetQuat, quadPtr_->GetDynamics()),
                                            std::bind(&QuadrotorDynamics::GetOmega, quadPtr_->GetDynamics()));
        Eigen::Vector3d B_r_BC(0.10*std::sqrt(2.0)/2.0, 0.10*std::sqrt(2.0)/2.0, 0.3);
        Eigen::Matrix3d R_BC;
        R_BC << 1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0;
        rightRGBCameraPtr_->SetRelPose(B_r_BC, R_BC);
        AddSensor(rightRGBCameraPtr_);
      }
    }

    // add objects
    void QuadStereoRGBCamera::AddObject(std::shared_ptr<BaseObject> object)
    {
      AddChild(object);
    }
  } // namespace Simulator
} // namespace RPGQ
