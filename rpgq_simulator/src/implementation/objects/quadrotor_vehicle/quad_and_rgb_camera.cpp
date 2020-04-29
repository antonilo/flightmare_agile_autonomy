#include <rpgq_simulator/implementation/objects/quadrotor_vehicle/quad_and_rgb_camera.h>

// rpgq rpgq_simulator
#include <rpgq_simulator/implementation/sensors/imu.h>

namespace RPGQ
{
  namespace Simulator
  {
    // constructor
    QuadRGBCamera::QuadRGBCamera(ObjectID id, const Node *prevSimNode, USecs maxSimUSecsInterval) :
      CompositeObject(ID::Object::QuadrotorRGBCamera, prevSimNode, maxSimUSecsInterval)
    {
      /* ------------------- */
      //  add quadrotor vehicle
      /* ------------------- */
      quadPtr_ = std::make_shared<QuadrotorVehicle>(id, &simNode_, maxSimUSecsInterval);
      AddChild(quadPtr_);

      /* ------------------- */
      //  add RGB camera
      //  and set up relative pose
      /* ------------------- */
      rgbCameraPtr_ = std::make_shared<RGBCamera>(ID::Sensor::RGBCamera,
        quadPtr_->GetSimNode()); // simulate at least every 33333us -> 30Hz
      rgbCameraPtr_->SetPoseCallbacks(std::bind(&QuadrotorDynamics::GetPos, quadPtr_->GetDynamics()),
                                      std::bind(&QuadrotorDynamics::GetVel, quadPtr_->GetDynamics()),
                                      std::bind(&QuadrotorDynamics::GetQuat, quadPtr_->GetDynamics()),
                                      std::bind(&QuadrotorDynamics::GetOmega, quadPtr_->GetDynamics()));
      Eigen::Vector3d B_r_BC(0.0, 0.5, 0.0);
      // forward looking between (B_e_X and B_e_Y)
      // Eigen::Matrix3d R_BC = Eigen::AngleAxisd(1.7177715175,
      //  Eigen::Vector3d(-0.862856209461017,0.357406744336593,-0.357406744336593)).toRotationMatrix();
      Eigen::Matrix3d R_BC;
      R_BC << 1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0;
      rgbCameraPtr_->SetRelPose(B_r_BC, R_BC);
      AddSensor(rgbCameraPtr_);
    }

    // add objects
    void QuadRGBCamera::AddObject(std::shared_ptr<BaseObject> object)
    {
      AddChild(object);
    }

  } // namespace Simulator
} // namespace RPGQ
