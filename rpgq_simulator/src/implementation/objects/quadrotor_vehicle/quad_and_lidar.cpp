#include <rpgq_simulator/implementation/objects/quadrotor_vehicle/quad_and_lidar.h>

// rpgq rpgq_simulator

namespace RPGQ
{
  namespace Simulator
  {
    // constructor
    QuadLidar::QuadLidar(ObjectID id, const Node *prevSimNode, USecs maxSimUSecsInterval) :
      CompositeObject(ID::Object::QuadLidar, prevSimNode, maxSimUSecsInterval)
    {
      /* ------------------- */
      //  add quadrotor vehicle
      /* ------------------- */
      quadPtr_ = std::make_shared<QuadrotorVehicle>(id, &simNode_, maxSimUSecsInterval);
      AddChild(quadPtr_);

      /* ------------------- */
      //  add Lidar
      //  and set up relative pose
      /* ------------------- */
      lidarPtr_ = std::make_shared<Lidar>(ID::Sensor::Lidar,
        quadPtr_->GetSimNode()); // simulate at least every

      Eigen::Vector3d B_r_BS(0.0, 0.0, 0.4);
      Eigen::Matrix3d R_BS;
      R_BS << 1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0;
      lidarPtr_->SetRelPose(B_r_BS, R_BS);
      AddSensor(lidarPtr_);
    }

    // add objects
    void QuadLidar::AddObject(std::shared_ptr<BaseObject> object)
    {
      AddChild(object);
    }
  } // namespace Simulator
} // namespace RPGQ
