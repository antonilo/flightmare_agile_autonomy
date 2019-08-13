#pragma once

#include <rpgq_simulator/framework/unity/unity_object.h>

// eigen
#include <eigen3/Eigen/Dense>

namespace RPGQ
{
  namespace Simulator
  {
    class UnityGate : public UnityObject
    {
     public:

      UnityGate(ObjectID id, PrefabID prefab_id);
      ~UnityGate(){};

      // publich set functions
      void SetPosition(const Eigen::Vector3d & position){ position_ = position;};
      void SetRotation(const Eigen::Quaterniond & quaternion) {quat_ = quaternion; };
      void SetSize(const Eigen::Vector3d & size) {size_ = size;};

      // publich get functions
      Eigen::Vector3d GetPos(void) { return position_; };
      Eigen::Quaterniond GetQuat(void) { return quat_;};
      Eigen::Vector3d GetSize(void) { return size_; };

     private:
      ObjectID ID_;
      PrefabID prefab_ID_;

      Eigen::Vector3d position_{0.0, 0.0, 0.0};
      Eigen::Quaterniond quat_{1.0, 0.0, 0.0, 0.0};
      Eigen::Vector3d size_{1.0, 1.0, 1.0};

    };
  }
}