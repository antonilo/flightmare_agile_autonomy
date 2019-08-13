#pragma once

#include <ros/ros.h>

// rpgq rpgq_common
#include <rpgq_common/time/ext_timer.h>

// rpgq rpgq_simulator
#include <rpgq_simulator/simulator_types.h>

// standard library
#include <memory>
#include <unordered_map>
#include <eigen3/Eigen/Dense>

namespace RPGQ
{
  namespace Simulator
  {
     enum UnityObjType
     {
       CAMERA = 0,
       LIGHT = 1,
       STATIC_OBJECT = 2,
       DYNAMIC_OBJECT = 3,
     };

     class UnityObject
     {
      public:
       // constructor & destructor
       UnityObject(ObjectID id, PrefabID prefab_id, UnityObjType unity_obj_type):
         id_(id),
         prefab_id_(prefab_id),
         unityObjType_(unity_obj_type)
       {};
       virtual ~UnityObject(){};

       virtual Eigen::Vector3d GetPos(void) = 0;
       virtual Eigen::Quaterniond GetQuat(void) = 0;
       virtual Eigen::Vector3d GetSize(void) = 0;

       // public get functions
       const ObjectID GetID(void) {return id_;};
       const PrefabID GetPrefabID(void) {return prefab_id_;};
       const UnityObjType GetUnityObjectType(void) { return unityObjType_; };

      protected:
       // general physical object variables
       const ObjectID id_;
       const PrefabID prefab_id_;
       const UnityObjType unityObjType_;

     };

  }
}