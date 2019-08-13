#include <rpgq_simulator/implementation/unity/unity_gate.h>

namespace RPGQ
{
  namespace Simulator
  {
    UnityGate::UnityGate(ObjectID id, PrefabID prefab_id)
      : UnityObject(id, prefab_id, STATIC_OBJECT)
    {
      ID_ = id;
      prefab_ID_ = prefab_id;
    };
  }
}