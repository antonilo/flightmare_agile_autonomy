#pragma once

// rpgq rpgq_simulator
#include <rpgq_simulator/framework/objects/composite_object.h>
#include <rpgq_simulator/implementation/objects/quadrotor_vehicle/quadrotor_vehicle.h>
#include <rpgq_simulator/implementation/sensors/rgb_camera.h>

namespace RPGQ
{
  namespace Simulator
  {
    class QuadRGBCamera : public CompositeObject
    {
     public:
      // constructor & deconstructor
      QuadRGBCamera(ObjectID id, const Node* prevSimNode, USecs maxSimUSecsInterval);
      ~QuadRGBCamera(){};

      // Adding objects ...
      void AddObject(std::shared_ptr<BaseObject> object);

      // public get functions
      const std::shared_ptr<QuadrotorVehicle> GetQuad(void)
        {return quadPtr_;};
      const std::shared_ptr<RGBCamera> GetRGBCamera(void)
        {return rgbCameraPtr_;};

     private:
      // (user defined) mediators
      void ChildrenMediator(void) {};
      void SensorMediator(void) {};

      // compute (user defined) object output
      void ComputeOutputOnce(void) {};
      void ComputeOutputContinuously(void) {};

      // auxiliary variables
      std::shared_ptr<QuadrotorVehicle> quadPtr_;
      std::shared_ptr<RGBCamera> rgbCameraPtr_;
    };
  } //  namespace Simulator
} // namespace RPGQ
