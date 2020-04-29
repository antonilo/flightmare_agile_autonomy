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
      Eigen::Vector3d GetPos(void) {return quadPtr_->GetPos(); };
      Eigen::Vector3d GetVel(void) {return quadPtr_->GetVel();};
      Eigen::Vector3d GetAcc(void) {return quadPtr_->GetAcc();};
      Eigen::Quaterniond GetQuat(void) {return quadPtr_->GetQuat();};
      Eigen::Vector3d GetOmega(void) {return quadPtr_->GetOmega();};
      Eigen::Vector3d GetPsi(void) {return quadPtr_->GetPsi();};
      const std::shared_ptr<QuadrotorVehicle> GetQuad(void)
        {return quadPtr_;};
      const std::shared_ptr<RGBCamera> GetRGBCamera(void)
        {return rgbCameraPtr_;};

      void EnableDepth(const bool & on) 
        {rgbCameraPtr_->EnableDepth(on); };
      void EnableOpticalFlow(const bool & on)
        {rgbCameraPtr_->EnableOpticalFlow(on); };
      void EnableObjectSegment(const bool & on)
        {rgbCameraPtr_->EnableObjectSegment(on); };
      void EnableCategorySegment(const bool & on)
        {rgbCameraPtr_->EnableCategorySegment(on); };
      void SetWidth(const int & width) 
        {rgbCameraPtr_->SetWidth(width); };
      void SetHeight(const int & height) 
        {rgbCameraPtr_->SetHeight(height); };
      void SetFov(const double & fov) 
        {rgbCameraPtr_->SetFov(fov); };

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
