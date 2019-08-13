#pragma once

// rpgq rpgq_simulator
#include <rpgq_simulator/framework/objects/composite_object.h>
#include <rpgq_simulator/implementation/objects/quadrotor_vehicle/quadrotor_vehicle.h>
#include <rpgq_simulator/implementation/sensors/event_camera/event_camera_types.h>

namespace RPGQ
{
    namespace Simulator
    {
        // forward declarations
        class EventCamera;
        class IMU;

        class QuadStereoEventCamera : public CompositeObject
        {
        public:
            // constructor & destructor
            QuadStereoEventCamera(ObjectID id, const Node* prevSimNode, USecs maxSimUSecsInterval);
            ~QuadStereoEventCamera() {};

            // add objects, ...
            void AddObject(std::shared_ptr<BaseObject> object,
                        const std::string &objectModelPath,
                        EventCameraTypes::GetPos_t GetPos,
                        EventCameraTypes::GetVel_t GetVel,
                        EventCameraTypes::GetQuat_t GetQuat,
                        EventCameraTypes::GetOmega_t GetOmega);

            // public get functions
            const std::shared_ptr<QuadrotorVehicle> GetQuad(void) {return quadPtr_;};

            // public set functions
            void SetSceneModel(const std::string &path);

        private:
            // (user defined) mediators
            void ChildrenMediator(void) {};
            void SensorMediator(void) {};

            // compute (user defined) object output
            void ComputeOutputOnce(void) {};
            void ComputeOutputContinuously(void) {};

            // auxiliary variables
            std::shared_ptr<QuadrotorVehicle> quadPtr_;
            std::shared_ptr<EventCamera> leftEventCameraPtr_, rightEventCameraPtr_;
            std::shared_ptr<IMU> leftImuPtr_, rightImuPtr_;
        };

    } // namespace Simulator
} // namespace RPGQ