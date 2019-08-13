#pragma once

// rpgq rpgq_simulator
#include <rpgq_simulator/framework/objects/base_object.h>
#include <rpgq_simulator/framework/dynamics/base_dynamics.h>
#include <rpgq_simulator/framework/sensors/base_sensor.h>

// others
#include <eigen3/Eigen/Dense>

namespace RPGQ
{
    namespace Simulator
    {
        class PhysicalObject : public BaseObject
        {
        public:
            // constructor & destructor
            PhysicalObject(ObjectID id, const Node* prevSimNode, ObjectType objType, USecs maxSimUSecsInterval):
                BaseObject(id, prevSimNode, objType, maxSimUSecsInterval)
            {};
            virtual ~PhysicalObject() {};

            // add dynamics, sensors, ...
            void AddDynamics(std::shared_ptr<BaseDynamics> dynamics) {dynamics_.emplace(dynamics->GetID(), dynamics);};
            void AddSensor(std::shared_ptr<BaseSensor> sensor) {sensors_.emplace(sensor->GetID(), sensor);};

            // get functions
            virtual Eigen::Vector3d GetPos(void)
            {
                return Eigen::Vector3d(0.0, 0.0, 0.0);
            };
            virtual Eigen::Quaterniond GetQuat(void)
            {
                return Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
            }

        protected:
            // dynamics, sensors, ...
            std::unordered_map<DynamicsID, std::shared_ptr<BaseDynamics>> dynamics_;
            std::unordered_map<SensorID, std::shared_ptr<BaseSensor>> sensors_;

            // (user defined) mediators
            virtual void DynamicsMediator(void) = 0;
            virtual void SensorMediator(void) = 0;
        };

    } // namespace Simulator
} // namespace RPGQ