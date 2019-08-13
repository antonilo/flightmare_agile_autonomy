#pragma once

// rpgq rpgq_simulator
#include <rpgq_simulator/framework/objects/base_object.h>
#include <rpgq_simulator/framework/sensors/base_sensor.h>

namespace RPGQ
{
    namespace Simulator
    {
        class CompositeObject : public BaseObject
        {
        public:
            CompositeObject(ObjectID id, const Node* prevSimNode, USecs maxSimUSecsInterval):
                BaseObject(id, prevSimNode,  COMPOSITE, maxSimUSecsInterval)
            {};
            virtual ~CompositeObject() {};

            // simulate directly controlled object
            void RunSimulation(USecs dt);

            // add children, sensors, ...
            void AddChild(std::shared_ptr<BaseObject> child) {children_.emplace(child->GetID(), child);};
            void AddSensor(std::shared_ptr<BaseSensor> sensor) {sensors_.emplace(sensor->GetID(), sensor);};

            // public get functions
            std::unordered_map<ObjectID, std::shared_ptr<BaseObject>> const GetChildren(void) {return children_;};

        protected:
            // children, sensors, ...
            std::unordered_map<ObjectID, std::shared_ptr<BaseObject>> children_;
            std::unordered_map<SensorID, std::shared_ptr<BaseSensor>> sensors_;

            // (user defined) mediators
            virtual void ChildrenMediator(void) {};
            virtual void SensorMediator(void) {};
        };

    } // namespace Simulator
} // namespace RPGQ