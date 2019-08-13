#pragma once

// rpgq rpgq_simulator
#include <rpgq_simulator/framework/objects/physical_object.h>
#include <rpgq_simulator/framework/actuators/base_actuator.h>
#include <rpgq_simulator/framework/onboard_code/base_onboard_code.h>

// rpgq rpgq_common
#include <rpgq_common/msgs/command_msg_helper.h>

namespace RPGQ
{
    namespace Simulator
    {
        class DirectlyControlledObject : public PhysicalObject
        {
        public:
            DirectlyControlledObject(ObjectID id, const Node* prevSimNode, USecs maxSimUSecsInterval):
                PhysicalObject(id, prevSimNode, DIRECTLY_CONTROLLED, maxSimUSecsInterval)
            {};
            virtual ~DirectlyControlledObject() {};

            // simulate directly controlled object
            void RunSimulation(USecs dt);

            // add onboard code, actuators...
            void AddOnboardCode(std::shared_ptr<BaseOnboardCode> onboardCode) {onboardCode_.emplace(onboardCode->GetID(), onboardCode);};
            void AddActuator(std::shared_ptr<BaseActuator> actuator) {actuators_.emplace(actuator->GetID(), actuator);};

            // public set functiosn
            virtual void SetCommandSet(const rpgq_msgs::CommandSet &cmdSet) {};

        protected:
            // onboard code, actuators ...
            std::unordered_map<OnboardCodeID, std::shared_ptr<BaseOnboardCode>> onboardCode_;
            std::unordered_map<ActuatorID, std::shared_ptr<BaseActuator>> actuators_;

            // (user defined) mediators
            virtual void OnboardCodeMediator(void) = 0;
            virtual void ActuatorMediator(void) = 0;
        };

    } // namespace Simulator
} // namespace RPGQ