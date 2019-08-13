#pragma once

// rpgq rpgq_common
#include <rpgq_common/time/timer.h>

// rpgq rpgq_simulator
#include <rpgq_simulator/simulator_types.h>

namespace RPGQ
{
    namespace Simulator
    {
        class BaseActuator
        {
        public:
            // constructor & destructor
            BaseActuator(ActuatorID id, const Node* prevSimNode, USecs maxSimUSecsInterval):
                id_(id),
                simNode_(Node{id,prevSimNode}),
                maxSimUSecsInterval_(maxSimUSecsInterval)
            {};
            virtual ~BaseActuator() {};

            // simulate actuator
            virtual void RunSimulation(USecs dt) = 0;

            // public get functions
            const ActuatorID GetID(void) {return id_;};
            const Node * GetSimNode(void) {return &simNode_;};
            const USecs GetMaxSimInterval(void) {return maxSimUSecsInterval_;};

        protected:
            // general dynamics variables
            const ActuatorID id_;
            const Node simNode_;

            // simulation timing variables
            const USecs maxSimUSecsInterval_;
        };

    } //  namespace Simulator
} // namespace RPGQ