#pragma once

// rpgq rpgq_common
#include <rpgq_common/time/timer.h>

// rpgq rpgq_simulator
#include <rpgq_simulator/simulator_types.h>

namespace RPGQ
{
    namespace Simulator
    {
        class BaseDynamics
        {
        public:
            // constructor & destructor
            BaseDynamics(DynamicsID id, const Node* prevSimNode, USecs maxSimUSecsInterval):
                id_(id),
                simNode_(Node{id,prevSimNode}),
                maxSimUSecsInterval_(maxSimUSecsInterval)
            {};
            virtual ~BaseDynamics() {};

            // simulate dynamics
            virtual void RunSimulation(USecs dt) = 0;

            // public get functions
            const DynamicsID GetID(void) {return id_;};
            const USecs GetMaxSimInterval(void) {return maxSimUSecsInterval_;};

        protected:
            // general dynamics variables
            const DynamicsID id_;
            const Node simNode_;

            // simulation timing variables
            const USecs maxSimUSecsInterval_;
        };

    } // namespace Simulator
} // namespace RPGQ