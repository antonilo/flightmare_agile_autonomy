#pragma once

// rpgq rpgq_simulator
#include <rpgq_simulator/framework/objects/physical_object.h>

namespace RPGQ
{
    namespace Simulator
    {
        class IndirectlyControlledObject : public PhysicalObject
        {
        public:
            IndirectlyControlledObject(ObjectID id, const Node* prevSimNode, USecs maxSimUSecsInterval):
                PhysicalObject(id, prevSimNode, INDIRECTLY_CONTROLLED, maxSimUSecsInterval)
            {};
            virtual ~IndirectlyControlledObject() {};

            // simulate directly controlled object
            void RunSimulation(USecs dt);
        };

    } // namespace Simulator
} // namespace RPGQ