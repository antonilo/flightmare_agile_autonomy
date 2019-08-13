#include <rpgq_simulator/framework/objects/composite_object.h>

// standard library
#include <algorithm>

namespace RPGQ
{
    namespace Simulator
    {
        // simulate composite object
        void CompositeObject::RunSimulation(USecs dt)
        {
            // simulation loop
            USecs remainingUSecsToSimulate = dt;
            while (remainingUSecsToSimulate > 0)
            {
                // compute simulation step size
                USecs simUSecs = std::min(remainingUSecsToSimulate, minUSecsNextSim_);
                minUSecsNextSim_ = maxSimUSecsInterval_;

                // simulate children
                ChildrenMediator();
                for (const auto & child : children_)
                {
                    child.second->RunSimulation(simUSecs);
                }

                // simulate sensors
                SensorMediator();
                for (const auto & sensor : sensors_)
                {
                    sensor.second->RunSimulation(simUSecs);
                    minUSecsNextSim_ = std::min(minUSecsNextSim_, sensor.second->GetUSecsNextSim());
                }

                // compute output data
                ComputeOutputContinuously();

                // push time forward
                timer_.AdvanceUSeconds(simUSecs);
                remainingUSecsToSimulate -= simUSecs;
            }

            // compute output data
            ComputeOutputOnce();
        }

    } // namespace Simulator
} // namespace RPGQ