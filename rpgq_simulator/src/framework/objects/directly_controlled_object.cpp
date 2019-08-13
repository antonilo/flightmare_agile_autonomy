#include <rpgq_simulator/framework/objects/directly_controlled_object.h>

// standard library
#include <algorithm>

namespace RPGQ
{
    namespace Simulator
    {
        // simulate directly controlled object
        void DirectlyControlledObject::RunSimulation(USecs dt)
        {
            // simulation loop
            USecs remainingUSecsToSimulate = dt;
            while (remainingUSecsToSimulate > 0)
            {
                // compute simulation step size
                USecs simUSecs = std::min(remainingUSecsToSimulate, minUSecsNextSim_);
                minUSecsNextSim_ = maxSimUSecsInterval_;

                // simulate onboard code
                OnboardCodeMediator();
                for (const auto & onboardCode : onboardCode_)
                {
                    onboardCode.second->RunSimulation(simUSecs);
                    minUSecsNextSim_ = std::min(minUSecsNextSim_, onboardCode.second->GetUSecsNextSim());
                }

                // simulate actuators
                ActuatorMediator();
                for (const auto & actuator : actuators_)
                {
                    actuator.second->RunSimulation(simUSecs);
                }

                // simulate dynamics
                DynamicsMediator();
                for (const auto & dynamics : dynamics_)
                {
                    dynamics.second->RunSimulation(simUSecs);
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
