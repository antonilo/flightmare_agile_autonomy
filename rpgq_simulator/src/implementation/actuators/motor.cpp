#include <rpgq_simulator/implementation/actuators/motor.h>

// rpgq rpgq_common
#include <rpgq_common/rpgq.h>

// standard library
#include <cmath>

namespace RPGQ
{
    namespace Simulator
    {
        // constructor
        Motor::Motor(ActuatorID id, const Node* prevSimNode, USecs maxSimUSecsInterval, Parameters<NUM_PARAMS> params):
            BaseActuator(id, prevSimNode, maxSimUSecsInterval)
        {            
            // system dynamics
            omega_ = 0.0;
            omegaDes_ = 0.0;

            // motor parameters
            tauUSecsInv_ = SECS_TO_USECS(1.0/params.GetDouble(Motor::tau));
            omegaMin_ = params.GetDouble(Motor::omegaMin);
            omegaMax_ = params.GetDouble(Motor::omegaMax);
        };

        // rpgq_simulator actuator
        void Motor::RunSimulation(USecs dt)
        {
            // simulate motor as first-order system
            double c = std::exp(-(double) dt*tauUSecsInv_);
            omega_ = c*omega_ + (1.0 - c)*omegaDes_;

            // stop motor if motor is slow enough and commanded to stop
            if (omegaDes_ == 0 && omega_ < 1.05*omegaMin_)
            {
                omega_ = 0.0;
            }
        };


        // public set functions
        void Motor::SetOmegaDes(double omegaDes)
        {
            if (omegaDes <= 0.0)
            {
                omegaDes_ = 0.0;
            }
            else
            {
                omegaDes_ = CONSTRAIN(omegaDes, omegaMin_, omegaMax_);
            }
        }

    } // namespace Simulator
} // namespace RPGQ