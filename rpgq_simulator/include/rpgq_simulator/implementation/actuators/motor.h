#pragma once

// rpgq rpgq_common
#include <rpgq_common/parameters/parameters.h>

// rpgq rpgq_simulator
#include <rpgq_simulator/framework/actuators/base_actuator.h>

namespace RPGQ
    {
    namespace Simulator
    {
        class Motor : public BaseActuator
        {
        public:
            // parameters
            enum
            {
                tau = 0, // [s]
                omegaMin = 1, // [rad/s]
                omegaMax = 2, // [rad/s]
                NUM_PARAMS
            };

            // constructor & destructor
            Motor(ActuatorID id, const Node* prevSimNode, USecs maxSimUSecsInterval, Parameters<NUM_PARAMS> params = Parameters<NUM_PARAMS>("simulator/actuators/motor"));
            ~Motor() {};

            // simulate actuator
            void RunSimulation(USecs dt);

            // public get functions
            double GetOmega(void) {return omega_;};

            // public set functions
            void SetOmegaDes(double omegaDes_);

        private:
            // system dynamics
            double omega_, omegaDes_;

            // motor parameters
            double tauUSecsInv_;
            double omegaMin_, omegaMax_;
        };

    } // namespace Simulator
} // namespace RPGQ