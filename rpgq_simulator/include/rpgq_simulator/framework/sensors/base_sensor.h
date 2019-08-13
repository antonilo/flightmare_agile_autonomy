#pragma once

// rpqg rpgq_common
#include <rpgq_common/time/ext_timer.h>

// rpgq rpgq_simulator
#include <rpgq_simulator/simulator_types.h>

namespace RPGQ
{
    namespace Simulator
    {
        class BaseSensor
        {
        public:
            // constructor & destructor
            BaseSensor(SensorID id, const Node* prevSimNode,
              USecs maxSimUSecsInterval, bool adaptiveSampling = false):
                id_(id),
                simNode_(Node{id,prevSimNode}),
                timer_(ExtTimer(true, false)),
                maxSimUSecsInterval_(maxSimUSecsInterval),
                adaptiveSampling_(adaptiveSampling)
            {
                usecsNextSim_ = maxSimUSecsInterval_;

                int argc = 0;
                ros::init(argc, (char **) nullptr, id);
                pnh_ = ros::NodeHandle(ID::Object::Root);
            };
            virtual ~BaseSensor() {};

            // simulate sensor
            inline void RunSimulation(USecs dt)
            {
                // update time (before sensor is simulated)
                timer_.AdvanceUSeconds(dt);

                // simulate sensor
                if (usecsNextSim_ == dt)
                {
                    RunSimulation_();
                    if (adaptiveSampling_)
                    {
                        usecsNextSim_ = UpdateSamplingInterval();
                    }
                    else
                    {
                        usecsNextSim_ = maxSimUSecsInterval_;
                    }
                }
                else
                {
                    usecsNextSim_ -= dt;
                }
            }

            // public get functions
            const SensorID GetID(void) {return id_;};
            const Node * GetSimNode(void) {return &simNode_;};
            const USecs GetMaxSimInterval(void) {return maxSimUSecsInterval_;};
            USecs GetUSecsNextSim(void) {return usecsNextSim_;};

        protected:
            // general dynamics variables
            const SensorID id_;
            const Node simNode_;
            ExtTimer timer_;
            ros::NodeHandle pnh_;

            // simulate sensor
            virtual void RunSimulation_(void) = 0;
            virtual USecs UpdateSamplingInterval(void) = 0;

            // simulation timing variables
            const USecs maxSimUSecsInterval_;
            const bool adaptiveSampling_;
            USecs usecsNextSim_;
        };

    } // namespace Simulator
} // namespace RPGQ