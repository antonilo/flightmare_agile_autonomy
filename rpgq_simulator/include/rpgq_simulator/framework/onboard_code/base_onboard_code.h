#pragma once

// rpgq rpgq_common
#include <rpgq_common/time/ext_timer.h>

// rpgq rpgq_simulator
#include <rpgq_simulator/simulator_types.h>

// ros
#include <ros/ros.h>

namespace RPGQ
{
    namespace Simulator
    {
        class BaseOnboardCode
        {
        public:
            // constructor & destructor
            BaseOnboardCode(OnboardCodeID id, const Node* prevSimNode, USecs maxSimUSecsInterval):
                id_(id),
                simNode_(Node{id,prevSimNode}),
                timer_(ExtTimer(true, false)),
                maxSimUSecsInterval_(maxSimUSecsInterval)
            {
                usecsNextSim_ = maxSimUSecsInterval_;

                int argc = 0;
                ros::init(argc, (char **) nullptr, id);
                pnh_ = ros::NodeHandle(ID::Object::Root);
            };
            virtual ~BaseOnboardCode() {};

            // simulate onboard code
            inline void RunSimulation(USecs dt)
            {
                // simulate onboard code
                if (usecsNextSim_ == dt)
                {
                    RunSimulation_(dt);
                    usecsNextSim_ = maxSimUSecsInterval_;
                }
                else
                {
                usecsNextSim_ -= dt;   
                }

                // update time (after onboard code is executed)
                timer_.AdvanceUSeconds(dt);
            };

            // public get functions
            const OnboardCodeID GetID(void) {return id_;};
            const Node * GetSimNode(void) {return &simNode_;};
            const USecs GetMaxSimInterval(void) {return maxSimUSecsInterval_;};
            USecs GetUSecsNextSim(void) {return usecsNextSim_;};

        protected:
            // general onboard code variables
            const OnboardCodeID id_;
            const Node simNode_;
            ExtTimer timer_;
            ros::NodeHandle pnh_;

            // simulate onboard code
            virtual void RunSimulation_(USecs dt) = 0;

            // simulation timing variables
            const USecs maxSimUSecsInterval_;
            USecs usecsNextSim_;

        };

    } // namespace Simulator
} // namespace RPGQ