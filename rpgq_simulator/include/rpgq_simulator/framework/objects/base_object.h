#pragma once

// ros
#include <ros/ros.h>

// rpgq rpgq_common
#include <rpgq_common/time/ext_timer.h>

// rpgq rpgq_simulator
#include <rpgq_simulator/simulator_types.h>

// standard library
#include <memory>
#include <unordered_map>

namespace RPGQ
{
    namespace Simulator
    {
        enum ObjectType
        {
            DIRECTLY_CONTROLLED = 0,
            INDIRECTLY_CONTROLLED = 1,
            COMPOSITE = 2
        };

        class BaseObject
        {
        public:
            // constructor & destructor
            BaseObject(ObjectID id, const Node* prevSimNode, ObjectType objType, USecs maxSimUSecsInterval):
                id_(id),
                simNode_(Node{id,prevSimNode}),
                objType_(objType),
                timer_(ExtTimer(true, false)),
                maxSimUSecsInterval_(maxSimUSecsInterval)
            {
                minUSecsNextSim_ = maxSimUSecsInterval;
                
                int argc = 0;
                ros::init(argc, (char **) nullptr, id);
                pnh_ = ros::NodeHandle(ID::Object::Root);
            };
            virtual ~BaseObject() {};

            // simulate object
            virtual void RunSimulation(USecs dt) = 0;

            // compute (user defined) object output
            virtual void ComputeOutputOnce(void) {};
            virtual void ComputeOutputContinuously(void) {};

            // public get functions
            const ObjectID GetID(void) {return id_;};
            void GetID(ObjectID &id ){ id = id_; };
            const ObjectType GetObjectType(void) {return objType_;};
            void GetObjectType(ObjectType & obj_type) { obj_type = objType_; };

            const Node * GetSimNode(void) {return &simNode_;};

        protected:
            // general physical object variables
            const ObjectID id_;
            const Node simNode_;
            const ObjectType objType_;
            ExtTimer timer_;
            ros::NodeHandle pnh_;

            // simulation timing variables
            const USecs maxSimUSecsInterval_;
            USecs minUSecsNextSim_;
        };

    } // namespace Simulator
} // namespace RPGQ
