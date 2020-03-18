#pragma once

// ros
#include <geometry_msgs/PoseStamped.h>

// rpgq rpgq_common
#include <rpgq_common/parameters/parameters.h>

// rpgq rpgq_simulator
#include <rpgq_simulator/framework/objects/physical_object.h>
#include <rpgq_simulator/framework/sensors/base_sensor.h>

// standard library
#include <deque>
#include <random>

namespace RPGQ
{
    namespace Simulator
    {
        class Optitrack : public BaseSensor
        {
        public:
            // parameters
            enum
            {
                updateFrequency = 0, // [Hz]
                xyzNoiseSD = 1, // [m]
                rotNoiseSD = 2, // [rad]
                latency = 3, // [s]
                NUM_PARAMS
            };

            // constructor & destructor
            Optitrack(SensorID id, const Node* prevSimNode, Parameters<NUM_PARAMS> params = Parameters<NUM_PARAMS>("simulator/sensors/optitrack"));
            ~Optitrack() {};

            // add object to sensor
            void AddObject(std::shared_ptr<PhysicalObject> physicalObject);

        private:
            // simulate sensor
            void RunSimulation_(void);
            USecs UpdateSamplingInterval(void) {};

            // publishers
            std::unordered_map<ObjectID, ros::Publisher> pubs_;
            std::unordered_map<ObjectID, std::deque<geometry_msgs::PoseStamped>> msgDeques_;

            // general Optitrack variables
            std::default_random_engine gen_;
            std::unordered_map<ObjectID, std::shared_ptr<PhysicalObject>> objects_;
            uint32_t frameCounter_;

            // sensor parameters
            Parameters<Optitrack::NUM_PARAMS> params_;
            std::normal_distribution<double> posNoise_, rotNoise_, auxRotNoise_;
            ros::Duration latency_;
        };

    } // namespace Simulator
} // namespace RPGQ