#pragma once

// standard libary
#include <string>

namespace RPGQ
{
    namespace Simulator
    {
        // simulation tree
        struct Node
        {
            std::string id;
            const struct Node* prev;
        };

        inline std::string GetSimTreePath(const Node& simNode)
        {
            std::string simTreePath(simNode.id);
            const struct Node* p = simNode.prev;
            while (p)
            {
                simTreePath = p->id + "/" + simTreePath;  
                p = p->prev;
            }

            return simTreePath;
        };

        // objects
        typedef std::string ObjectID;

        namespace ID
        {
            namespace Object
            {
                const ObjectID Root = "rpgq_simulator";
                const ObjectID Optitrack = "optitrack_composite";

                const ObjectID Quadrotor = "quadrotor";
                const ObjectID QuadLidar = "quadrotor_lidar";

                const ObjectID QuadrotorEventCamera = "quadrotor_event_camera";
                const ObjectID QuadrotorStereoEventCamera = "quadrotor_stereo_event_camera";

                const ObjectID QuadrotorRGBCamera = "quadrotor_rgb_camera";
                const ObjectID QuadrotorStereoRGBCamera = "quadrotor_stereo_rgb_camera";


                const ObjectID SoccerBall = "soccer_ball";
                const ObjectID UnityGate = "unity_gate";
                const ObjectID UnitySoccerBall = "soccer_ball";
            }
        }

        // Unity objects
        typedef std::string PrefabID;

        namespace ID
        {
            namespace Object
            {
                const PrefabID RPGGate = "rpg_gate";
                const PrefabID SpotLight = "spot_light";
                const PrefabID Camera = "HD_camera";
            }
        }

        // onboard code
        typedef std::string OnboardCodeID;

        namespace ID
        {
            namespace OnboardCode
            {
                const OnboardCodeID OnboardLogic = "onboard_logic";
                const OnboardCodeID CommandBridgeClient = "command_bridge_client";
            }
        }

        // actuators
        typedef std::string ActuatorID;

        namespace ID
        {
            namespace Actuator
            {
                const ActuatorID Motor1 = "motor1";
                const ActuatorID Motor2 = "motor2";
                const ActuatorID Motor3 = "motor3";
                const ActuatorID Motor4 = "motor4";
            }
        }

        // dynamics
        typedef std::string DynamicsID;

        namespace ID
        {
            namespace Dynamics
            {
                const DynamicsID Quadrotor = "quadrotor";
                const DynamicsID SoccerBall = "soccer_ball";
            }
        }

        // sensors
        typedef std::string SensorID;

        namespace ID
        {
            namespace Sensor
            {
                const SensorID IMU = "imu";
                const SensorID Lidar = "lidar";
                const SensorID RGBCamera = "rgb_camera";
                const SensorID EventCamera = "event_camera";
                const SensorID Optitrack = "optitrack";
            }
        }

    } // namespace Simulator
} // namespace RPGQ