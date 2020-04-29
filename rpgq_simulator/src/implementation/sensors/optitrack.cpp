#include <rpgq_simulator/implementation/sensors/optitrack.h>

// others
#include <eigen3/Eigen/Dense>

namespace RPGQ
{
    namespace Simulator
    {
        // constructor
        Optitrack::Optitrack(SensorID id, const Node* prevSimNode, Parameters<NUM_PARAMS> params):
            BaseSensor(id, prevSimNode, SECS_TO_USECS(1.0/params.GetDouble(Optitrack::updateFrequency)))
        {
            // sensor parameters
            params_ = params;
            posNoise_ = std::normal_distribution<double>(0.0, params_.GetDouble(Optitrack::xyzNoiseSD));
            rotNoise_ = std::normal_distribution<double>(0.0, params_.GetDouble(Optitrack::rotNoiseSD));
            auxRotNoise_ = std::normal_distribution<double>(0.0, 1.0);
            latency_ = ros::Duration(params_.GetDouble(Optitrack::latency));

            // general Optitrack variables
            frameCounter_ = 0;
        };

        // add object to sensor
        void Optitrack::AddObject(std::shared_ptr<PhysicalObject> physicalObject)
        {
            // store pointer to object and create new publisher and message queue
            objects_.emplace(physicalObject->GetID(), physicalObject);

            ros::Publisher pub = pnh_.advertise<geometry_msgs::PoseStamped>(GetSimTreePath(simNode_) + "/" + physicalObject->GetID(), 10);
            pubs_.emplace(physicalObject->GetID(), pub);
            msgDeques_.emplace(physicalObject->GetID(), std::deque<geometry_msgs::PoseStamped>());
        }


        // simulate sensor
        void Optitrack::RunSimulation_(void)
        {
            // generate header
            std_msgs::Header header;
            header.stamp.fromNSec(1000*timer_.ElapsedUSeconds());
            header.seq = frameCounter_++;

            for (const auto & obj : objects_)
            {
                ObjectID obj_id = obj.first;
                // generate new geometry message
                geometry_msgs::PoseStamped poseMsg;
                poseMsg.header = header; // all rpgq_msgs have the same header

                Eigen::Vector3d pos = obj.second->GetPos();
                poseMsg.pose.position.x = pos(0) + posNoise_(gen_);
                poseMsg.pose.position.y = pos(1) + posNoise_(gen_);;
                poseMsg.pose.position.z = pos(2) + posNoise_(gen_);;

                Eigen::Quaterniond quat = obj.second->GetQuat();
                Eigen::Vector3d k(auxRotNoise_(gen_), auxRotNoise_(gen_), auxRotNoise_(gen_));
                while (k.norm() < 1e-6)
                {
                    k(0) = auxRotNoise_(gen_);
                    k(1) = auxRotNoise_(gen_);
                    k(2) = auxRotNoise_(gen_);
                }
                k = k.normalized();
                quat = Eigen::Quaterniond(Eigen::AngleAxisd(rotNoise_(gen_), k))*quat;
                poseMsg.pose.orientation.w = quat.w();
                poseMsg.pose.orientation.x = quat.x();
                poseMsg.pose.orientation.y = quat.y();
                poseMsg.pose.orientation.z = quat.z();

                // add message to queue
                msgDeques_[obj_id].push_back(poseMsg);

                // publish messages if necessary
                while (msgDeques_[obj_id].size() > 0)
                {
                    if (msgDeques_[obj_id].front().header.stamp + latency_ <= header.stamp)
                    {
                        pubs_[obj_id].publish(msgDeques_[obj_id].front());
                        msgDeques_[obj_id].pop_front();
                    }
                    else
                    {
                        break;
                    }
                }
            }
        }

    } // namespace Simulator
} // namespace RPGQ