#include <rpgq_simulator/implementation/objects/soccer_ball/soccer_ball.h>

// ros
#include <geometry_msgs/PoseStamped.h>

namespace RPGQ
{
    namespace Simulator
    {
        // constructor
        SoccerBall::SoccerBall(ObjectID id, const Node* prevSimNode, USecs maxSimUSecsInterval):
            IndirectlyControlledObject(id, prevSimNode, maxSimUSecsInterval)
        {
            // dynamics
            dynamicsPtr_ = std::make_shared<SoccerBallDynamics>(ID::Dynamics::SoccerBall, &simNode_, 50000); // maximum simulation step size 20'000us -> 50ms
            AddDynamics(dynamicsPtr_);  

            // quadrotor output
            posePub_ = pnh_.advertise<geometry_msgs::PoseStamped>(GetSimTreePath(simNode_) + "/pose_stamped", 1);
        }


        // object output
        void SoccerBall::ComputeOutputOnce(void)
        {
            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.stamp.fromNSec(1000*timer_.ElapsedUSeconds());
            pose_msg.header.frame_id = "map";

            Eigen::Vector3d pos = GetPos();
            pose_msg.pose.position.x = pos(0);
            pose_msg.pose.position.y = pos(1);
            pose_msg.pose.position.z = pos(2);

            Eigen::Quaterniond quat = GetQuat();
            pose_msg.pose.orientation.w = quat.w();
            pose_msg.pose.orientation.x = quat.x();
            pose_msg.pose.orientation.y = quat.y();
            pose_msg.pose.orientation.z = quat.z();

            posePub_.publish(pose_msg);
        }

    } // namespace Simulator
} // namespace RPGQ