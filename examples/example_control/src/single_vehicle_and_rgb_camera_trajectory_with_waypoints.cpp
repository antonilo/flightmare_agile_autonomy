// rpqg simulator
#include <rpgq_simulator/simulator.h>
#include <rpgq_simulator/implementation/objects/quadrotor_vehicle/quad_and_rgb_camera.h>
#include <rpgq_simulator/implementation/unity/unity_gate.h>

// rpgq components
#include <rpgq_components/controller/low_level_offboard_controller.h>
#include <rpgq_components/estimator/optitrack_estimator.h>

// standard library
#include <cmath>
#include <Eigen/Dense>

// trajectory
#include <quadrotor_common/trajectory_point.h>
#include <polynomial_trajectories/minimum_snap_trajectories.h>
#include <polynomial_trajectories/polynomial_trajectory.h>
#include <polynomial_trajectories/polynomial_trajectory_settings.h>
#include <polynomial_trajectories/polynomial_trajectories_common.h>

// messages
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#define CONTROL_UPDATE_RATE 50.0

using namespace RPGQ;

int main(int argc, char * argv[]) {
  // initialize ROS
  ros::init(argc, argv, "flightmare_example");
  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");

  // define the scene name
  FlightmareTypes::SceneID scene_id = FlightmareTypes::SCENE_WAREHOUSE;

  // quad ID can be any real number between
  // 0 ~ 25, each ID corresponding to a unique
  // quad name
  QuadrotorID quadID = 0;
  std::string quadName = QuadrotorName(quadID);

  // create simulator
  Simulator::Simulator sim;
  //
  sim.SetFlightmare(true);

  // create a quadrotor with a RGB Camera on it.
  std::shared_ptr<Simulator::QuadRGBCamera> quadRGB = 
    std::make_shared<Simulator::QuadRGBCamera>(quadName, nullptr, 1000000);

  // create RGB camera and enable different filters
  std::shared_ptr<Simulator::RGBCamera> rgb = quadRGB->GetRGBCamera();
  rgb->EnableOpticalFlow(true);
  rgb->EnableDepth(true);
  rgb->EnableObjectSegment(true);
  rgb->EnableCategorySegment(true);
  rgb->SetWidth(320);
  rgb->SetHeight(240);

  // initialize position, orientation and scale of quad
  std::shared_ptr<Simulator::QuadrotorVehicle> quad = quadRGB->GetQuad();
  // spawn point on the floor and behind the first waypoint of the ring trajectory
  Eigen::Vector3d spawn_position{0.0, -20.0, 0.0};
  Eigen::Quaterniond spawn_orientation(std::cos(0.5*M_PI_2),0.0,0.0,std::sin(0.5*M_PI_2));
  quad->SetPos(spawn_position);
  quad->SetQuat(spawn_orientation);
  quad->SetSize(Eigen::Vector3d(1, 1, 1));

  // add the quadrotor with RGB camera to Optitrack
  // if not added, post-processing images are not received
  sim.AddObjectToOptitrack(quadRGB);

  // add the quadrotor with RGB camera to Unity
  sim.AddObjectToUnity(quadRGB);

  // set up multi-purpose timer
  std::shared_ptr<ExtTimer> timer;
  timer.reset(new ExtTimer(sim.GetSimTimer()));

  // set up controller
  LowLevelOffboardController ctrl(quadID, timer);
  ctrl.SetCommandLevel(LowLevelOffboardController::CommandLevel::POS_CMD);
  ctrl.SetPosDes(spawn_position);
  ctrl.SetYawDes(0.0);

  // ring trajectory initialization
  Eigen::VectorXd segment_times(8);
  segment_times << 10, 10, 10, 10, 10, 10, 10, 10;

  // define waypoints
  std::vector<Eigen::Vector3d> way_points;
  way_points.push_back(Eigen::Vector3d(-10.0, 0.0, 4.0)); // initiale position
  way_points.push_back(Eigen::Vector3d(0, 30, 7));
  way_points.push_back(Eigen::Vector3d(10, 0, 5)); 
  way_points.push_back(Eigen::Vector3d(0, -25, 10));
  way_points.push_back(Eigen::Vector3d(-5, 0, 7));
  way_points.push_back(Eigen::Vector3d(0, 15, 5));
  way_points.push_back(Eigen::Vector3d(5, 0, 7));
  way_points.push_back(Eigen::Vector3d(0, -10, 5));

  // minimization weights for each waypoint
  Eigen::VectorXd minimization_weights(8);
  minimization_weights << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  // trajectory settings calculate the minimum polynomial order for the given
  // waypoints
  polynomial_trajectories::PolynomialTrajectorySettings trajectory_settings = 
  polynomial_trajectories::PolynomialTrajectorySettings(way_points, minimization_weights, 5, 4);
  
  // generate the ring trajectory
  polynomial_trajectories::PolynomialTrajectory trajectory = 
  polynomial_trajectories::minimum_snap_trajectories::generateMinimumSnapRingTrajectoryWithSegmentRefinement(
    segment_times, trajectory_settings, 20.0, 20.0, 6.0);

  
  // start trajectory initialization
  // This trajectory goes from the ground to the initial state of the ring trajectory

  // initiale state of the ring trajectory
  quadrotor_common::TrajectoryPoint ring_init_state = polynomial_trajectories::getPointFromTrajectory(trajectory, ros::Duration(0));

  // define spawn state
  quadrotor_common::TrajectoryPoint spawn_state;
  spawn_state.position = spawn_position;
  spawn_state.orientation = spawn_orientation;
  Eigen::Vector3d zero_vec{0, 0, 0};
  spawn_state.velocity = zero_vec;
  spawn_state.acceleration = zero_vec;
  spawn_state.jerk = zero_vec;
  spawn_state.snap = zero_vec;
  spawn_state.bodyrates = zero_vec;
  spawn_state.angular_acceleration = zero_vec;
  spawn_state.angular_jerk = zero_vec;
  spawn_state.angular_snap = zero_vec;
  spawn_state.heading = 0.0;
  spawn_state.heading_rate = 0.0;
  spawn_state.heading_acceleration = 0.0;

  Eigen::VectorXd start_segment(1);
  start_segment << 10.0;

  Eigen::VectorXd start_min_weight(1);
  start_min_weight << 1.0;

  std::vector<Eigen::Vector3d> start_way_points;

  polynomial_trajectories::PolynomialTrajectorySettings start_trajectory_settings = 
  polynomial_trajectories::PolynomialTrajectorySettings(start_way_points, start_min_weight, 5, 4);

  polynomial_trajectories::PolynomialTrajectory startup_trajectory = 
  polynomial_trajectories::minimum_snap_trajectories::generateMinimumSnapTrajectoryWithSegmentRefinement(
    start_segment, spawn_state, ring_init_state, start_trajectory_settings, 20.0, 20.0, 6.0);

  // add gate to the simulator
  // name of the Unity prefab in Assets/Resources for Rescources.Load()
  std::string gate_prefab_name = "rpg_gate";

  // the orientation of the gates at the waypoints defined in the ring trajectory
  std::vector<Eigen::Quaterniond> way_points_orientation;
  way_points_orientation.push_back(Eigen::Quaterniond(std::cos(0),0.0,0.0,std::sin(0)));
  way_points_orientation.push_back(Eigen::Quaterniond(std::cos(0.5*M_PI_2),0.0,0.0,std::sin(0.5*M_PI_2)));
  way_points_orientation.push_back(Eigen::Quaterniond(std::cos(0),0.0,0.0,std::sin(0)));
  way_points_orientation.push_back(Eigen::Quaterniond(std::cos(0.5*M_PI_2),0.0,0.0,std::sin(0.5*M_PI_2)));
  way_points_orientation.push_back(Eigen::Quaterniond(std::cos(0),0.0,0.0,std::sin(0)));
  way_points_orientation.push_back(Eigen::Quaterniond(std::cos(0.5*M_PI_2),0.0,0.0,std::sin(0.5*M_PI_2)));
  way_points_orientation.push_back(Eigen::Quaterniond(std::cos(0),0.0,0.0,std::sin(0)));
  way_points_orientation.push_back(Eigen::Quaterniond(std::cos(0.5*M_PI_2),0.0,0.0,std::sin(0.5*M_PI_2)));
  
  // add the gates to unity
  for (int i = 0; i<way_points.size(); i++)
  {
    std::string gate_name = "gate_"+std::to_string(i)+"_"+std::to_string(1);
    std::shared_ptr<Simulator::UnityGate> gate_i =
      std::make_shared<Simulator::UnityGate>(gate_name, gate_prefab_name);
    gate_i->SetPosition(way_points.at(i));
    gate_i->SetRotation(way_points_orientation.at(i));
    // add objects to unity for simulation and visulization.
    sim.AddObjectToUnity(gate_i);
  }

  // define ROS Rate
  ros::Rate loop_rate(10);

  // Publisher of pose and trajectory path of quadrotor
  ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>("quadrotor_position", 10);
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("quadrotor_path", 1);

  // Path message with the poses in ms accuracy
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = "map";
  
  std::vector<geometry_msgs::PoseStamped> poses;

  for (int i = 0; i<=ceil((trajectory.T).toSec())*1000; i++){
        geometry_msgs::PoseStamped msg_pose;
        geometry_msgs::Pose pose;
        Eigen::Isometry3d new_pose;

        quadrotor_common::TrajectoryPoint path_pose_t = polynomial_trajectories::getPointFromTrajectory(trajectory, ros::Duration(i/1000.0));
        new_pose.translation() = path_pose_t.position;
        tf::poseEigenToMsg(new_pose, pose);
        tf::quaternionEigenToMsg(path_pose_t.orientation, pose.orientation);
        msg_pose.header.stamp = ros::Time::now();
        msg_pose.header.frame_id = "map"; //string map or world
        // msg_pose.header.seq = 1; //not important for visualization
        msg_pose.pose = pose; 
        poses.insert(poses.begin() + i, msg_pose);
  }
  path_msg.poses = poses;
  path_pub.publish(path_msg);
  
  // main loop
  Timer loopTimer;

  while (ros::ok())
  {

    // enable visualization and add objects
    if (!sim.FlightmareIsReady())
    {
      sim.ConnectFlightmare(scene_id);
    }
    // start measuring time
    loopTimer.Reset();

    // run ros callbacks
    ros::spinOnce();

    // startup times and elapsed time in seconds
    double startup_time_simulator = 2.0;
    double startup_time_trajectory = startup_trajectory.T.toSec() + startup_time_simulator;
    double elapsed_time = sim.ElapsedSeconds();

    // ring trajectory
    if (elapsed_time > startup_time_trajectory)
    {
      quadrotor_common::TrajectoryPoint desired_pose = polynomial_trajectories::getPointFromTrajectory(trajectory, ros::Duration(elapsed_time-startup_time_trajectory));
      path_msg.header.stamp = ros::Time::now();
      path_pub.publish(path_msg);

      ctrl.SetPosDes(desired_pose.position);
      ctrl.SetVelDes(desired_pose.velocity);
      ctrl.SetAccDes(desired_pose.acceleration);
      // heading is not implemented in getPointFromTrajectory, is default = 0.0
      double yaw = std::atan2(desired_pose.velocity(1),desired_pose.velocity(0));
      ctrl.SetYawDes(-yaw + M_PI_2);
      ctrl.SetOmegaDes(desired_pose.bodyrates);
    }
    // start trajectory
    else if (elapsed_time > startup_time_simulator){
      quadrotor_common::TrajectoryPoint desired_pose = polynomial_trajectories::getPointFromTrajectory(startup_trajectory, ros::Duration(elapsed_time-startup_time_simulator));
      ctrl.SetPosDes(desired_pose.position);
      ctrl.SetVelDes(desired_pose.velocity);
      ctrl.SetAccDes(desired_pose.acceleration);
      // heading is not implemented in getPointFromTrajectory, is default = 0.0
      double yaw = std::atan2(desired_pose.velocity(1),desired_pose.velocity(0));
      ctrl.SetYawDes(-yaw + M_PI_2);
      ctrl.SetOmegaDes(desired_pose.bodyrates);
    }
    else{
      // simulator not ready yet
    }

    // Pose message
    geometry_msgs::PoseStamped msg_pose;
    geometry_msgs::Pose pose;
    Eigen::Isometry3d new_pose;
    new_pose.translation() = quad->GetPos();
    tf::poseEigenToMsg(new_pose, pose);
    tf::quaternionEigenToMsg(quad->GetQuat(), pose.orientation);
    msg_pose.header.stamp = ros::Time::now();
    msg_pose.header.frame_id = "map"; //string map or world
    // msg_pose.header.seq = 1; //not important for visualization
    msg_pose.pose = pose; 
    pos_pub.publish(msg_pose);

    // run controller
    rpgq_msgs::CommandSet cmdSet;
    if (elapsed_time < startup_time_simulator)
    {
      // idle
      rpgq_msgs::AddCommandToCommandSet(rpgq_msgs::CreateArmCommand(quadID), cmdSet);
    }
    else
    {
      // current state of quadrotor 
      RPGQ::State6DoF state_ground_truth;
      state_ground_truth.pos = quad->GetPos();
      state_ground_truth.vel = quad->GetVel();
      state_ground_truth.quat = quad->GetQuat();
      state_ground_truth.omega = quad->GetOmega();
      ctrl.Run(state_ground_truth);
      rpgq_msgs::AddCommandToCommandSet(ctrl.GetCommand(), cmdSet);
    }

    sim.SetCommandSet(cmdSet);

    // run simulation
    sim.Run(1.0/CONTROL_UPDATE_RATE);

    // stop measuring time, sleep accordingly
    double secsToSleep = 1.0/CONTROL_UPDATE_RATE - loopTimer.ElapsedSeconds();
    if (secsToSleep > 0.0)
    {
      usleep(secsToSleep*1e6);
    }
  }
  return 0;
}
