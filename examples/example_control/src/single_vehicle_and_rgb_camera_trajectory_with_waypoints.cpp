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

// message
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

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

  //initialize position, orientation and scale of quad
  std::shared_ptr<Simulator::QuadrotorVehicle> quad = quadRGB->GetQuad();
  Eigen::Vector3d init_position{-10.0, 0.0, 1.0};
  Eigen::Quaterniond init_orientation(std::cos(0.5*M_PI_2),0.0,0.0,std::sin(0.5*M_PI_2));
  quad->SetPos(init_position);
  quad->SetQuat(init_orientation);
  quad->SetSize(Eigen::Vector3d(1, 1, 1));

  // add the quadrotor with RGB camera to Optitrack
  // necessary to enable different filters for the camera
  sim.AddObjectToOptitrack(quadRGB);

  // add the quadrotor with RGB camera to Unity
  sim.AddObjectToUnity(quadRGB);

  // set up multi-purpose timer
  std::shared_ptr<ExtTimer> timer;
  timer.reset(new ExtTimer(sim.GetSimTimer()));

  // set up controller
  LowLevelOffboardController ctrl(quadID, timer);
  ctrl.SetCommandLevel(LowLevelOffboardController::CommandLevel::POS_CMD);
  ctrl.SetPosDes(init_position);
  ctrl.SetYawDes(0.0);

  // trajectory initialization
  std::cout << "start creating trajectory" << std::endl;
  Eigen::VectorXd segment_times(4);
  segment_times << 10, 10, 10, 10;

  // define start state
  quadrotor_common::TrajectoryPoint start_state;
  start_state.position = init_position;
  start_state.orientation = init_orientation;
  Eigen::Vector3d zero_vec{0, 0, 0};
  start_state.velocity = zero_vec;
  start_state.acceleration = zero_vec;
  start_state.jerk = zero_vec;
  start_state.snap = zero_vec;
  start_state.bodyrates = zero_vec;
  start_state.angular_acceleration = zero_vec;
  start_state.angular_jerk = zero_vec;
  start_state.angular_snap = zero_vec;
  start_state.heading = 0.0;
  start_state.heading_rate = 0.0;
  start_state.heading_acceleration = 0.0;

  // define end state
  quadrotor_common::TrajectoryPoint end_state;
  Eigen::Vector3d end_position(0, -25, 10);
  Eigen::Quaterniond end_orientation = init_orientation;
  end_state.position = end_position;
  end_state.orientation = end_orientation;
  end_state.velocity = zero_vec;
  end_state.acceleration = zero_vec;
  end_state.jerk = zero_vec;
  end_state.snap = zero_vec;
  end_state.bodyrates = zero_vec;
  end_state.angular_acceleration = zero_vec;
  end_state.angular_jerk = zero_vec;
  end_state.angular_snap = zero_vec;
  end_state.heading = 0.0;
  end_state.heading_rate = 0.0;
  end_state.heading_acceleration = 0.0;

  // define waypoints
  std::vector<Eigen::Vector3d> way_points;
  way_points.push_back(init_position);
  way_points.push_back(Eigen::Vector3d(0, 30, 7));
  way_points.push_back(Eigen::Vector3d(10, 0, 5)); 
  way_points.push_back(end_position);


  // minimization weights for each waypoint
  Eigen::VectorXd minimization_weights(4);
  minimization_weights << 1.0, 1.0, 1.0, 1.0;

  // trajectory settings calculate the minimum polynomial order for the given
  // waypoints
  polynomial_trajectories::PolynomialTrajectorySettings trajectory_settings = 
  polynomial_trajectories::PolynomialTrajectorySettings(way_points, minimization_weights, 5, 4);
  
  // generate the trajectory
  polynomial_trajectories::PolynomialTrajectory trajectory = 
  polynomial_trajectories::minimum_snap_trajectories::generateMinimumSnapRingTrajectory(
    segment_times, trajectory_settings, 50.0, 4*850.0, 10.0);

  std::cout << "trajectory calculated" << std::endl;

  // add gate to the simulator
  std::string gate_prefab_name = "rpg_gate";
  std::vector<Eigen::Quaterniond> way_points_orientation;
  way_points_orientation.push_back(Eigen::Quaterniond(std::cos(0),0.0,0.0,std::sin(0)));
  way_points_orientation.push_back(Eigen::Quaterniond(std::cos(0.5*M_PI_2),0.0,0.0,std::sin(0.5*M_PI_2)));
  way_points_orientation.push_back(Eigen::Quaterniond(std::cos(0),0.0,0.0,std::sin(0)));
  way_points_orientation.push_back(Eigen::Quaterniond(std::cos(0.5*M_PI_2),0.0,0.0,std::sin(0.5*M_PI_2)));
  

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

    // calculate the desired pose at time t for controller
    double elapsed_time = sim.ElapsedSeconds();
    double startup_time = 2.0;
    double end_time = trajectory.T.toSec() + startup_time;
    if (elapsed_time > startup_time)
    {
      quadrotor_common::TrajectoryPoint desired_pose = polynomial_trajectories::getPointFromTrajectory(trajectory, ros::Duration(elapsed_time-2.0));
      quad->SetPos(desired_pose.position);
      quad->SetQuat(desired_pose.orientation);
      // ctrl.SetPosDes(desired_pose.position);
      // ctrl.SetVelDes(desired_pose.velocity);
      // ctrl.SetAccDes(desired_pose.acceleration);
      // // heading is not implemented in getPointFromTrajectory, is default = 0.0
      // double yaw = std::atan2(desired_pose.velocity(1),desired_pose.velocity(0));
      // ctrl.SetYawDes(-yaw + M_PI_2);
      // ctrl.SetOmegaDes(desired_pose.bodyrates);
    }

    //run controller
    // rpgq_msgs::CommandSet cmdSet;
    // if (elapsed_time < startup_time)
    // {
    //   rpgq_msgs::AddCommandToCommandSet(rpgq_msgs::CreateArmCommand(quadID), cmdSet);
    // }
    // else
    // {
    //   RPGQ::State6DoF state_ground_truth;
    //   state_ground_truth.pos = quad->GetPos();
    //   state_ground_truth.vel = quad->GetVel();
    //   state_ground_truth.quat = quad->GetQuat();
    //   state_ground_truth.omega = quad->GetOmega();
    //   ctrl.Run(state_ground_truth);
    //   rpgq_msgs::AddCommandToCommandSet(ctrl.GetCommand(), cmdSet);
    // }

    // sim.SetCommandSet(cmdSet);

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

