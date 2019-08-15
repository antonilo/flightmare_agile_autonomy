// rpqg simulator
#include <rpgq_simulator/simulator.h>
#include <rpgq_simulator/implementation/objects/quadrotor_vehicle/quad_and_rgb_camera.h>
#include <rpgq_simulator/implementation/unity/unity_gate.h>

// rpgq components
#include <rpgq_components/controller/low_level_offboard_controller.h>
#include <rpgq_components/estimator/optitrack_estimator.h>

// standard library
#include <cmath>

// others
#include <trajectory/trajectory.h>

#define CONTROL_UPDATE_RATE 50.0

using namespace RPGQ;

int main(int argc, char * argv[]) {
  // initialize ROS
  ros::init(argc, argv, "flightmare_example");
  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");

  // get parameters
  std::string quadName1 = "leonardo";
  std::string quadName2 = "michelangelo";

  // Hack
  QuadrotorID quadID1 = QuadrotorNameToID(quadName1);
  QuadrotorID quadID2 = QuadrotorNameToID(quadName2);

  std::string sceneName;
  if (!pnh.getParam("scene_name", sceneName))
  {
    ROS_ERROR("[%s] Could not determine scene name.", pnh.getNamespace().c_str());
    return -1;
  }

  // create simulator
  std::shared_ptr<Simulator::Simulator> sim = std::make_shared<Simulator::Simulator>();

  // create a quadrotor with a RGB Camera on it.
  std::shared_ptr<Simulator::QuadRGBCamera> quadRGB1 = std::make_shared<Simulator::QuadRGBCamera>(quadName1, nullptr, 1000000);
  std::shared_ptr<Simulator::QuadrotorVehicle> quad1 = quadRGB1->GetQuad();
  quad1->SetPos(Eigen::Vector3d(0.0, 0.0, 2.0));
  quad1->SetQuat(Eigen::Quaterniond(std::cos(0.5*M_PI_2),0.0,0.0,std::sin(0.5*M_PI_2)));
  quad1->SetSize(Eigen::Vector3d(0.3, 0.3, 0.3));

  std::shared_ptr<Simulator::QuadRGBCamera> quadRGB2 = std::make_shared<Simulator::QuadRGBCamera>(quadName2, nullptr, 1000000);
  std::shared_ptr<Simulator::QuadrotorVehicle> quad2 = quadRGB2->GetQuad();
  quad2->SetPos(Eigen::Vector3d(3.0, 0.0, 2.0));
  quad2->SetQuat(Eigen::Quaterniond(std::cos(0.5*M_PI_2),0.0,0.0,std::sin(0.5*M_PI_2)));
  quad2->SetSize(Eigen::Vector3d(0.3, 0.3, 0.3));

  //
  sim->AddObjectToOptitrack(quad1);
  sim->AddObjectToOptitrack(quad2);

  //
  sim->SetFlightmare(true);

  // add gate to the simulator
  std::string gate_name = "gate01";
  std::string gate_prefab_name = "rpg_gate";
  std::shared_ptr<Simulator::UnityGate>
    gate01 = std::make_shared<Simulator::UnityGate>(gate_name, gate_prefab_name);
  gate01->SetPosition(Eigen::Vector3d(0.0, 0.0, 2.4));
  gate01->SetRotation(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
  gate01->SetSize(Eigen::Vector3d(1.0, 1.0, 1.0));

  // add objects to unity for simulation and visulization.
  sim->AddObjectToUnity(gate01);
  sim->AddObjectToUnity(quadRGB1);
  sim->AddObjectToUnity(quadRGB2);

  // set up multi-purpose timer
  std::shared_ptr<ExtTimer> timer;
  timer.reset(new ExtTimer(sim->GetSimTimer()));

  // set up estimator
  OptitrackEstimator est1(quadID1);
  OptitrackEstimator est2(quadID2);
  //
  ros::Subscriber poseSub1;
  ros::Subscriber poseSub2;
  poseSub1 = nh.subscribe("/rpgq_simulator/optitrack/" + quadName1, 10, &OptitrackEstimator::MeasurementUpdate, &est1);
  poseSub2 = nh.subscribe("/rpgq_simulator/optitrack/" + quadName2, 10, &OptitrackEstimator::MeasurementUpdate, &est2);

  // set up controller
  LowLevelOffboardController ctrl1(quadID1, timer);
  ctrl1.SetCommandLevel(LowLevelOffboardController::CommandLevel::POS_CMD);
  ctrl1.SetPosDes(Eigen::Vector3d(0.0, 0.0, 2.0));
  ctrl1.SetYawDes(0.0);

  LowLevelOffboardController ctrl2(quadID2, timer);
  ctrl2.SetCommandLevel(LowLevelOffboardController::CommandLevel::POS_CMD);
  ctrl2.SetPosDes(Eigen::Vector3d(3.0, 0.0, 2.0));
  ctrl2.SetYawDes(0.0);

  bool fig8Started = false;
  Trajectory trajectory1(sim->GetSimTimer());
  trajectory1.SetStartPose(Eigen::Vector3d(0.0, 0.0, 2.0), 0.0);

  Trajectory trajectory2(sim->GetSimTimer());
  trajectory2.SetStartPose(Eigen::Vector3d(3.0, 0.0, 2.0), 0.0);


  // main loop
  Timer loopTimer;

  while (ros::ok())
  {

    // enable visualization and add objects
    if (!sim->FlightmareIsReady())
    {
      sim->StartFlightmare(sceneName);
    }
    // start measuring time
    loopTimer.Reset();

    // run ros callbacks
    ros::spinOnce();

    est1.Predict();//ros::Time::now()); // TODO
    est2.Predict();//ros::Time::now()); //

    if (sim->ElapsedSeconds() > 2.0)
    {
      if (!fig8Started)
      {
          fig8Started = trajectory1.StartMotion(Trajectory::Motion::Figure8);
          fig8Started = trajectory2.StartMotion(Trajectory::Motion::Figure8);
      }

      if (fig8Started)
      {
        TrajectoryState trajStateDes1 = trajectory1.GetCurrentState();
        ctrl1.SetPosDes(trajStateDes1.pos);
        ctrl1.SetVelDes(trajStateDes1.vel);
        ctrl1.SetAccDes(trajStateDes1.acc);
        ctrl1.SetYawDes(trajStateDes1.yaw);
        ctrl1.SetOmegaDes(trajStateDes1.omega);

        TrajectoryState trajStateDes2 = trajectory2.GetCurrentState();
        ctrl2.SetPosDes(trajStateDes2.pos);
        ctrl2.SetVelDes(trajStateDes2.vel);
        ctrl2.SetAccDes(trajStateDes2.acc);
        ctrl2.SetYawDes(trajStateDes2.yaw);
        ctrl2.SetOmegaDes(trajStateDes2.omega);
      }
    }

    // run controller
    rpgq_msgs::CommandSet cmdSet1;
    rpgq_msgs::CommandSet cmdSet2;
    if (timer->ElapsedSeconds() < 1.0)
    {
      rpgq_msgs::AddCommandToCommandSet(rpgq_msgs::CreateArmCommand(quadID1), cmdSet1);
      rpgq_msgs::AddCommandToCommandSet(rpgq_msgs::CreateArmCommand(quadID2), cmdSet2);
    }
    else
    {
      ctrl1.Run(est1.GetPredictedState());
      rpgq_msgs::AddCommandToCommandSet(ctrl1.GetCommand(), cmdSet1);

      ctrl2.Run(est2.GetPredictedState());
      rpgq_msgs::AddCommandToCommandSet(ctrl2.GetCommand(), cmdSet2);
    }

    // feed estimator with commands
    est1.FeedCommandQueue(cmdSet1);
    sim->SetCommandSet(cmdSet1);
    sim->Run(1.0/CONTROL_UPDATE_RATE);
    // big hack
    est2.FeedCommandQueue(cmdSet2);
    sim->SetCommandSet(cmdSet2);
    sim->Run(1.0/CONTROL_UPDATE_RATE);

    // stop measuring time, sleep accordingly
    double secsToSleep = 1.0/CONTROL_UPDATE_RATE - loopTimer.ElapsedSeconds();
    if (secsToSleep > 0.0)
    {
      usleep(secsToSleep*1e6);
    }
  }
  return 0;
}