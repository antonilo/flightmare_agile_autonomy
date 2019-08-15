// rpqg simulator
#include <rpgq_simulator/simulator.h>
#include <rpgq_simulator/implementation/objects/quadrotor_vehicle/quad_and_stereo_rgb_camera.h>
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
  std::string quadName = "leonardo";
  std::string sceneName;
  if (!pnh.getParam("scene_name", sceneName))
  {
    ROS_ERROR("[%s] Could not determine scene name.", pnh.getNamespace().c_str());
    return -1;
  }

  QuadrotorID quadID = QuadrotorNameToID(quadName);

  // create simulator
  std::shared_ptr<Simulator::Simulator> sim = std::make_shared<Simulator::Simulator>();

  // create a quadrotor with a RGB Camera on it.
  std::shared_ptr<Simulator::QuadStereoRGBCamera> quadStereoRGB = std::make_shared<Simulator::QuadStereoRGBCamera>(quadName, nullptr, 1000000);
  std::shared_ptr<Simulator::QuadrotorVehicle> quad = quadStereoRGB->GetQuad();
  quad->SetPos(Eigen::Vector3d(0.0, 0.0, 2.0));
  quad->SetQuat(Eigen::Quaterniond(std::cos(0.5*M_PI_2),0.0,0.0,std::sin(0.5*M_PI_2)));
  quad->SetSize(Eigen::Vector3d(0.1, 0.1, 0.1));
  //
  sim->AddObjectToOptitrack(quad);
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
  sim->AddObjectToUnity(quadStereoRGB);

  // set up multi-purpose timer
  std::shared_ptr<ExtTimer> timer;
  timer.reset(new ExtTimer(sim->GetSimTimer()));

  // set up estimator
  OptitrackEstimator est(quadID);
  //
  ros::Subscriber poseSub;
  //
  poseSub = nh.subscribe("/rpgq_simulator/optitrack/" + quadName, 10, &OptitrackEstimator::MeasurementUpdate, &est);

  // set up controller
  LowLevelOffboardController ctrl(quadID, timer);
  ctrl.SetCommandLevel(LowLevelOffboardController::CommandLevel::POS_CMD);
  ctrl.SetPosDes(Eigen::Vector3d(0.0, 0.0, 2.0));
  ctrl.SetYawDes(0.0);

  bool fig8Started = false;
  Trajectory trajectory(sim->GetSimTimer());
  trajectory.SetStartPose(Eigen::Vector3d(0.0, 0.0, 2.0), 0.0);

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

    est.Predict();//ros::Time::now()); // TODO

    if (sim->ElapsedSeconds() > 2.0)
    {
      if (!fig8Started)
      {
        fig8Started = trajectory.StartMotion(Trajectory::Motion::Figure8);
      }

      if (fig8Started)
      {
        TrajectoryState trajStateDes = trajectory.GetCurrentState();
        ctrl.SetPosDes(trajStateDes.pos);
        ctrl.SetVelDes(trajStateDes.vel);
        ctrl.SetAccDes(trajStateDes.acc);
        ctrl.SetYawDes(trajStateDes.yaw);
        ctrl.SetOmegaDes(trajStateDes.omega);
      }
    }

    // run controller
    rpgq_msgs::CommandSet cmdSet;

    if (timer->ElapsedSeconds() < 1.0)
    {
      rpgq_msgs::AddCommandToCommandSet(rpgq_msgs::CreateArmCommand(quadID), cmdSet);
    }
    else
    {
      ctrl.Run(est.GetPredictedState());
      rpgq_msgs::AddCommandToCommandSet(ctrl.GetCommand(), cmdSet);
    }

    // feed estimator with commands
    est.FeedCommandQueue(cmdSet);
    sim->SetCommandSet(cmdSet);
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

