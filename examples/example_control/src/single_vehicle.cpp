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
#include <trajectory/fig8_trajectory.h>

#define CONTROL_UPDATE_RATE 50.0

using namespace RPGQ;

int main(int argc, char * argv[]) {
  // initialize ROS
  ros::init(argc, argv, "flightmare_example");
  ros::NodeHandle nh("");
  ros::NodeHandle pnh("~");

  // define the scene id
  FlightmareTypes::SceneID scene_id = FlightmareTypes::SCENE_WAREHOUSE;

  // quad ID can be any real number between
  // 0 ~ 25, each ID corresponding to a unique
  // quad name
  QuadrotorID quadID = 1;
  std::string quadName = QuadrotorName(quadID);

  // create simulator
  std::shared_ptr<Simulator::Simulator> sim = std::make_shared<Simulator::Simulator>();
  // enable flightmare for visualization
  sim->SetFlightmare(true);

  // create a quadrotor with a RGB Camera on it.
  std::shared_ptr<Simulator::QuadrotorVehicle> quad =
    std::make_shared<Simulator::QuadrotorVehicle>(quadName, nullptr, 1000000);
  Eigen::Vector3d init_pose{0.0, 0.0, 3.0};
  quad->SetPos(init_pose);
  quad->SetQuat(Eigen::Quaterniond(std::cos(0.5*M_PI_2),0.0,0.0,std::sin(0.5*M_PI_2)));
  quad->SetSize(Eigen::Vector3d(0.3, 0.3, 0.3));

  // add quad to simulated optitrack for
  // pose estimation of the quadrotor
  sim->AddObjectToOptitrack(quad);

  // add quadrotor to Unity standalone
  // for visulization
  sim->AddObjectToUnity(quad);

  // set up multi-purpose timer
  std::shared_ptr<ExtTimer> timer;
  timer.reset(new ExtTimer(sim->GetSimTimer()));

  // set up estimator
  OptitrackEstimator est(quadID);

  // set up pose subscriber
  ros::Subscriber poseSub;
  poseSub = nh.subscribe("/rpgq_simulator/optitrack/" + quadName, 10, &OptitrackEstimator::MeasurementUpdate, &est);

  // set up controller
  LowLevelOffboardController ctrl(quadID, timer);
  ctrl.SetCommandLevel(LowLevelOffboardController::CommandLevel::POS_CMD);
  ctrl.SetPosDes(init_pose);
  ctrl.SetYawDes(0.0);

  // set up figure8 trajectory
  bool fig8Started = false;
  Trajectory trajectory(sim->GetSimTimer());
  trajectory.SetStartPose(init_pose, 0.0);

  // main loop
  Timer loopTimer;

  while (ros::ok())
  {
    // enable visualization and add objects
    if (!sim->FlightmareIsReady()){
      sim->ConnectFlightmare(scene_id);
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

    // run simulation
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

