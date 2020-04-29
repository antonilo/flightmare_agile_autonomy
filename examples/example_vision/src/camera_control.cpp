
// rpgq simulator
#include <rpgq_simulator/implementation/objects/quadrotor_vehicle/quad_and_rgb_camera.h>
#include <rpgq_simulator/visualization/flightmare_message_types.hpp>
#include <rpgq_simulator/visualization/flightmare_bridge.hpp>

// Eigen
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace RPGQ;

int main(int argc, char * argv[])
{
  // initialize ROS
  ros::init(argc, argv, "camera_control");

  // quad ID can be any real number between
  // 0 ~ 25, each ID corresponding to a unique name
  QuadrotorID quad_ID = 1;
  std::string quad_name = QuadrotorName(quad_ID);
  // create quadrotor with a RGB camera attached.
  std::shared_ptr<Simulator::QuadRGBCamera> quad_rgb =
    std::make_shared<Simulator::QuadRGBCamera>(quad_name, nullptr, 1000000);
  
  // configure the camera
  std::shared_ptr<Simulator::RGBCamera> rgb_camera = quad_rgb->GetRGBCamera();
  // rgb_camera->EnableOpticalFlow(true);       // default is false
  // rgb_camera->EnableDepth(true);
  // rgb_camera->EnableObjectSegment(true);
  // rgb_camera->EnableCategorySegment(true);
  rgb_camera->SetWidth(320);
  rgb_camera->SetHeight(240);

  // set the relative position of the camera with respect to quadrotor center mass
  Eigen::Vector3d B_r_BC(0.0, 0.5, 0.0);
  // set the relative rotation of the camera
  Eigen::Matrix3d R_BC = Eigen::AngleAxisd(1.7177715175,
    Eigen::Vector3d(-0.862856209461017,0.357406744336593,-0.357406744336593)).toRotationMatrix();
  rgb_camera->SetRelPose(B_r_BC, R_BC);
  
  // configure the quadrotor
  std::shared_ptr<Simulator::QuadrotorVehicle> quad = quad_rgb->GetQuad();
  Eigen::Vector3d quad_position{0.0, 0.0, 2.0};
  quad->SetPos(quad_position);
  quad->SetQuat(Eigen::Quaterniond(std::cos(0.5*M_PI_2),0.0,0.0,std::sin(0.5*M_PI_2)));
  quad->SetSize(Eigen::Vector3d(1, 1, 1));
  
  // flightmare
  FlightmareTypes::SceneID scene_id = FlightmareTypes::SCENE_WAREHOUSE;
  bool flightmare_ready{false};
  Simulator::RenderMessage_t unity_output;
  
  // create flightmare birdge and connect sockets..
  std::shared_ptr<Simulator::FlightmareBridge> flightmareBridge_ptr;
  flightmareBridge_ptr = Simulator::FlightmareBridge::getInstance();
  
  //
  flightmareBridge_ptr->initializeConnections();
  flightmareBridge_ptr->addQuadRGB(quad_rgb);

  // connect to unity. 
  // please open the Unity3D standalone. 
  double time_out_count = 0;
  double sleep_useconds = 1e5;
  const double connection_time_out = 10.0; // seconds
  while (!flightmare_ready)
  {
    if (flightmareBridge_ptr != nullptr)
    {
      // connect unity
      flightmareBridge_ptr->setScene(scene_id);
      flightmare_ready = flightmareBridge_ptr->connectUnity();
    } 
    if (time_out_count/1e6 > connection_time_out)
    {
      std::cout << "Flightmare connection failed, time out." << std::endl;
      break;
    }
    // sleep
    usleep(sleep_useconds);
    // increase time out counter
    time_out_count += sleep_useconds;
  }
  
  FlightmareTypes::USecs  dt_dummy = 0;
  
  cv::Mat rgb_img;
  for (int i=0; i< 100; i++){
    quad_position << 0, 0, float(i);
    // change the quadrotor position and rotation.
    // the camera pos will be changed implicitly
    quad->SetPos(quad_position);
    quad->SetQuat(Eigen::Quaterniond(std::cos(0.5*M_PI_2),0.0,0.0,std::sin(0.5*M_PI_2)));
    
    // send message to unity (e.g., update quadrotor pose)
    flightmareBridge_ptr->getRender(dt_dummy);

    // receive message update from Unity3D (e.g. receive image)
    flightmareBridge_ptr->handleOutput(unity_output);

    // not sure if this is the most efficient way to retrieve images.
    rgb_camera->GetRGBImage(rgb_img);

    //
    cv::imshow("rgb_img", rgb_img);
    cv::waitKey(0);
    // usleep(sleep_useconds);
  }
 return 0;
}
