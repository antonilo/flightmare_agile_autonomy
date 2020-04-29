
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
  Eigen::Matrix3d R_BC;
  R_BC << 1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0;
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
  double sleep_useconds = 0.2 * 1e5;
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
  
  FlightmareTypes::ImgID  img_id = 0;
  
  cv::Mat rgb_img;
  for (int i=0; i< 100; i++){
    quad_position << 0, 0, float(i);
    // change the quadrotor position and rotation.
    // the camera pos will be changed implicitly
    quad->SetPos(quad_position);
    quad->SetQuat(Eigen::Quaterniond(std::cos(0.5*M_PI_2),0.0,0.0,std::sin(0.5*M_PI_2)));
    
    img_id = i;
    std::cout << "send img_id: " << img_id << std::endl;
    // send message to unity (e.g., update quadrotor pose)
    flightmareBridge_ptr->getRender(img_id);

    // wait a bit for Unity simulation
    // Unfortunately, it is difficult to say how long should 
    // you wait until Unity finish rendering...
    usleep(sleep_useconds);

    // receive message update from Unity3D (e.g. receive image)
    FlightmareTypes::ImgID receive_id;
    receive_id = flightmareBridge_ptr->handleOutput(unity_output);
    std::cout << "receive img_id: " << receive_id << std::endl;
    
    // not sure if this is the most efficient way to retrieve images.
    rgb_camera->GetRGBImage(rgb_img);

    //
    
    std::string file_path = std::string(getenv("RPGQ_PARAM_DIR")) + std::string("/examples/example_vision/src/saved_image/");
    std::string img_string = std::to_string(img_id) + ".png";
    std::string file_name = file_path + img_string;
    cv::imwrite(file_name, rgb_img);
    // cv::waitKey(0);
  }
 return 0;
}
