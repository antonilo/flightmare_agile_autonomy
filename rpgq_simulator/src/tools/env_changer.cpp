#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <string>
#include <unistd.h>
#include <cmath>
#include <zmqpp/zmqpp.hpp>

#include <rpgq_simulator/visualization/flightmare_message_types.hpp>
#include "rpgq_simulator/tools/json.hpp"
#include "rpgq_simulator/tools/env_changer.h"

using json = nlohmann::json;


void RPGQ::Simulator::placeTrees(RPGQ::Simulator::TreeMessage_t &tree_msg, double timeout){
    std::string client_address_{"tcp://*"};
    std::string pub_port_{"10255"};
    zmqpp::context context_;
    zmqpp::socket pub_{context_, zmqpp::socket_type::publish};
    pub_.set(zmqpp::socket_option::send_high_water_mark, 6);
    pub_.bind(client_address_ + ":" + pub_port_);


    std::string sub_port_{"10256"};
    zmqpp::socket sub_{context_, zmqpp::socket_type::subscribe};
    sub_.set(zmqpp::socket_option::receive_high_water_mark, 6);
    sub_.bind(client_address_ + ":" + sub_port_);
    sub_.subscribe("PLACETREE");

    // wait until publisher is properly connected
    usleep(1000000);
    zmqpp::message msg;
    msg << "PLACETREE";
    
    // check if seed is not initialized
    if (tree_msg.seed == -1)
        tree_msg.seed = rand();

    json json_msg = tree_msg;
    msg << json_msg.dump();
    pub_.send(msg, true);
    pub_.close();

    double time_out_count = 0;
    double sleep_useconds = 0.2 * 1e5;
    const double connection_time_out = timeout; // seconds
    double z = 0.0;
    //  Wait until response received
    while (true) {
        if (sub_.receive(msg, true)) {
            break;
        }
        if (time_out_count / 1e6 > connection_time_out) {
            std::cout << "Tree placed message not received, time out." << std::endl;
            break;
        }
        // sleep
        usleep(sleep_useconds);
        // increase time out counter
        time_out_count += sleep_useconds;
    }

    sub_.close();
}

void RPGQ::Simulator::rmTrees(){
    std::string client_address_{"tcp://*"};
    std::string pub_port_{"10255"};
    zmqpp::context context_;
    zmqpp::socket pub_{context_, zmqpp::socket_type::publish};
    pub_.set(zmqpp::socket_option::send_high_water_mark, 6);
    pub_.bind(client_address_ + ":" + pub_port_);

    // wait until publisher is properly connected
    usleep(1000000);
    zmqpp::message msg;
    msg << "RMTREE";
    pub_.send(msg, true);
    pub_.close();

}

void RPGQ::Simulator::placeObjects(ObjectMessage_t &obj_msg, double timeout){
    std::string client_address_{"tcp://*"};
    std::string pub_port_{"10255"};
    zmqpp::context context_;
    zmqpp::socket pub_{context_, zmqpp::socket_type::publish};
    pub_.set(zmqpp::socket_option::send_high_water_mark, 6);
    pub_.bind(client_address_ + ":" + pub_port_);


    std::string sub_port_{"10256"};
    zmqpp::socket sub_{context_, zmqpp::socket_type::subscribe};
    sub_.set(zmqpp::socket_option::receive_high_water_mark, 6);
    sub_.bind(client_address_ + ":" + sub_port_);
    sub_.subscribe("PLACEOBJECT");

    // wait until publisher is properly connected
    usleep(1000000);
    zmqpp::message msg;
    msg << "PLACEOBJECT";
    
    // check if seed is not initialized
    if (obj_msg.seed == -1)
        obj_msg.seed = rand();

    json json_msg = obj_msg;
    msg << json_msg.dump();
    pub_.send(msg, true);
    pub_.close();

    double time_out_count = 0;
    double sleep_useconds = 0.2 * 1e5;
    const double connection_time_out = timeout; // seconds
    double z = 0.0;
    //  Wait until response received
    while (true) {
        if (sub_.receive(msg, true)) {
            break;
        }
        if (time_out_count / 1e6 > connection_time_out) {
            std::cout << "Object placed message not received, time out." << std::endl;
            break;
        }
        // sleep
        usleep(sleep_useconds);
        // increase time out counter
        time_out_count += sleep_useconds;
    }
    sub_.close();

}

void RPGQ::Simulator::placeFixRatioObjects(FixRatioObjectMessage_t &obj_msg, double timeout){
    std::string client_address_{"tcp://*"};
    std::string pub_port_{"10255"};
    zmqpp::context context_;
    zmqpp::socket pub_{context_, zmqpp::socket_type::publish};
    pub_.set(zmqpp::socket_option::send_high_water_mark, 6);
    pub_.bind(client_address_ + ":" + pub_port_);


    std::string sub_port_{"10256"};
    zmqpp::socket sub_{context_, zmqpp::socket_type::subscribe};
    sub_.set(zmqpp::socket_option::receive_high_water_mark, 6);
    sub_.bind(client_address_ + ":" + sub_port_);
    sub_.subscribe("PLACEFIXRATIOOBJECT");

    // wait until publisher is properly connected
    usleep(1000000);
    zmqpp::message msg;
    msg << "PLACEFIXRATIOOBJECT";
    
    // check if seed is not initialized
    if (obj_msg.seed == -1)
        obj_msg.seed = rand();

    json json_msg = obj_msg;
    msg << json_msg.dump();
    pub_.send(msg, true);
    pub_.close();

    double time_out_count = 0;
    double sleep_useconds = 0.2 * 1e5;
    const double connection_time_out = timeout; // seconds
    double z = 0.0;
    //  Wait until response received
    while (true) {
        if (sub_.receive(msg, true)) {
            break;
        }
        if (time_out_count / 1e6 > connection_time_out) {
            std::cout << "Object placed message not received, time out." << std::endl;
            break;
        }
        // sleep
        usleep(sleep_useconds);
        // increase time out counter
        time_out_count += sleep_useconds;
    }
    sub_.close();
}

void RPGQ::Simulator::rmObjects(){
    std::string client_address_{"tcp://*"};
    std::string pub_port_{"10255"};
    zmqpp::context context_;
    zmqpp::socket pub_{context_, zmqpp::socket_type::publish};
    pub_.set(zmqpp::socket_option::send_high_water_mark, 6);
    pub_.bind(client_address_ + ":" + pub_port_);

    // wait until publisher is properly connected
    usleep(1000000);
    zmqpp::message msg;
    msg << "RMOBJECT";
    pub_.send(msg, true);
    pub_.close();
}

void RPGQ::Simulator::changeLight(RPGQ::Simulator::LightMessage_t &light_msg){
    std::string client_address_{"tcp://*"};
    std::string pub_port_{"10255"};
    zmqpp::context context_;
    zmqpp::socket pub_{context_, zmqpp::socket_type::publish};
    pub_.set(zmqpp::socket_option::send_high_water_mark, 6);
    pub_.bind(client_address_ + ":" + pub_port_);

    // wait until publisher is properly connected
    usleep(1000000);
    zmqpp::message msg;
    msg << "LIGHT";
    json json_msg = light_msg;
    msg << json_msg.dump();
    pub_.send(msg, true);
    pub_.close();
}


void RPGQ::Simulator::HSVToRGB(int H, double S, double V, int output[3]) {
	double C = S * V;
	double X = C * (1 - abs(fmod(H / 60.0, 2) - 1));
	double m = V - C;
	double Rs, Gs, Bs;

	if(H >= 0 && H < 60) {
		Rs = C;
		Gs = X;
		Bs = 0;	
	}
	else if(H >= 60 && H < 120) {	
		Rs = X;
		Gs = C;
		Bs = 0;	
	}
	else if(H >= 120 && H < 180) {
		Rs = 0;
		Gs = C;
		Bs = X;	
	}
	else if(H >= 180 && H < 240) {
		Rs = 0;
		Gs = X;
		Bs = C;	
	}
	else if(H >= 240 && H < 300) {
		Rs = X;
		Gs = 0;
		Bs = C;	
	}
	else {
		Rs = C;
		Gs = 0;
		Bs = X;	
	}
	
	output[0] = (Rs + m) * 255;
	output[1] = (Gs + m) * 255;
	output[2] = (Bs + m) * 255;
}