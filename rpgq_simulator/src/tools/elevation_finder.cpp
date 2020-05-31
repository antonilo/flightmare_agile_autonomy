#include <iostream>
#include <string>
#include <unistd.h>
#include <zmqpp/zmqpp.hpp>

#include "rpgq_simulator/tools/elevation_finder.h"


double RPGQ::Simulator::elevationFinder(double x, double y, double timeout){
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
    sub_.subscribe("GROUND");

    // wait until publisher is properly connected
    usleep(1000000);
    zmqpp::message msg;
    msg << "GROUND";
    msg << std::to_string(x);
    msg << std::to_string(y);
    pub_.send(msg, true);
    pub_.close();

    double time_out_count = 0;
    double sleep_useconds = 0.2 * 1e5;
    const double connection_time_out = timeout; // seconds
    double z = 0.0;
    //  Wait until response received
    while (true) {
        if (sub_.receive(msg, true)) {
            std::string sub_msg = msg.get(1);
            std::string::size_type sz;     // alias of size_t
            z = std::stod(sub_msg, &sz);
            break;
        }
        if (time_out_count / 1e6 > connection_time_out) {
            std::cout << "Elevation message not received, time out." << std::endl;
            break;
        }
        // sleep
        usleep(sleep_useconds);
        // increase time out counter
        time_out_count += sleep_useconds;
    }

    sub_.close();
    return z;
}
