#pragma once

namespace RPGQ {
    namespace Simulator {

        /*
        Send ZMQ message with the desired x, y coordinates to receive the 
        corresponding elevation z at the given positon on the map.

        Unity standalone has to be running while using this command.

        */
        double elevationFinder(double x, double y, double timeout = 60.0);

    }//Simulator
}//RPGQ