#pragma once

#include <rpgq_simulator/visualization/flightmare_message_types.hpp>

namespace RPGQ {
    namespace Simulator {

        /*
        Send ZMQ message with the desired origin, area and density of trees that
        should be placed.

        The trees are placed on the ground at the given x,y postion.

        Unity standalone has to be running while using this command.

        */
        void placeTrees(TreeMessage_t &tree_msg, double timeout = 60.0);

        /*
        All placed trees are removed from the scene.

        Does not remove trees that have been added to the scene in the unity 
        editor without the tree tag.
        */
        void rmTrees();

        /*
        Change color and intensity of all the light objects in the scene.

        Does not change the skybox or backed lightning.
        */
        void changeLight(LightMessage_t &light_msg);

        /*
        Convert HSV to RGB.
        
        H(Hue): 0 - 360 degree (integer)
        S(Saturation): 0 - 1.00 (double)
        V(Value): 0 - 1.00 (double)
        
        output[3]: Output, array size 3, int
        */
        void HSVToRGB(int H, double S, double V, int output[3]);

    }  //Simulator
}  //RPGQ
