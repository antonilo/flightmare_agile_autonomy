#pragma once

// standard library
#include <cstdint>
#include <string>

namespace RPGQ
{
    typedef uint8_t QuadrotorID;

    const QuadrotorID QUAD_INVALID = 0;

    const QuadrotorID QUAD_DONATELLO = 4;
    const QuadrotorID QUAD_LEONARDO = 12;
    const QuadrotorID QUAD_MICHELANGELO = 13;
    const QuadrotorID QUAD_RAFFAELLO = 18;


    inline std::string QuadrotorName(QuadrotorID id)
    {
        switch (id)
        {
            case QUAD_INVALID: return "invalid quadrotor ID";

            case QUAD_DONATELLO: return "donatello";
            case QUAD_LEONARDO: return "leonardo";
            case QUAD_MICHELANGELO: return "michelangelo";
            case QUAD_RAFFAELLO: return "raffaello";

            default:
                return "invalid quadrotor ID";
        }
    }

    inline QuadrotorID QuadrotorNameToID(std::string name)
    {
        if (name == "invalid quadrotor ID") return QUAD_INVALID;

        if (name == "donatello") return QUAD_DONATELLO;
        if (name == "leonardo") return QUAD_LEONARDO;
        if (name == "michelangelo") return QUAD_MICHELANGELO;
        if (name == "raffaello") return QUAD_RAFFAELLO;

        // default
        return QUAD_INVALID;
    }
}