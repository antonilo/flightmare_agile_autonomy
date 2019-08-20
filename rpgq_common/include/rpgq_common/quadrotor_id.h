#pragma once

// standard library
#include <cstdint>
#include <string>

namespace RPGQ
{
    typedef uint16_t QuadrotorID;

    const QuadrotorID QUAD_INVALID = 255;

    const QuadrotorID QUAD_ALLIGATOR = 0;
    const QuadrotorID QUAD_BEAR = 1;
    const QuadrotorID QUAD_CAT = 2;
    const QuadrotorID QUAD_DONKEY = 3;
    const QuadrotorID QUAD_ELEPHANT = 4;
    const QuadrotorID QUAD_FLAMINGO = 5;
    const QuadrotorID QUAD_GIRAFFE = 6;
    const QuadrotorID QUAD_HIPPOPOTAMUS = 7;
    const QuadrotorID QUAD_IGUANA = 8;
    const QuadrotorID QUAD_JAGUAR = 9;
    const QuadrotorID QUAD_KANGAROO = 10;
    const QuadrotorID QUAD_LION = 11;
    const QuadrotorID QUAD_MACAW = 12;
    const QuadrotorID QUAD_NEWT = 13;
    const QuadrotorID QUAD_OSTRICH = 14;
    const QuadrotorID QUAD_PIG = 15;
    const QuadrotorID QUAD_QUAIL = 16;
    const QuadrotorID QUAD_RHINOCEROS = 17;
    const QuadrotorID QUAD_SHEEP = 18;
    const QuadrotorID QUAD_TIGER = 19;
    const QuadrotorID QUAD_URIAL = 20;
    const QuadrotorID QUAD_VOLE = 21;
    const QuadrotorID QUAD_WOLF = 22;
    const QuadrotorID QUAD_XRAY = 23;
    const QuadrotorID QUAD_YAK = 24;
    const QuadrotorID QUAD_ZEBRA = 25;


    inline std::string QuadrotorName(QuadrotorID id)
    {
        switch (id)
        {
          case QUAD_INVALID: return "invalid quadrotor ID";
          case QUAD_ALLIGATOR: return "alligator";
          case QUAD_BEAR: return "bear";
          case QUAD_CAT: return "cat";
          case QUAD_DONKEY: return "donkey";
          case QUAD_ELEPHANT: return "elephant";
          case QUAD_FLAMINGO: return "flamingo";
          case QUAD_GIRAFFE: return "giraffe";
          case QUAD_HIPPOPOTAMUS: return "hippopotamus";
          case QUAD_IGUANA: return "iguana";
          case QUAD_JAGUAR: return "jaguar";
          case QUAD_KANGAROO: return "kangaroo";
          case QUAD_LION: return "lion";
          case QUAD_MACAW: return "macaw";
          case QUAD_NEWT: return "newt";
          case QUAD_OSTRICH: return "ostrich";
          case QUAD_PIG: return "pig";
          case QUAD_QUAIL: return "quail";
          case QUAD_RHINOCEROS: return "rhinoceros";
          case QUAD_SHEEP: return "sheep";
          case QUAD_TIGER: return "tiger";
          case QUAD_URIAL: return "urial";
          case QUAD_VOLE: return "vole";
          case QUAD_WOLF: return "wolf";
          case QUAD_XRAY: return "xray";
          case QUAD_YAK: return "yak";
          case QUAD_ZEBRA: return "zebra";
          default:
              return "invalid quadrotor ID";
        }
    }

    inline QuadrotorID QuadrotorNameToID(std::string name)
    {
      if (name == "invalid quadrotor ID") return QUAD_INVALID;

      if (name=="alligator") return QUAD_ALLIGATOR;
      if (name=="bear") return QUAD_BEAR;
      if (name=="cat") return QUAD_CAT;
      if (name == "donkey") return QUAD_DONKEY;
      if (name=="elephant") return QUAD_ELEPHANT;
      if (name=="flamingo") return QUAD_FLAMINGO;
      if (name=="giraffe") return QUAD_GIRAFFE;
      if (name=="hippopotamus") return QUAD_HIPPOPOTAMUS;
      if (name=="iguana") return QUAD_IGUANA;
      if (name=="jaguar") return QUAD_JAGUAR;
      if (name=="kangaroo") return QUAD_KANGAROO;
      if (name == "lion") return QUAD_LION;
      if (name == "macaw") return QUAD_MACAW;
      if (name=="newt") return QUAD_NEWT;
      if (name=="ostrich") return QUAD_OSTRICH;
      if (name=="pig") return QUAD_PIG;
      if (name=="quail") return QUAD_QUAIL;
      if (name == "rhinoceros") return QUAD_RHINOCEROS;
      if (name=="sheep") return QUAD_SHEEP;
      if (name=="tiger") return QUAD_TIGER;
      if (name=="urial") return QUAD_URIAL;
      if (name=="vole") return QUAD_VOLE;
      if (name=="wolf") return QUAD_WOLF;
      if (name=="xray") return QUAD_XRAY;
      if (name=="yak") return QUAD_YAK;
      if (name=="zebra") return QUAD_ZEBRA;
      // default
      return QUAD_INVALID;
    }
}