#pragma once

// others
#include <eigen3/Eigen/Dense>

namespace RPGQ
{
    struct State6DoF
    {
        Eigen::Vector3d pos;
        Eigen::Vector3d vel;
        Eigen::Quaterniond quat;
        Eigen::Vector3d omega;

        State6DoF(void):
            pos(0.0,0.0,0.0),
            vel(0.0,0.0,0.0),
            quat(1.0,0.0,0.0,0.0),
            omega(0.0,0.0,0.0)
        {};
    };

    struct State6DoFExt : State6DoF
    {
        Eigen::Vector3d acc;
        Eigen::Vector3d psi;

        State6DoFExt(void):
            State6DoF(),
            acc(0.0,0.0,0.0),
            psi(0.0,0.0,0.0)
        {};
    };

} // namespace RPGQ