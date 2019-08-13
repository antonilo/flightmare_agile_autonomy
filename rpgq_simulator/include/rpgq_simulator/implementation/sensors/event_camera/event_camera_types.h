#pragma once

// standard library
#include <functional>
#include <stdint.h>

// others
#include <opencv2/core.hpp>
#include <eigen3/Eigen/Dense>

namespace RPGQ
{
    namespace Simulator
    {
        namespace EventCameraTypes
        {
            typedef float Intensity_t;
            typedef cv::Mat_<Intensity_t> Image_t;
            typedef cv::Mat_<Intensity_t> Depthmap_t;
            typedef cv::Mat_<cv::Vec<Intensity_t, 2>> OpticFlow_t;

            typedef Eigen::Matrix4d Mat4_t;
            typedef Eigen::Vector3d Vec3_t;

            typedef std::function<Eigen::Vector3d()> GetPos_t;
            typedef std::function<Eigen::Vector3d()> GetVel_t;
            typedef std::function<Eigen::Vector3d()> GetAcc_t;
            typedef std::function<Eigen::Quaterniond()> GetQuat_t;
            typedef std::function<Eigen::Vector3d()> GetOmega_t;
            typedef std::function<Eigen::Vector3d()> GetPsi_t;

            struct Event
            {
                Event(uint16_t x, uint16_t y, uint64_t t, bool pol):
                    x(x),
                    y(y),
                    t(t),
                    pol(pol)
                {};

                uint16_t x;
                uint16_t y;
                uint64_t t;
                bool pol;
            };
            
            typedef std::vector<Event> Events;
        }

    } // namespace Simulator
} // namespace RPGQ