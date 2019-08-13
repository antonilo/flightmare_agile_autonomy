#pragma once

// rpgq rpgq_common
#include <rpgq_common/time/ext_timer.h>

// standard library
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

// others
#include <eigen3/Eigen/Dense>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

// forward declarations
class GLFWwindow;

namespace RPGQ
{
    // forward declarations
    class Shader;
    class Model;

    namespace Simulator
    {
        namespace VisualizerTypes
        {
            typedef std::function<Eigen::Vector3d()> GetPos_t;
            typedef std::function<Eigen::Quaterniond()> GetQuat_t;
        }

        class Visualizer
        {
        public:
            // constructor & destructor
            Visualizer(USecs maxSimUSecsInterval);
            ~Visualizer(void);

            // main function
            void Render(USecs dt);

            // public set functions
            void SetScene(const std::string &path);
            void AddObject(const std::string &path, VisualizerTypes::GetPos_t GetPos, VisualizerTypes::GetQuat_t GetQuat);

            // public get functions
            USecs GetUSecsNextSim(void) {return usecsNextSim_;};

        private:
            // general
            bool initialized_;

            // timing variables
            const USecs maxSimUSecsInterval_;
            USecs usecsNextSim_;

            // parameters
            uint32_t width_, height_;
            double fx_, fy_, u0_, v0_;
            float minDepth_, maxDepth_;
            double mouseSensitivityOrientation_;
            double mouseSensitivityForward_;
            double mouseSensitivitySideways_;
            Eigen::Matrix3d R_BC_;

            // visualization variables
            GLFWwindow* window_;
            glm::mat4 projection_;
            std::unique_ptr<Shader> shader_;
            std::unique_ptr<Model> scene_;
            std::vector<std::unique_ptr<Model>> objects_;
            std::vector<VisualizerTypes::GetPos_t> GetPosObjects_;
            std::vector<VisualizerTypes::GetQuat_t> GetQuatObjects_;
            uint32_t colorBuf_, depthBuf_;

            // camera motion variables and functions
            Eigen::Vector3d position_;
            Eigen::Quaterniond quat_;
            Eigen::Vector3d forward_, left_, up_;
            double yaw_, pitch_;
            bool currLeftButtonPressed_, currRightButtonPressed_;
            bool lastLeftButtonPressed_, lastRightButtonPressed_;
            double currXPos_, currYPos_;
            double lastXPos_, lastYPos_;
            double currYOffset_;
            enum CameraMotion
            {
                FORWARD = 0,
                BACKWARD,
                LEFT,
                RIGHT
            };
            void MouseCallback(double xPos, double yPos);
            void MouseButtonCallback(int button, int action);
            void MouseScrollCallback(double yOffset);
            void ProcessInput(void);
            void UpdateCameraOrientation(double xOffset, double yOffset, bool constrainPitch = true);
            void UpdateCameraVectors(void);
            void UpdateCameraPositionForward(double yOffset);
            void UpdateCameraPositionSideways(double xOffset, double yOffset);

            // auxiliary functions
            glm::mat4 ComputeOpenGLProjection(double fx, double fy, double u0, double v0,
                                            uint32_t width, uint32_t height, double minDepth, double maxDepth) const;
            Eigen::Matrix4d ComputeInverseTransformation(const Eigen::Matrix4d& T_AB);
        };

    } // namespace Simulator
} //  namespace RPGQ