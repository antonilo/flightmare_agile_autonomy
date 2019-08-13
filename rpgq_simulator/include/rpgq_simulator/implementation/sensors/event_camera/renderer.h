#pragma once

// rpgq rpgq_simulator
#include <rpgq_simulator/implementation/sensors/event_camera/event_camera_types.h>

// standard library
#include <cstdint>
#include <memory>

// others
#include <glad/glad.h>
#include <glm/glm.hpp>

// forward declarations
class GLFWwindow;

namespace RPGQ
{
    // forward declarations
    class Shader;
    class Model;

    namespace Simulator
    {
        class Renderer
        {
        public:
            // constructor & destructor
            Renderer();
            ~Renderer();

            // initialize renderer
            void Initialize(void);

            // render image
            void Render(const EventCameraTypes::Mat4_t& T_WC,
                        const EventCameraTypes::Vec3_t& W_v_C,
                        const EventCameraTypes::Vec3_t& W_omega_WC,
                        const std::vector<EventCameraTypes::Mat4_t>& T_WK,
                        const std::vector<EventCameraTypes::Vec3_t>& W_v_K,
                        const std::vector<EventCameraTypes::Vec3_t>& W_omega_WK,
                        std::shared_ptr<EventCameraTypes::Image_t> image,
                        std::shared_ptr<EventCameraTypes::Depthmap_t> depthmap,
                        std::shared_ptr<EventCameraTypes::OpticFlow_t> opticFlow);


            // public set functions
            void SetCameraParameters(double fx, double fy, double u0, double v0,
                                    int32_t width, uint32_t height);
            void SetSceneModel(const std::string &path);
            void AddObjectModel(const std::string &path);

        private:
            // general
            bool initialized_;

            uint32_t width_;
            uint32_t height_;
            double fx_, fy_, u0_, v0_;
            glm::mat4 projection_;

            float minDepth_;
            float maxDepth_;

            GLFWwindow* window_;
            std::unique_ptr<Shader> shader_;
            std::unique_ptr<Shader> opticFlowShader_;
            std::unique_ptr<Model> sceneModel_;
            std::vector<std::unique_ptr<Model>> dynamicObjectModels_;
            uint8_t numObjects_;

            // framebuffers and renderbuffers for color, depth and optical flow images
            uint32_t multisampledFbo_, fbo_, opticalFlowFbo_;
            uint32_t multisampledColorBuf_, colorBuf_, opticalFlowBuf_;
            uint32_t multisampledDepthBuf_, depthBuf_, opticalFlowDepthBuf_;

            // auxiliary functions
            glm::mat4 ComputeOpenGLProjection(double fx, double fy, double u0, double v0,
                                            uint32_t width, uint32_t height, double minDepth, double maxDepth) const;
            EventCameraTypes::Mat4_t ComputeInverseTransformation(const EventCameraTypes::Mat4_t& T);
        };

    } // namespace Simulator
} // namespace RPGQ