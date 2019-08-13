#include <rpgq_simulator/visualization/visualizer.h>

// rpgq rpgq_common
#include <rpgq_common/opengl/shader.h>
#include <rpgq_common/opengl/model.h>
#include <rpgq_common/opengl/filesystem.h>

// standard library
#include <cstdio>
#include <functional>

// others
#include <glad/glad.h>
#include <glm/gtc/type_ptr.hpp>
#include <GLFW/glfw3.h>

namespace RPGQ
{
    namespace Simulator
    {
        // constructor & destructor
        Visualizer::Visualizer(USecs maxSimUSecsInterval):
            maxSimUSecsInterval_(maxSimUSecsInterval)
        {
            // general
            initialized_ = false;

            // timing variables
            usecsNextSim_ = maxSimUSecsInterval_;

            // parameters
            usecsNextSim_ = 
            width_ = 1080;
            height_ = 720;
            fx_ = 660.0;
            fy_ = 660.0;
            u0_ = 540.0;
            v0_ = 360.0;
            minDepth_ = 0.2;
            maxDepth_ = 40.0;
            mouseSensitivityOrientation_ = 0.1;
            mouseSensitivityForward_ = 0.75;
            mouseSensitivitySideways_ = 0.01;
            R_BC_ = Eigen::Matrix3d::Zero();
            R_BC_(1,0) = -1.0;
            R_BC_(2,1) = -1.0;
            R_BC_(0,2) = 1.0;

            // initialize OpenGL projection
            projection_ = ComputeOpenGLProjection(fx_, fy_, u0_, v0_, width_, height_, minDepth_, maxDepth_);

            // initialize and configure glfw
            glfwInit();
            glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
            glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
            glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

            // create glfw window
            window_ = glfwCreateWindow(width_, height_, "Simulation Visualization", NULL, NULL);
            if (window_ == NULL)
            {
                std::printf("Simulation visualization failed to create window.\n");
                glfwTerminate();
                return;
            }
            glfwMakeContextCurrent(window_);
            glfwSwapInterval(0);

            // camera motion
            position_ = Eigen::Vector3d(-5.0, -5.0, 5.0);
            quat_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
            forward_ = Eigen::Vector3d(1.0, 0.0, 0.0);
            left_ = Eigen::Vector3d(0.0, -1.0, 0.0);
            up_ = Eigen::Vector3d(0.0, 0.0, 1.0);
            yaw_ = 45.0;
            pitch_ = 15.0;
            currLeftButtonPressed_ = false;
            lastLeftButtonPressed_ = false;
            currRightButtonPressed_ = false;
            lastRightButtonPressed_ = false;
            currXPos_ = 0.0;
            currYPos_ = 0.0;
            lastXPos_ = 0.0;
            lastYPos_ = 0.0;
            currYOffset_ = 0.0;
            UpdateCameraVectors();
            glfwSetWindowUserPointer(window_, this);
            {
                auto func = [](GLFWwindow* w, double xPos, double yPos)
                {
                    static_cast<Visualizer*>(glfwGetWindowUserPointer(w))->MouseCallback(xPos, yPos);
                };

                glfwSetCursorPosCallback(window_, func);
            }
            {
                auto func = [](GLFWwindow* w, int button, int action, int)
                {
                    static_cast<Visualizer*>(glfwGetWindowUserPointer(w))->MouseButtonCallback(button, action);
                };

                glfwSetMouseButtonCallback(window_, func);
            }
            {
                auto func = [](GLFWwindow* w, double, double yOffset)
                {
                    static_cast<Visualizer*>(glfwGetWindowUserPointer(w))->MouseScrollCallback(yOffset);
                };

                glfwSetScrollCallback(window_, func);
            }
            glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

            // load OpenGL function points
            if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
            {
                std::printf("Simulation visualization failed to initialize GLAD.\n");
                return;
            }

            // create and attach a color buffer
            glGenRenderbuffers(1, &colorBuf_);
            glBindRenderbuffer(GL_RENDERBUFFER, colorBuf_);
            glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, width_, height_); // set format
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, colorBuf_); // attach color buffer to window

            // create and attach a depth buffer
            glGenRenderbuffers(1, &depthBuf_);
            glBindRenderbuffer(GL_RENDERBUFFER, depthBuf_);
            glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width_, height_); // set format
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthBuf_); // attach depth buffer to window

            GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
            if(status != GL_FRAMEBUFFER_COMPLETE)
            {
                std::printf("Visualizer: Failed to set up framebuffer.\n");
            }

            // build and compile our shader
            shader_.reset(new Shader(FileSystem::getPath("src/visualization/shader.vert").c_str(),
                                    FileSystem::getPath("src/visualization/shader.frag").c_str()));

            // enable depth test to ensure objects are rendered correctly
            glEnable(GL_DEPTH_TEST);

            // set default scene
            SetScene(FileSystem::getPath("resources/scenes/default/empty_world.obj"));

            // initialization done
            initialized_ = true;
        }

        Visualizer::~Visualizer(void)
        {
            glDeleteFramebuffers(1, &colorBuf_);
            glDeleteFramebuffers(1, &depthBuf_);

            glfwTerminate();
        }


        // main function
        void Visualizer::Render(USecs dt)
        {
            // timing
            if (dt >= usecsNextSim_)
            {
                usecsNextSim_ = maxSimUSecsInterval_;
                // continue with code
            }
            else
            {
                usecsNextSim_ -= dt;
                return;
                // don't render image
            }

            // return if not everything initialized
            if (!initialized_) return;

            // set context
            glfwMakeContextCurrent(window_);

            // process input
            ProcessInput();

            // draw screen
            glBindFramebuffer(GL_FRAMEBUFFER, 0);

            // render
            glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // activate shader
            shader_->use();

            // draw scene
            glm::mat4 model = glm::mat4(1.0f);

            Eigen::Matrix4d T_WC = Eigen::Matrix4d::Zero();
            T_WC.block<3,3>(0,0) = quat_.normalized().toRotationMatrix()*R_BC_;
            T_WC.block<3,1>(0,3) = position_;
            T_WC(3,3) = 1.0;

            Eigen::Matrix4d T_CW = ComputeInverseTransformation(T_WC);
            // invert the z-axis here because NDC coordinates are left-handed by default in OpenGL(see https://stackoverflow.com/a/12336360)
            Eigen::Matrix4d T_CW_tilde = T_CW;
            T_CW_tilde.block<1,4>(2,0) *= -1.0;

            // view = transformation from point in world to point in camera
            glm::mat4 view = glm::make_mat4(T_CW_tilde.data());

            uint32_t modelLoc = glGetUniformLocation(shader_->ID, "model");
            glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

            uint32_t viewLoc = glGetUniformLocation(shader_->ID, "view");
            glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

            uint32_t projectionLoc = glGetUniformLocation(shader_->ID, "projection");
            glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projection_));

            // enable face culling for scene
            glEnable(GL_CULL_FACE);
            glCullFace(GL_BACK);

            scene_->Draw(*shader_);

            // disable face culling for objects
            glDisable(GL_CULL_FACE);

            // draw objects
            Eigen::Matrix4d T_WObj = Eigen::Matrix4d::Zero();
            T_WObj(3,3) = 1.0;
            for (size_t i = 0; i < objects_.size(); i++)
            {
                T_WObj.block<3,3>(0,0) = GetQuatObjects_[i]().normalized().toRotationMatrix();
                T_WObj.block<3,1>(0,3) = GetPosObjects_[i]();
                model = glm::make_mat4(T_WObj.data());
                glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

                objects_[i]->Draw(*shader_);
            }

            // swap buffers and poll IO events (keys, mouse, ...)
            glfwSwapBuffers(window_);
            glfwPollEvents();
        }


        // public set functions
        void Visualizer::SetScene(const std::string &path)
        {
            // set context
            glfwMakeContextCurrent(window_);

            // set scene
            scene_.reset(new Model(path));
        }

        void Visualizer::AddObject(const std::string &path, VisualizerTypes::GetPos_t GetPos, VisualizerTypes::GetQuat_t GetQuat)
        {
            // set context
            glfwMakeContextCurrent(window_);

            // add object
            objects_.push_back(std::unique_ptr<Model>(new Model(path)));
            GetPosObjects_.push_back(GetPos);
            GetQuatObjects_.push_back(GetQuat);
        }


        // camera motion
        void Visualizer::MouseCallback(double xPos, double yPos)
        {
            currXPos_ = xPos;
            currYPos_ = yPos;
        }

        void Visualizer::MouseButtonCallback(int button, int action)
        {
            if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
            {
                currLeftButtonPressed_ = true;
            }
            if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE)
            {
                currLeftButtonPressed_ = false;
            }
            if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS)
            {
                currRightButtonPressed_ = true;
            }
            if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_RELEASE)
            {
                currRightButtonPressed_ = false;
            }
        }

        void Visualizer::MouseScrollCallback(double yOffset)
        {
            currYOffset_ = yOffset;
        }

        void Visualizer::ProcessInput(void)
        {
            // process mouse input
            if (lastLeftButtonPressed_ != currLeftButtonPressed_ || lastRightButtonPressed_ != currRightButtonPressed_)
            {
                // store cursor position for moving camera
                lastXPos_ = currXPos_;
                lastYPos_ = currYPos_;
            }
            lastLeftButtonPressed_ = currLeftButtonPressed_;
            lastRightButtonPressed_ = currRightButtonPressed_;

            // update orientation
            if (!lastLeftButtonPressed_ && lastRightButtonPressed_)
            {
                UpdateCameraOrientation(currXPos_ - lastXPos_, currYPos_ - lastYPos_);
                lastXPos_ = currXPos_;
                lastYPos_ = currYPos_;
            }

            // move camera forward
            if (currYOffset_ != 0.0)
            {
                UpdateCameraPositionForward(currYOffset_);
                currYOffset_ = 0.0;
            }

            // move camera sideways
            if (lastLeftButtonPressed_ && !lastRightButtonPressed_)
            {
                UpdateCameraPositionSideways(currXPos_ - lastXPos_, currYPos_ - lastYPos_);
                lastXPos_ = currXPos_;
                lastYPos_ = currYPos_;
            }
        }

        void Visualizer::UpdateCameraOrientation(double xOffset, double yOffset, bool constrainPitch)
        {
            xOffset *= mouseSensitivityOrientation_;
            yOffset *= mouseSensitivityOrientation_;

            yaw_ += xOffset;
            pitch_ -= yOffset;

            // constrain pitch
            if (constrainPitch)
            {
                if (pitch_ > 89.f)
                {
                    pitch_ = 89.f;
                }
                if (pitch_ < -89.f)
                {
                    pitch_ = -89.f;
                }
            }

            // update camera vectors
            UpdateCameraVectors();
        }

        void Visualizer::UpdateCameraVectors(void)
        {
            quat_ = Eigen::AngleAxisd(yaw_/180.f*M_PI, Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(pitch_/180.f*M_PI, Eigen::Vector3d::UnitY());
            Eigen::Matrix3d R_IB = quat_.normalized().toRotationMatrix();

            forward_ = R_IB.block<3,1>(0,0);
            left_ = R_IB.block<3,1>(0,1);
            up_ = R_IB.block<3,1>(0,2);
        }

        void Visualizer::UpdateCameraPositionForward(double yOffset)
        {
            position_ += forward_*mouseSensitivityForward_*yOffset;
        }

        void Visualizer::UpdateCameraPositionSideways(double xOffset, double yOffset)
        {
            position_ += left_*mouseSensitivitySideways_*xOffset;
            position_ += up_*mouseSensitivitySideways_*yOffset;
        }
        
        // auxiliary functions
        glm::mat4 Visualizer::ComputeOpenGLProjection(double fx, double fy, double u0, double v0,
            uint32_t width, uint32_t height, double minDepth, double maxDepth) const
        {
            // parameters that define the final viewport that is rendered into by the camera
            double L = 0.0, R = (double) width, B = 0.0, T = (double) height;

            // near and far clipping planes, these only matter for the mapping from world-space z-coordinate into the depth coordinate for OpenGL
            double N = minDepth;
            double F = maxDepth;

            // construct an orthographic matrix, which maps from projected coordinates to normalized device coordinates in the range [-1, 1].
            // OpenGL then maps coordinates in NDC to the current viewport.
            Eigen::Matrix4d ortho = Eigen::Matrix4d::Zero();
            ortho(0,0) =  2.0/(R-L); ortho(0,3) = -(R+L)/(R-L);
            ortho(1,1) =  2.0/(T-B); ortho(1,3) = -(T+B)/(T-B);
            ortho(2,2) = -2.0/(F-N); ortho(2,3) = -(F+N)/(F-N);
            ortho(3,3) =  1.0;

            // construct a projection matrix, this is identical to the projection matrix computed for the intrinsicx, except an additional
            // row is inserted to map the z-coordinate to OpenGL.
            double skew = 0.0;
            Eigen::Matrix4d tproj = Eigen::Matrix4d::Zero();
            tproj(0,0) = fx;    tproj(0,1) = skew;  tproj(0,2) = -u0;
                                tproj(1,1) = fy;   tproj(1,2) = -v0;
                                                    tproj(2,2) = N+F;   tproj(2,3) = N*F;
                                                    tproj(3,2) = -1.0;

            // resulting OpenGL frustum is the product of the orthographic mapping to normalized device coordinates and the augmented
            // camera intrinsic matrix
            Eigen::Matrix4d frustum = ortho*tproj;
            frustum.block<1,4>(1,0) *= -1.0;
            return glm::make_mat4(frustum.data());
        }

        Eigen::Matrix4d Visualizer::ComputeInverseTransformation(const Eigen::Matrix4d& T_AB)
        {
            Eigen::Matrix4d T_BA = Eigen::Matrix4d::Zero();
            T_BA.block<3,3>(0,0) = T_AB.block<3,3>(0,0).transpose();
            T_BA.block<3,1>(0,3) = -T_BA.block<3,3>(0,0)*T_AB.block<3,1>(0,3);
            T_BA(3,3) = 1.0;

            return T_BA;
        }

    } // namespace Simulator
} // namespace RPGQ