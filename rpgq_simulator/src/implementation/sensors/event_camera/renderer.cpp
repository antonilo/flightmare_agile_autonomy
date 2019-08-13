#include <rpgq_simulator/implementation/sensors/event_camera/renderer.h>

// ros
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// rpgq rpgq_common
#include <rpgq_common/opengl/shader.h>
#include <rpgq_common/opengl/model.h>
#include <rpgq_common/opengl/filesystem.h>

// others
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define RENDERER_DEBUG_OUTPUT 0
#define RENDERER_PUBLISH_COLOR_FRAME 0

#if RENDERER_PUBLISH_COLOR_FRAME
static image_transport::Publisher frameColorPub;
#endif

namespace RPGQ
{
    namespace Simulator
{
    // constructor & destructor
    Renderer::Renderer()
    {
        initialized_ = false;

        minDepth_ = 0.1;
        maxDepth_ = 20.0;

        numObjects_ = 0;

#if RENDERER_PUBLISH_COLOR_FRAME
        int argc = 0;
        ros::init(argc, (char **) nullptr, "renderer_debug");
        ros::NodeHandle pnh =  ros::NodeHandle("~");
        image_transport::ImageTransport it(pnh);
        frameColorPub = it.advertise("/colorFrame", 1);
#endif
    }

    Renderer::~Renderer()
    {
        // cleanup
        glDeleteFramebuffers(1, &multisampledFbo_);
        glDeleteFramebuffers(1, &fbo_);
        glDeleteFramebuffers(1, &opticalFlowFbo_);

        glDeleteRenderbuffers(1, &multisampledColorBuf_);
        glDeleteRenderbuffers(1, &multisampledDepthBuf_);
        glDeleteRenderbuffers(1, &colorBuf_);
        glDeleteRenderbuffers(1, &depthBuf_);
        glDeleteRenderbuffers(1, &opticalFlowBuf_);
        glDeleteRenderbuffers(1, &opticalFlowDepthBuf_);

        // glfw: terminate, clearing all previously allocated GLFW resources
        glfwTerminate();
    }


    // initialize renderer
    void Renderer::Initialize(void)
    {
        // initialize OpenGL projection
        projection_ = ComputeOpenGLProjection(fx_, fy_, u0_, v0_, width_, height_, minDepth_, maxDepth_);

        // glfw: initialize and configure
        glfwInit();
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_VISIBLE, GL_FALSE);

        // glfw window creation
        window_ = glfwCreateWindow(width_, height_, "OpenGLRenderer", NULL, NULL);
        if (window_ == NULL)
        {
#if  RENDERER_DEBUG_OUTPUT
            std::printf("Renderer: Failed to create GLFW window.\n");
#endif
            glfwTerminate();
            return;
        }
            glfwMakeContextCurrent(window_);
            glfwSwapInterval(0);

            // glad: load all OpenGL function pointers
            if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
            {
    #if  RENDERER_DEBUG_OUTPUT
                std::printf("Renderer: Failed to initialize GLAD.\n");
    #endif
                return;
            }

            // create multisampled framebuffer object
            const int num_samples = 8;
            glGenFramebuffers(1, &multisampledFbo_);
            glBindFramebuffer(GL_FRAMEBUFFER, multisampledFbo_);

            // create and attach a multisampled color buffer
            glGenRenderbuffers(1, &multisampledColorBuf_);
            glBindRenderbuffer(GL_RENDERBUFFER, multisampledColorBuf_);
            glRenderbufferStorageMultisample(GL_RENDERBUFFER, num_samples, GL_RGBA8, width_, height_); // set format
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, multisampledColorBuf_); // attach color buffer to FBO

            // create and attach a multisampled depth buffer
            glGenRenderbuffers(1, &multisampledDepthBuf_);
            glBindRenderbuffer(GL_RENDERBUFFER, multisampledDepthBuf_);
            glRenderbufferStorageMultisample(GL_RENDERBUFFER, num_samples, GL_DEPTH_COMPONENT24, width_, height_); // set format
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, multisampledDepthBuf_); // attach depth buffer to FBO

            GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
            if(status != GL_FRAMEBUFFER_COMPLETE)
            {
    #if  RENDERER_DEBUG_OUTPUT
                std::printf("Renderer: Failed to set up multisampled framebuffer.\n");
    #endif        
            }

            // create framebuffer object
            glGenFramebuffers(1, &fbo_);
            glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

            // create and attach a color buffer
            glGenRenderbuffers(1, &colorBuf_);
            glBindRenderbuffer(GL_RENDERBUFFER, colorBuf_);
            glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, width_, height_); // set format
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, colorBuf_); // attach color buffer to FBO

            // create and attach a depth buffer
            glGenRenderbuffers(1, &depthBuf_);
            glBindRenderbuffer(GL_RENDERBUFFER, depthBuf_);
            glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width_, height_); // set format
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthBuf_); // attach depth buffer to FBO

            status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
            if(status != GL_FRAMEBUFFER_COMPLETE)
            {
    #if  RENDERER_DEBUG_OUTPUT
                std::printf("Renderer: Failed to set up framebuffer.\n");
    #endif
            }

            // create framebuffer object for optic flow
            glGenFramebuffers(1, &opticalFlowFbo_);
            glBindFramebuffer(GL_FRAMEBUFFER, opticalFlowFbo_);

            // create and attach a color buffer for optic flow
            glGenRenderbuffers(1, &opticalFlowBuf_);
            glBindRenderbuffer(GL_RENDERBUFFER, opticalFlowBuf_);
            glRenderbufferStorage(GL_RENDERBUFFER, GL_RGB32F, width_, height_); // set format
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, opticalFlowBuf_); // attach optic flow buffer to frame buffer object

            // create and attach a depth buffer for optic flow
            glGenRenderbuffers(1, &opticalFlowDepthBuf_);
            glBindRenderbuffer(GL_RENDERBUFFER, opticalFlowDepthBuf_);
            glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width_, height_); // set format
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, opticalFlowDepthBuf_); // attach depth buffer to FBO

            GLenum status_of = glCheckFramebufferStatus(GL_FRAMEBUFFER);
            if(status_of != GL_FRAMEBUFFER_COMPLETE)
            {
    #if  RENDERER_DEBUG_OUTPUT
                std::printf("Renderer: Failed to set up framebuffer for optic flow\n");
    #endif
            }
            // create shader program
            shader_.reset(new Shader(FileSystem::getPath("src/implementation/sensors/event_camera/shader.vert").c_str(),
                                    FileSystem::getPath("src/implementation/sensors/event_camera/shader.frag").c_str()));

            opticFlowShader_.reset(new Shader(FileSystem::getPath("src/implementation/sensors/event_camera/shader.vert").c_str(),
                                            FileSystem::getPath("src/implementation/sensors/event_camera/optic_flow_shader.frag").c_str()));

            glEnable(GL_DEPTH_TEST);
            glEnable(GL_MULTISAMPLE);

            initialized_ = true;
        }


        // render images
        void Renderer::Render(const EventCameraTypes::Mat4_t& T_WC,
            const EventCameraTypes::Vec3_t& W_v_C,
            const EventCameraTypes::Vec3_t& W_omega_WC,
            const std::vector<EventCameraTypes::Mat4_t>& T_WK,
            const std::vector<EventCameraTypes::Vec3_t>& W_v_K,
            const std::vector<EventCameraTypes::Vec3_t>& W_omega_WK,
            std::shared_ptr<EventCameraTypes::Image_t> image,
            std::shared_ptr<EventCameraTypes::Depthmap_t> depthmap,
            std::shared_ptr<EventCameraTypes::OpticFlow_t> opticFlow)
        {
            if (!initialized_) return;

            // set context
            glfwMakeContextCurrent(window_);

            // draw to our framebuffer instead of screen
            glBindFramebuffer(GL_FRAMEBUFFER, multisampledFbo_);

            glClearColor(0.2f, 0.3f, 0.3f, 1.0f); // TODO?
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            shader_->use();

            glm::mat4 model = glm::mat4(1.0f);

            EventCameraTypes::Mat4_t T_CW = ComputeInverseTransformation(T_WC);
            // invert the z-axis here because NDC coordinates are left-handed by default in OpenGL(see https://stackoverflow.com/a/12336360)
            EventCameraTypes::Mat4_t T_CW_tilde = T_CW;
            T_CW_tilde.block<1,4>(2,0) *= -1.0;

            // view = transformation from point in world to point in camera
            glm::mat4 view = glm::make_mat4(T_CW_tilde.data());

            uint32_t modelLoc = glGetUniformLocation(shader_->ID, "model");
            glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

            uint32_t viewLoc = glGetUniformLocation(shader_->ID, "view");
            glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

            uint32_t projectionLoc = glGetUniformLocation(shader_->ID, "projection");
            glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projection_)); // TODO outside of main loop

            sceneModel_->Draw(*shader_);

            // draw dynamic objects
            for (uint8_t i = 0; i < numObjects_; i++)
            {
                model = glm::make_mat4(T_WK[i].data());

                shader_->setMat4("model", model);
                dynamicObjectModels_[i]->Draw(*shader_);
            }

            // now resolve multisampled buffer into the normal fbo
            glBindFramebuffer(GL_READ_FRAMEBUFFER, multisampledFbo_);
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo_);
            glBlitFramebuffer(0, 0, width_, height_, 0, 0, width_, height_, GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT, GL_NEAREST);

            // bind fbo back so that we read from it
            glBindFramebuffer(GL_FRAMEBUFFER, fbo_);

            /********************************/
            /*** read out grayscale image ***/
            cv::Mat imgColor(height_, width_, CV_8UC3);
            glPixelStorei(GL_PACK_ALIGNMENT, (imgColor.step & 3)?1:4);
            glPixelStorei(GL_PACK_ROW_LENGTH, imgColor.step/imgColor.elemSize());

            glReadPixels(0, 0, imgColor.cols, imgColor.rows, GL_BGR, GL_UNSIGNED_BYTE, imgColor.data);

    #if RENDERER_PUBLISH_COLOR_FRAME
            cv_bridge::CvImage cvFrameColor;
            cvFrameColor.encoding = "bgr8";
            imgColor.convertTo(cvFrameColor.image, CV_8UC3, 1.0);
            frameColorPub.publish(cvFrameColor.toImageMsg());
    #endif

            GLenum err = glGetError();
            if (err) {
    #if  RENDERER_DEBUG_OUTPUT
                printf("Renderer: Something went wrong while reading pixels: %x\n", err);
    #endif
                return;
            }
            cv::Mat imgGrayscale;
            cv::cvtColor(imgColor, imgGrayscale, cv::COLOR_RGB2GRAY);
            imgGrayscale.convertTo(*image, CV_32F, 1.f/255.f);

            /**************************/
            /*** read out depth map ***/
            cv::Mat imgDepth(height_, width_, CV_32FC1);
            glPixelStorei(GL_PACK_ALIGNMENT, (imgDepth.step & 3)?1:4);
            glPixelStorei(GL_PACK_ROW_LENGTH, imgDepth.step/imgDepth.elemSize());

            glReadPixels(0, 0, imgDepth.cols, imgDepth.rows, GL_DEPTH_COMPONENT, GL_FLOAT, imgDepth.data);

            err = glGetError();
            if (err) {
    #if  RENDERER_DEBUG_OUTPUT
                printf("Renderer: Something went wrong while reading depth data: %x\n", err);
    #endif
                return;
            }

            // convert inverse depth buffer to linear depth between minDepth and maxDepth
            // see the "Learn OpenGL" book, page 177
            cv::Mat linearDepth = (2.0 * minDepth_ * maxDepth_) / (maxDepth_ + minDepth_ - (2 * imgDepth - 1.f) * (maxDepth_ - minDepth_));
            linearDepth.copyTo(*depthmap);

            /********************************/
            /*** OPTICAL FLOW STUFF BELOW ***/
            
            // draw to our optic flow framebuffer instead of screen
            glBindFramebuffer(GL_FRAMEBUFFER, opticalFlowFbo_);

            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            opticFlowShader_->use();

            //@TODO: the vertex shaders should be shared
            // set projection matrices for vertex shader
            model = glm::mat4(1.0f);

            // view = transformation from point in world to point in camera
            view = glm::make_mat4(T_CW_tilde.data());

            modelLoc = glGetUniformLocation(opticFlowShader_->ID, "model");
            glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

            viewLoc = glGetUniformLocation(opticFlowShader_->ID, "view");
            glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

            projectionLoc = glGetUniformLocation(opticFlowShader_->ID, "projection");
            glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projection_));

            // compute optic flow and draw it to the screen buffer
            EventCameraTypes::Vec3_t C_v_C = T_CW.block<3,3>(0,0)*W_v_C;
            glm::vec3 v_C = glm::make_vec3(C_v_C.data());
            opticFlowShader_->setVec3("C_v_C", v_C);

            EventCameraTypes::Vec3_t C_omega_WC = T_CW.block<3,3>(0,0)*W_omega_WC;
            glm::vec3 omega_WC = glm::make_vec3(C_omega_WC.data());
            opticFlowShader_->setVec3("C_omega_WC", omega_WC);

            opticFlowShader_->setBool("dynamic_object", false);

            opticFlowShader_->setFloat("fx", (float) fx_);
            opticFlowShader_->setFloat("fy", (float) fy_);
            opticFlowShader_->setFloat("u0", (float) u0_);
            opticFlowShader_->setFloat("v0", (float) v0_);
            opticFlowShader_->setFloat("width", (float) width_);
            opticFlowShader_->setFloat("height", (float) height_);

            opticFlowShader_->setFloat("near", (float) minDepth_);
            opticFlowShader_->setFloat("far", (float) maxDepth_);

            sceneModel_->Draw(*opticFlowShader_);

            // draw optical flow for dynamic objects
            for (size_t i = 0; i < numObjects_; i++)
            {
                opticFlowShader_->setBool("dynamic_object", true);

                // relative position (in camera frame)
                EventCameraTypes::Vec3_t C_r_CK = (T_CW*T_WK[i]).block<3,1>(0,3);
                glm::vec3 r_CK = glm::make_vec3(C_r_CK.data());
                opticFlowShader_->setVec3("C_r_CK", r_CK);

                // linear velocity (in camera frame)
                EventCameraTypes::Vec3_t C_v_K = T_CW.block<3,3>(0,0)*W_v_K[i];
                glm::vec3 v_K = glm::make_vec3(C_v_K.data());
                opticFlowShader_->setVec3("C_v_K", v_K);

                // angular velocity (in camera frame)
                EventCameraTypes::Vec3_t C_omega_WK = T_CW.block<3,3>(0,0)*W_omega_WK[i];
                glm::vec3 omega_WK = glm::make_vec3(C_omega_WK.data());
                opticFlowShader_->setVec3("C_omega_WK", omega_WK);

                // set absolute position of dynamic object
                model = glm::make_mat4(T_WK[i].data());
                opticFlowShader_->setMat4("model", model);

                dynamicObjectModels_[i]->Draw(*opticFlowShader_);
            }

            // read out the optic flow we just rendered
            cv::Mat flow(height_, width_, CV_32FC3);
            glPixelStorei(GL_PACK_ALIGNMENT, (flow.step & 3)?1:4);
            glPixelStorei(GL_PACK_ROW_LENGTH, flow.step/flow.elemSize());

            glReadPixels(0, 0, flow.cols, flow.rows, GL_BGR, GL_FLOAT, flow.data);

            err = glGetError();
            if (err)
            {
    #if  RENDERER_DEBUG_OUTPUT
                std::printf("Renderer: Something went wrong while reading pixels: %x\n", err);
    #endif
                return;
            }

            for(int y=0; y<height_; y++)
            {
                for(int x=0; x<width_; x++)
                {
                    (*opticFlow)(y,x) = cv::Vec<float,2>(flow.at<cv::Vec<float,3>>(y,x)[2], flow.at<cv::Vec<float,3>>(y,x)[1]);
                }
            }
        }


        // public set functions
        void Renderer::SetCameraParameters(double fx, double fy, double u0, double v0,
                                        int32_t width, uint32_t height)
        {
            if (initialized_)
            {
    #if  RENDERER_DEBUG_OUTPUT
                std::printf("Renderer: SetCameraParameters() was called, but Renderer was already initialized!\n");
    #endif
                return;
            }

            fx_ = fx;
            fy_ = fy;
            u0_ = u0;
            v0_ = v0;
            width_ = width;
            height_ = height;
        }

        void Renderer::SetSceneModel(const std::string &path)
        {
            // set context
            glfwMakeContextCurrent(window_);

            // set scene
            sceneModel_.reset(new Model(path));
        }

        void Renderer::AddObjectModel(const std::string &path)
        {
            // set context
            glfwMakeContextCurrent(window_);

            // add object
            dynamicObjectModels_.push_back(std::unique_ptr<Model>(new Model(path)));
            numObjects_++;
        }


        // auxiliary functions
        glm::mat4 Renderer::ComputeOpenGLProjection(double fx, double fy, double u0, double v0,
            uint32_t width, uint32_t height, double minDepth, double maxDepth) const
        {
            // parameters that define the final viewport that is rendered into by the camera
            double L = 0.0, R = (double) width, B = 0.0, T = (double) height;

            // near and far clipping planes, these only matter for the mapping from world-space z-coordinate into the depth coordinate for OpenGL
            double N = minDepth;
            double F = maxDepth;

            // construct an orthographic matrix, which maps from projectedn coordinates to normalized device coordinates in the range [-1, 1].
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
            return glm::make_mat4(frustum.data());
        }

        EventCameraTypes::Mat4_t Renderer::ComputeInverseTransformation(const EventCameraTypes::Mat4_t& T)
        {
            EventCameraTypes::Mat4_t TInv = EventCameraTypes::Mat4_t::Zero();
            TInv.block<3,3>(0,0) = T.block<3,3>(0,0).transpose();
            TInv.block<3,1>(0,3) = -TInv.block<3,3>(0,0)*T.block<3,1>(0,3);
            TInv(3,3) = 1.0;

            return TInv;
        }

    } // namespace Simulator
} // namespace RPGQ