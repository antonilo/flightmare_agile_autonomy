#include <rpgq_simulator/implementation/sensors/event_camera.h>

// ros
#include <rpgq_msgs/EventArray.h>
#include <cv_bridge/cv_bridge.h>

// standard library
#include <algorithm>
#include <cmath>

// others
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace RPGQ
{
    namespace Simulator
    {
        // function declaration
        void MakeColorwheel(int (*colorwheel)[3]);

        // constructor & destructor
        EventCamera::EventCamera(SensorID id, const Node* prevSimNode, USecs maxSimUSecsInterval):
            BaseSensor(id, prevSimNode, maxSimUSecsInterval, true)
        {
            // general event camera variables
            initialized_ = false;
            sceneInitialized_ = false;
            numObjects_ = 0;
            sensorNoise_ = std::normal_distribution<EventCameraTypes::Intensity_t>(0.f,1.f);

            // event camera parameters
            gaussianBlurSigma_ = 0.0;
            adaptiveSamplingLambda_ = 0.5;
            useLogImage_ = true;
            usePredictedBrightnessChange_ = true;
            logEps_ = 0.001f;
            intensityTol_ = 1e-6f;
            Cp_ = 0.5f;
            Cn_ = 0.5f;
            sigmaCp_ = 0.0*0.02f;
            sigmaCn_ = 0.0*0.02f;
            dusecsFrame_ = (USecs) (1.0/30.0*1e6);
            imageWidth_ = 240;
            imageHeight_ = 180;
            fx_ = 147.f;
            fy_ = 147.f;
            cx_ = 120.f;
            cy_ = 90.f;

            // set up renderer
            renderer_.SetCameraParameters(fx_, fy_, cx_, cy_, imageWidth_, imageHeight_);
            renderer_.Initialize();

            // set up publisher
            image_transport::ImageTransport it(pnh_);
            framePub_ = it.advertise(GetSimTreePath(simNode_) + "/frame", 1);
            depthmapPub_ = it.advertise(GetSimTreePath(simNode_) + "/depthmap", 1);
            opticFlowPub_ = it.advertise(GetSimTreePath(simNode_) + "/optic_flow", 1);
            eventsPub_ = pnh_.advertise<rpgq_msgs::EventArray>(GetSimTreePath(simNode_) + "/events", 0);
            cameraInfoPub_ = pnh_.advertise<sensor_msgs::CameraInfo>(GetSimTreePath(simNode_) + "/camera_info", 1);

            const cv::Size size = cv::Size(imageWidth_,imageHeight_);
            image_.reset(new EventCameraTypes::Image_t(size));
            depthmap_.reset(new EventCameraTypes::Depthmap_t(size));
            opticFlow_.reset(new EventCameraTypes::OpticFlow_t(size));

            // relative pose
            B_r_BC_.setZero();
            T_BC_.setZero();
            T_BC_.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
            T_BC_.block<3,1>(0,3) = B_r_BC_;
            T_BC_(3,3) = 1.0;

            // simulation timing variables
            usecsNextTFrame_ = maxSimUSecsInterval;
            usecsNextTFlow_ = maxSimUSecsInterval;
            usecsCameraInfoInterval_ = 500000; // 500ms, 2Hz
            usecsNextCameraInfo_ = usecsCameraInfoInterval_;

            // auxiliary variables
            MakeColorwheel(colorwheel_);
        }

        EventCamera::~EventCamera()
        {
            framePub_.shutdown();
            depthmapPub_.shutdown();
            opticFlowPub_.shutdown();
        }


        // add objects
        void EventCamera::AddObjectModelAndCallbacks(const std::string &path,
                                                    EventCameraTypes::GetPos_t GetPos,
                                                    EventCameraTypes::GetVel_t GetVel,
                                                    EventCameraTypes::GetQuat_t GetQuat,
                                                    EventCameraTypes::GetOmega_t GetOmega)
        {
            renderer_.AddObjectModel(path);

            GetPosObj_.push_back(GetPos);
            GetVelObj_.push_back(GetVel);
            GetQuatObj_.push_back(GetQuat);
            GetOmegaObj_.push_back(GetOmega);

            numObjects_++;
        };


        // simulate sensor
        void EventCamera::RunSimulation_(void)
        {
            if (!sceneInitialized_)
            {
                std::printf("%s: Scene is not set! Cannot simulate event camera.\n", GetSimTreePath(simNode_).c_str());
                return;
            }

            // compute pose of camera
            EventCameraTypes::Mat4_t T_WB;
            T_WB.setZero();
            T_WB.block<3,3>(0,0) = GetQuat_().toRotationMatrix();
            T_WB.block<3,1>(0,3) = GetPos_();
            T_WB(3,3) = 1.0;
            EventCameraTypes::Mat4_t T_WC = T_WB*T_BC_;

            EventCameraTypes::Vec3_t B_omega_WC = GetOmega_();
            EventCameraTypes::Vec3_t W_omega_WC = T_WB.block<3,3>(0,0)*B_omega_WC;
            EventCameraTypes::Vec3_t W_v_C = GetVel_() +  T_WB.block<3,3>(0,0)*(B_omega_WC.cross(B_r_BC_)); // TODO: check if everything is correct

            // compute pose of objects
            std::vector<EventCameraTypes::Mat4_t> T_WKs;
            std::vector<EventCameraTypes::Vec3_t> W_v_Ks;
            std::vector<EventCameraTypes::Vec3_t> W_omega_WKs;
            for (uint8_t i = 0; i < numObjects_; i++)
            {
                EventCameraTypes::Mat4_t T_WK;
                T_WK.setZero();
                T_WK.block<3,3>(0,0) = (GetQuatObj_[i]()).toRotationMatrix();
                T_WK.block<3,1>(0,3) = GetPosObj_[i]();
                T_WK(3,3) = 1.0;

                EventCameraTypes::Vec3_t K_omega_WK = GetOmegaObj_[i]();
                EventCameraTypes::Vec3_t W_omega_WK = T_WK.block<3,3>(0,0)*K_omega_WK;
                EventCameraTypes::Vec3_t W_v_K = GetVelObj_[i]();

                T_WKs.push_back(T_WK);
                W_v_Ks.push_back(W_v_K);
                W_omega_WKs.push_back(W_omega_WK);
            }

            // render image(s)
            renderer_.Render(T_WC, W_v_C, W_omega_WC,        // camera pose and velocity
                            T_WKs, W_v_Ks, W_omega_WKs,     // object poses and velocities,
                            image_, depthmap_, opticFlow_); // output images

            // blur image
            if (gaussianBlurSigma_ > 0.0)
            {
                GaussianBlur(*image_, gaussianBlurSigma_);
            }

            // compute events
            EventCameraTypes::Events events = ComputeEvents(*image_, timer_.ElapsedUSeconds());

            // publish data
            if (timer_.ElapsedUSeconds() >= usecsNextCameraInfo_)
            {
                PublishCameraInfo();
                usecsNextCameraInfo_ = timer_.ElapsedUSeconds() + usecsCameraInfoInterval_;
            }
            if (timer_.ElapsedUSeconds() == usecsNextTFrame_)
            {
                PublishFrame(*image_);
                //PublishDepthmap(*depthmap_);
                //PublishOpticFlow(*opticFlow_);
            }
            PublishEvents(events);
        }

        USecs EventCamera::UpdateSamplingInterval(void)
        {
            // compute next sampling instance due to frame rate
            if (timer_.ElapsedUSeconds() == usecsNextTFrame_)
            {
                usecsNextTFrame_ += dusecsFrame_;
            }

            /*if (usePredictedBrightnessChange_)
            {
                EventCameraTypes::Intensity_t maxNegBrightnessChange, maxPosBrightnessChange;
                std::tie(maxNegBrightnessChange,maxPosBrightnessChange) = ComputeMaximumPredictedAbsBrightnessChange(*opticFlow_, previousImage_);
                double dsecsBrightnessChange = std::min(Cn_/(-maxNegBrightnessChange), Cp_/maxPosBrightnessChange);

                usecsNextTFlow_ = timer_.ElapsedUSeconds() + SECS_TO_USECS(dsecsBrightnessChange);
                //std::printf("step size: %f\n", dsecsBrightnessChange);
            }
            else
            {
                // compute next sampling instance due to optic flow
                double maxFlowMagnitude = ComputeMaximumOpticFlowMagnitude(*opticFlow_);
                double dsecsOpticFlow = adaptiveSamplingLambda_ / maxFlowMagnitude;

                if (std::isinf(dsecsOpticFlow))
                {
                    usecsNextTFlow_ = timer_.ElapsedUSeconds() + maxSimUSecsInterval_;
                }
                else
                {
                    usecsNextTFlow_ = timer_.ElapsedUSeconds() + SECS_TO_USECS(dsecsOpticFlow);
                }
            }*/

            /*EventCameraTypes::Intensity_t maxNegBrightnessChange, maxPosBrightnessChange;
            std::tie(maxNegBrightnessChange,maxPosBrightnessChange) = ComputeMaximumPredictedAbsBrightnessChange(*opticFlow_, previousImage_);
            double dsecsBrightnessChange = adaptiveSamplingLambda_ * std::min(Cn_/(-maxNegBrightnessChange), Cp_/maxPosBrightnessChange);*/

            // compute next sampling instance due to optic flow
            double maxFlowMagnitude = ComputeMaximumOpticFlowMagnitude(*opticFlow_);
            double dsecsOpticFlow = adaptiveSamplingLambda_ / maxFlowMagnitude;
            if (std::isinf(dsecsOpticFlow)) dsecsOpticFlow = USECS_TO_SECS(maxSimUSecsInterval_);

            usecsNextTFlow_ = timer_.ElapsedUSeconds() + SECS_TO_USECS(dsecsOpticFlow);//std::max(dsecsBrightnessChange, dsecsOpticFlow));

            // compute earlier sampling instance (either due to flow or frame rate)
            return std::min(usecsNextTFlow_ - timer_.ElapsedUSeconds(), usecsNextTFrame_ - timer_.ElapsedUSeconds());
        }


        // publishers
        void EventCamera::PublishFrame(const EventCameraTypes::Image_t& image)
        {
            cv_bridge::CvImage cvFrame;
            cvFrame.header.stamp.fromNSec(1000*timer_.ElapsedUSeconds());
            cvFrame.encoding = "mono8";
            //cv::normalize(image, cvFrame.image, 0.0, 255.0, cv::NORM_MINMAX, CV_8U);
            image.convertTo(cvFrame.image, CV_8U, 255.0);
            framePub_.publish(cvFrame.toImageMsg());
        }

        void EventCamera::PublishDepthmap(const EventCameraTypes::Depthmap_t& depthmap)
        {
            cv_bridge::CvImage cvDepthmap;
            cvDepthmap.header.stamp.fromNSec(1000*timer_.ElapsedUSeconds());
            cvDepthmap.encoding = "mono8";
            cv::Mat_<float> depthmapScaled; 
            cv::threshold(depthmap, depthmapScaled, 20.0, 20.0, cv::THRESH_TRUNC); // TODO, scaling
            depthmapScaled.convertTo(cvDepthmap.image, CV_8U, 255.0/20.0);
            //cv::normalize(cvDepthmap.image, cvDepthmap.image, 0.0, 255.0, cv::NORM_MINMAX, CV_8U);
            depthmapPub_.publish(cvDepthmap.toImageMsg());
        }

        void EventCamera::PublishOpticFlow(const EventCameraTypes::OpticFlow_t& opticFlow)
        {
            cv_bridge::CvImage cvOpticFlow;
            cvOpticFlow.header.stamp.fromNSec(1000*timer_.ElapsedUSeconds());
            cvOpticFlow.encoding = "bgr8";

            cv::Mat_<float> opticFlowX, opticFlowY, channels[2];
            split(opticFlow, channels);
            opticFlowX = channels[0];
            opticFlowY = channels[1];

            cv::Mat_<float> magnitude, angle;
            cv::cartToPolar(opticFlowX, opticFlowY, magnitude, angle);
            cv::threshold(magnitude, magnitude, 200.0, 200.0, cv::THRESH_TRUNC); // TODO, scaling
            //cv::normalize(magnitude, magnitude, 0.0, 255.0, cv::NORM_MINMAX, CV_32F);
            cv::Size s = opticFlow_->size();
            cv::Mat hsv(s.height, s.width, CV_8UC3, cv::Scalar(0,255,0));
            cv::Mat rgb(s.height, s.width, CV_8UC3, cv::Scalar(0,0,0));
            for (uint32_t x = 0; x < s.width; x++)
            {
                for (uint32_t y = 0; y < s.height; y++)
                {
                    hsv.at<cv::Vec3b>(y,x)[0] = 0.5*angle.at<float>(y,x)*180.0/M_PI;
                    hsv.at<cv::Vec3b>(y,x)[2] = magnitude.at<float>(y,x)*255.0/200.0; // TODO, scaling

                    float fk = (angle.at<float>(y,x)/M_PI)/2.0 * 54.0; // angle is between 0 and 2*pi
                    int k0 = (int) fk;
                    int k1 = (k0 + 1) % 54;
                    float f = fk - k0;
                    for (int b = 0; b < 3; b++)
                    {
                        float col0 = colorwheel_[k0][b] / 255.0;
                        float col1 = colorwheel_[k1][b] / 255.0;
                        float col = (1-f)*col0 + f*col1;
                        float rad = magnitude.at<float>(y,x)/200.0;
                        if (rad <= 1.f) 
                        {
                            col = 1.f - rad*(1.f - col); // increase saturation with radius
                        }
                        else
                        {
                            col *= 0.75f;
                        }
                        rgb.at<cv::Vec3b>(y,x)[b] = (int) (255.f*col);
                    }
                }
            }
            cv::cvtColor(hsv, cvOpticFlow.image, cv::COLOR_HSV2BGR);
            cv::cvtColor(rgb, cvOpticFlow.image, cv::COLOR_RGB2BGR);

            opticFlowPub_.publish(cvOpticFlow.toImageMsg());
        }

        void EventCamera::PublishEvents(const EventCameraTypes::Events& events)
        {
            if (events.empty()) return;
            
            rpgq_msgs::EventArrayPtr msg;
            msg.reset(new rpgq_msgs::EventArray);

            msg->height = imageSize_.height;
            msg->width = imageSize_.width;
            for (const auto & event : events)
            {
                rpgq_msgs::Event e;
                e.x = event.x;
                e.y = event.y;
                e.ts.fromNSec(event.t);
                e.polarity = event.pol;
                msg->events.push_back(e);
            }
            msg->header.stamp = msg->events.back().ts;

            eventsPub_.publish(msg);
        }

        void EventCamera::PublishCameraInfo(void)
        {
            sensor_msgs::CameraInfo camInfoMsg;
            camInfoMsg.header.stamp.fromNSec(1000*timer_.ElapsedUSeconds());

            camInfoMsg.width = imageWidth_;
            camInfoMsg.height = imageHeight_;
            
            camInfoMsg.distortion_model = "plumb_bob";
            camInfoMsg.D = {0.0, 0.0, 0.0, 0.0, 0.0};

            camInfoMsg.K[0] = fx_; camInfoMsg.K[1] = 0.f; camInfoMsg.K[2] = cx_;
            camInfoMsg.K[3] = 0.f; camInfoMsg.K[4] = fy_; camInfoMsg.K[5] = cy_;
            camInfoMsg.K[6] = 0.f; camInfoMsg.K[7] = 0.f; camInfoMsg.K[8] = 1.f;

            cameraInfoPub_.publish(camInfoMsg);
        }


        // auxiliary functions
        void EventCamera::GaussianBlur(EventCameraTypes::Image_t& image, double sigma)
        {
            cv::GaussianBlur(image, image, cv::Size(15,15), sigma, sigma);
        }

        std::pair<EventCameraTypes::Intensity_t,EventCameraTypes::Intensity_t> EventCamera::ComputeMaximumPredictedAbsBrightnessChange(
                                                                                    const EventCameraTypes::OpticFlow_t &opticFlow,
                                                                                    const EventCameraTypes::Image_t& image)
        {
            // "image" is assumed to be a log intensity image

            // compute horizontal/vertical gradient
            EventCameraTypes::Image_t dLogIdx, dLogIdy; 
            cv::Sobel(image, dLogIdx, cv::DataType<EventCameraTypes::Intensity_t>::type, 1, 0, 3, 0.25); // 0.25 accounts for scaling introduced by sobel filter mask
            cv::Sobel(image, dLogIdy, cv::DataType<EventCameraTypes::Intensity_t>::type, 0, 1, 3, 0.25);

            // compute gradient w.r.t time
            EventCameraTypes::Intensity_t maxNegBrightnessChange = 0.f, maxPosBrightnessChange = 0.f;
            for (uint32_t y = 0; y < imageSize_.height; y++)
            {
                for (uint32_t x = 0; x < imageSize_.width; x++)
                {
                    // d(LogI)/dt ≃ <nabla LogI, flow>
                    double brightnessChange = opticFlow(y,x)[0]*dLogIdx(y,x) + opticFlow(y,x)[1]*dLogIdy(y,x);

                    if (brightnessChange < maxNegBrightnessChange)
                    {
                        maxNegBrightnessChange = brightnessChange;
                    }
                    else if (brightnessChange > maxPosBrightnessChange)
                    {
                        maxPosBrightnessChange = brightnessChange;
                    }
                }
            }

            return std::make_pair(maxNegBrightnessChange,maxPosBrightnessChange);
        }

        double EventCamera::ComputeMaximumOpticFlowMagnitude(const EventCameraTypes::OpticFlow_t& opticFlow) const
        {
            double maxSquaredMagnitude = 0.0;
            for (int y = 0; y < opticFlow.rows; y++)
            {
                for (int x = 0; x < opticFlow.cols; x++)
                {
                    double squaredMagnitude = cv::norm(opticFlow(y,x), cv::NORM_L2SQR);
                    if (squaredMagnitude > maxSquaredMagnitude) maxSquaredMagnitude = squaredMagnitude;
                }
            }

            return sqrt(maxSquaredMagnitude);
        }

        EventCameraTypes::Events EventCamera::ComputeEvents(const EventCameraTypes::Image_t& image, const USecs t)
        {
            EventCameraTypes::Image_t preprocessedImage = image.clone();
            if (useLogImage_)
            {
                cv::log(logEps_ + image, preprocessedImage);
            }

            if (!initialized_)
            {
                initialized_ = true;
                previousImage_ = preprocessedImage.clone();
                triggerImage_ = preprocessedImage.clone();
                imageSize_ = preprocessedImage.size();

                previousImageTime_ = t;
                return {};
            }

            // for each pixel, check if new events need to be generated since the previous image
            EventCameraTypes::Events events;
            EventCameraTypes::Intensity_t dt = (EventCameraTypes::Intensity_t) (t - previousImageTime_);
            for (int y = 0; y < imageSize_.height; y++)
            {
                for (int x = 0; x < imageSize_.width; x++)
                {
                    EventCameraTypes::Intensity_t I = preprocessedImage(y,x);
                    EventCameraTypes::Intensity_t prevI = previousImage_(y,x);

                    if (std::fabs(I - prevI) > intensityTol_)
                    {
                        bool pol = (I > prevI) ? true : false;
                        EventCameraTypes::Intensity_t C = pol ? Cp_ : -Cn_;
                        EventCameraTypes::Intensity_t sigmaC = pol ? sigmaCp_ : sigmaCn_;

                        // find all trigger instances
                        EventCameraTypes::Intensity_t currI = triggerImage_(y,x);
                        while (1)
                        {
                            currI += C;

                            if (sigmaC > 0.f)
                            {
                                currI += sigmaC*sensorNoise_(gen_);
                            }

                            if ((pol && prevI < currI && currI <= I) || (!pol && prevI > currI && currI >= I))
                            {
                                uint64_t et = 1000*t + (uint64_t) (1000.f*(currI - prevI)/(I - prevI)*dt); // convert from usec to nsec!
                                events.emplace_back(x, y, et, pol);
                                triggerImage_(y,x) = currI;
                            }
                            else
                            {
                                break;
                            }
                        }

                    }
                }
            }

            // sort events by increasing timestamps
            std::sort(events.begin(), events.end(), [](const EventCameraTypes::Event& a, const EventCameraTypes::Event& b) {return a.t > b.t;});

            // update internal event camera variables
            previousImage_ = preprocessedImage.clone();
            previousImageTime_ = t;

            return events;
        }

        // auxiliary functions for optic flow coloring
        void SetCols(int r, int g, int b, int k, int (*colorwheel)[3])
        {
            colorwheel[k][0] = r;
            colorwheel[k][1] = g;
            colorwheel[k][2] = b;
        }

        void MakeColorwheel(int (*colorwheel)[3])
        {
            // relative lengths of color transitions:
            // these are chosen based on perceptual similarity
            // (e.g. one can distinguish more shades between red and yellow 
            //  than between yellow and green)
            int RY = 15;
            int YG = 6;
            int GC = 4;
            int CB = 11;
            int BM = 13;
            int MR = 6;
            int ncols = RY + YG + GC + CB + BM + MR;
            //printf("ncols = %d\n", ncols);
            if (ncols > 55)
            exit(1);
            int i;
            int k = 0;
            for (i = 0; i < RY; i++) SetCols(255,	        255*i/RY,	    0,	            k++, colorwheel);
            for (i = 0; i < YG; i++) SetCols(255-255*i/YG,  255,		    0,	            k++, colorwheel);
            for (i = 0; i < GC; i++) SetCols(0,		        255,		    255*i/GC,       k++, colorwheel);
            for (i = 0; i < CB; i++) SetCols(0,		        255-255*i/CB,   255,	        k++, colorwheel);
            for (i = 0; i < BM; i++) SetCols(255*i/BM,	    0,		        255,	        k++, colorwheel);
            for (i = 0; i < MR; i++) SetCols(255,	        0,		        255-255*i/MR,   k++, colorwheel);
        }

    } // namespace Simulator
} //  namespace RPGQ