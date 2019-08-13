#pragma once

// rpgq rpgq_common
#include <rpgq_common/time/timer.h>

// standard library
#include <memory>

namespace RPGQ
{
    class ExtTimer : public Timer
    {
    public:
        // constructor & destructor
        ExtTimer(bool autostart = true, bool freeRunning = true):
            Timer(autostart)
        {
            masterTimer_ = nullptr;
            startTimeUsecs_ = 0;
            currentTimeUsecs_ = 0;
            freeRunning_ = freeRunning;

            if (autostart)
            {
                Reset();
            }
            else
            {
                init_ = false;
            }
        }
        ExtTimer(std::shared_ptr<Timer> timer, bool autostart = true):
            Timer(autostart)
        {
            masterTimer_ = timer;
            freeRunning_ = false;

            startTimeUsecs_ = masterTimer_->ElapsedUSeconds();
            currentTimeUsecs_ = 0;

            if (autostart)
            {
                Reset();
            }
            else
            {
                init_ = false;
            }
        }
        ~ExtTimer(void) {}

        // timer functions
        double ElapsedSeconds(void)
        {
            if (masterTimer_)
            {
                return USECS_TO_SECS(masterTimer_->ElapsedUSeconds() - startTimeUsecs_);
            }
            else if (!freeRunning_)
            {
                return USECS_TO_SECS(currentTimeUsecs_ - startTimeUsecs_);
            }
            else
            {
                return Timer::ElapsedSeconds();
            }
        }

        USecs ElapsedUSeconds(void)
        {
            if (masterTimer_)
            {
                return masterTimer_->ElapsedUSeconds() - startTimeUsecs_;
            }
            else if (!freeRunning_)
            {
                return currentTimeUsecs_ - startTimeUsecs_;
            }
            else
            {
                return Timer::ElapsedUSeconds();
            }
        }

        void AdvanceSeconds(double dt)
        {
            if (masterTimer_ || freeRunning_)
            {
                std::printf("ExtTimer ERROR: Slaved or free running timer cannot be manually advanced.\n");
                return;
            }

            currentTimeUsecs_ += SECS_TO_USECS(dt);
        }

        void AdvanceUSeconds(USecs dt)
        {
            if (masterTimer_ || freeRunning_)
            {
                std::printf("ExtTimer ERROR: Slaved or free running timer cannot be manually advanced.\n");
                return;
            }

            currentTimeUsecs_ += dt;
        }

        void Reset(void)
        {
            if (masterTimer_)
            {
                startTimeUsecs_ = masterTimer_->ElapsedUSeconds();
            }
            else if (!freeRunning_)
            {
                startTimeUsecs_ = currentTimeUsecs_;
            }
            else
            {
                Timer::Reset();
            }
            init_ = true;
        }

    private:
        std::shared_ptr<Timer> masterTimer_;
        volatile USecs startTimeUsecs_;
        volatile USecs currentTimeUsecs_;

        volatile bool freeRunning_;
    };
} // namespace RPQG