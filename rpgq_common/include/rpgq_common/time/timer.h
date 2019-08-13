#pragma once

// rpgq rpgq_common
#include <rpgq_common/time/base_timer.h>

// standard library
#include <chrono>
#include <limits>

namespace RPGQ
{
    class Timer : public BaseTimer
    {
    public:
        // constructor & destructor
        Timer(bool autostart = true)
        {
            if (autostart)
            {
                Reset();
            }
            else
            {
                init_ = false;
            }
        }

        // timer functions
        virtual double ElapsedSeconds(void)
        {
            using namespace std::chrono;

            if (!init_) return std::numeric_limits<double>::max();

            return USECS_TO_SECS(duration_cast<microseconds>(system_clock::now() - timer_start_).count());
        }

        virtual USecs ElapsedUSeconds(void)
        {
            using namespace std::chrono;

            if (!init_) return std::numeric_limits<USecs>::max();

            return (USecs) duration_cast<microseconds>(system_clock::now() - timer_start_).count();
        }

        virtual void Reset(void)
        {
            using namespace std::chrono;

            timer_start_ = system_clock::now();
            init_ = true;
        }


        // public get functions
        bool IsInit(void) {return init_;};

    protected:
        // general timer variables
        volatile bool init_;
        std::chrono::system_clock::time_point timer_start_;
    };

} // namespace RPGQ