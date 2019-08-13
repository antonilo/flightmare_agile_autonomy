#pragma once

// standard library
#include <cstdint>

#define SECS_TO_USECS(seconds)          ((USecs)((seconds)*1e6))
#define USECS_TO_SECS(microseconds)     ((double)(microseconds)*1e-6)
#define ROSTIME_TO_USECS(rostime)       ((USecs) (1e6*rostime.sec) + (USecs) (rostime.nsec/1000))

namespace RPGQ
{
    typedef uint64_t USecs;

    class BaseTimer
    {
    public:
        // constructor & destructor
        BaseTimer(void) {};
        ~BaseTimer(void) {};

        // timer functions
        virtual double ElapsedSeconds(void) = 0;
        virtual USecs ElapsedUSeconds(void) = 0;

    private:
        // none
    };

} // namespace RPGQ