#pragma once
#include <chrono>
#include <cmath>

namespace so101_time {
// Optional wall-clock admission for each physics step. Simulation time and
// sensor rates remain unchanged. A delayed step never accumulates catch-up debt.
template<class Clock=std::chrono::steady_clock>
class WallStepPacer {
public:
    using Duration=typename Clock::duration;
    using TimePoint=typename Clock::time_point;

    bool Configure(double simulation_step_s,double speed_factor)
    {
        period_=Duration::zero(); started_=false;
        if (!std::isfinite(simulation_step_s) || simulation_step_s<=0. ||
            !std::isfinite(speed_factor) || speed_factor<=0.) return false;
        const double seconds=simulation_step_s/speed_factor;
        // Avoid an overflowing duration conversion or time-point addition.
        const double maximum=std::chrono::duration<double>(Duration::max()/2).count();
        if (!std::isfinite(seconds) || seconds<=0. || seconds>=maximum) return false;
        const auto requested=std::chrono::duration<double>(seconds);
        period_=std::chrono::duration_cast<Duration>(requested);
        // Round UP: duration truncation must not admit a step earlier.
        if (std::chrono::duration<double>(period_)<requested) period_+=Duration(1);
        return period_>Duration::zero();
    }

    template<class Now,class SleepUntil>
    void Pace(Now now,SleepUntil sleep_until)
    {
        if (period_==Duration::zero()) return;
        auto current=now();
        if (started_) {
            const auto earliest=previous_+period_;
            while (current<earliest) {
                sleep_until(earliest);
                current=now();
            }
        }
        // Use ACTUAL time even after long work / oversleep. Never advance an
        // old deadline by one period, which would release a burst after a stall.
        previous_=current; started_=true;
    }
    Duration period() const { return period_; }
private:
    Duration period_{Duration::zero()};
    TimePoint previous_{};
    bool started_{false};
};
}
