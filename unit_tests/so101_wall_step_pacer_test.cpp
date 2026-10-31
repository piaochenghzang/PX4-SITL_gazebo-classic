#include <so101/wall_step_pacer.h>
#include <chrono>
#include <iostream>
#include <limits>
#include <vector>

namespace {
using Clock=std::chrono::steady_clock;
using Duration=Clock::duration;
using TimePoint=Clock::time_point;
using Pacer=so101_time::WallStepPacer<>;
unsigned checks=0;
bool check(bool value,const char *label)
{
    ++checks;
    if (!value) std::cerr << "FAIL: " << label << '\n';
    return value;
}
struct FakeWall {
    TimePoint time{};
    Duration oversleep{};
    bool early_once{false};
    unsigned reads{0};
    std::vector<TimePoint> deadlines;
    void Step(Pacer &pacer)
    {
        pacer.Pace([this] { ++reads; return time; },[this](TimePoint deadline) {
            deadlines.push_back(deadline);
            if (early_once) { time=deadline-Duration(1); early_once=false; }
            else time=deadline+oversleep;
        });
    }
};
}
int main()
{
    bool passed=true;
    Pacer disabled; FakeWall idle;
    idle.Step(disabled);
    passed &= check(idle.reads==0 && idle.deadlines.empty(),"disabled pacing has no clock/sleep side effects");

    Pacer regular; FakeWall wall;
    passed &= check(regular.Configure(.001,.25),"valid simulation step and speed configure");
    const auto period=regular.period();
    passed &= check(std::chrono::duration<double>(period).count()>=.004 &&
        std::chrono::duration<double>(period).count()<.0040000011,"wall period is physics step divided by speed");
    wall.Step(regular);
    passed &= check(wall.deadlines.empty(),"first step establishes an actual-time baseline");
    wall.Step(regular);
    passed &= check(wall.time==TimePoint{}+period && wall.deadlines.size()==1,"second immediate step waits one full period");
    wall.Step(regular);
    passed &= check(wall.time==TimePoint{}+period*2,"successive steps cannot run faster than the configured factor");

    wall.time+=std::chrono::seconds(1);
    const auto after_stall=wall.time; const auto sleeps=wall.deadlines.size();
    wall.Step(regular);
    passed &= check(wall.time==after_stall && wall.deadlines.size()==sleeps,"a long work stall needs no extra delay");
    wall.Step(regular);
    passed &= check(wall.time==after_stall+period && wall.deadlines.back()==after_stall+period,
        "the step AFTER a stall waits a full period instead of catching up");
    for (unsigned i=0;i<20;++i) {
        const auto before=wall.time; wall.Step(regular);
        passed &= check(wall.time-before==period,"no following burst repays missed deadlines");
    }

    Pacer overslept; FakeWall late;
    passed &= check(overslept.Configure(.002,.5),"alternate physics rate preserves step/speed ratio");
    late.Step(overslept);late.oversleep=std::chrono::milliseconds(100);late.Step(overslept);
    const auto woke=late.time;late.oversleep=Duration::zero();late.Step(overslept);
    passed &= check(late.time==woke+overslept.period(),"oversleep resets the baseline to actual wake time");

    Pacer early; FakeWall interrupted;
    passed &= check(early.Configure(.001,1.),"normal speed configures");
    interrupted.Step(early);interrupted.early_once=true;interrupted.Step(early);
    passed &= check(interrupted.deadlines.size()==2 && interrupted.time==TimePoint{}+early.period(),
        "an early wake cannot admit an early step");

    passed &= check(early.Configure(.001,3.) &&
        std::chrono::duration<double>(early.period()).count()>=.001/3.,"fractional wall period rounds upward");
    const double bad[]{0.,-1.,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()};
    for (double value:bad) {
        passed &= check(!early.Configure(value,1.),"invalid physics step is rejected");
        passed &= check(!early.Configure(.001,value),"invalid speed factor is rejected");
    }
    passed &= check(!early.Configure(std::numeric_limits<double>::max(),std::numeric_limits<double>::min()),
        "overflowing wall period is rejected");
    const auto reads=interrupted.reads;interrupted.Step(early);
    passed &= check(interrupted.reads==reads,"invalid reconfiguration leaves pacing disabled");
    std::cout << checks << " wall-step pacing checks " << (passed ? "passed" : "FAILED") << '\n';
    return passed ? 0 : 1;
}
