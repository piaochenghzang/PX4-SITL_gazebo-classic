#pragma once
#include <algorithm>
#include <cmath>
#include <limits>

namespace so101_contact {
// Gazebo Classic ODE reports each body's wrench in THAT LINK's local frame.
// Select the finger's wrench, never the target's pose or world contact normal.
// This is the calibrated pad compression component, not a 3-D contact normal.
template<class JointWrench>
bool compression(const JointWrench &pair, bool target_first, unsigned side, double &out)
{
    out=std::numeric_limits<double>::quiet_NaN();
    if (side>1) return false;
    const auto &w=target_first ? pair.body_2_wrench() : pair.body_1_wrench();
    if (!w.has_force()) return false;
    const auto &f=w.force();
    if (!f.has_x() || !f.has_y() || !f.has_z() ||
        !std::isfinite(f.x()) || !std::isfinite(f.y()) || !std::isfinite(f.z())) return false;
    // Fixed pad inner face points +X, moving pad inner face points -X.
    // Force exerted ON each pad under compression points the other way.
    out=std::max(0.,side==0 ? -f.x() : f.x());
    return true;
}
}
