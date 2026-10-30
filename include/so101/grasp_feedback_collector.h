#pragma once
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <development/mavlink.h>
#include <mutex>

namespace gazebo {
// Explicitly opt-in, read-only feedback. Contact-only mode never reads poses.
// No ROS or joint actuation; the legacy bench source remains the default.
class GraspFeedbackCollector {
public:
    void Load(physics::ModelPtr model, const std::string &target, const std::string &support,
        bool contact_only=false);
    bool Sample(mavlink_arm_grasp_feedback_t &out);
private:
    void Contacts(ConstContactsPtr &message);
    bool Resolve();
    void ClearWindow();
    bool BenchValid() const;
    physics::ModelPtr model_, target_, support_;
    physics::LinkPtr gripper_, jaw_, object_, support_link_;
    physics::CollisionPtr fixed_, moving_, body_, support_collision_, grasp_surface_;
    transport::NodePtr node_;
    transport::SubscriberPtr subscriber_;
    std::string target_name_, support_name_;
    std::mutex mutex_;
    double last_publish_{0}, last_contact_{-1}, pose_stamp_{-1};
    ignition::math::Quaterniond object_rotation_;
    uint32_t sequence_{0}, epoch_{0};
    uint32_t target_instance_{0}, support_instance_{0};
    bool instance_latched_{false}, instance_changed_{false};
    unsigned frames_{0}, fixed_frames_{0}, moving_frames_{0}, both_frames_{0};
    unsigned support_frames_{0}, other_frames_{0}, support_arm_frames_{0};
    double force_[2]{}, vertical_{0};
    bool force_valid_{true}, contact_only_{false};
    unsigned force_wrenches_{0};
};
}
