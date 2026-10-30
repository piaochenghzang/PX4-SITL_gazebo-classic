#include <so101/grasp_feedback_collector.h>
#include <so101/contact_force.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <gazebo/physics/ode/ODEPhysics.hh>

namespace gazebo {
namespace {
bool prefix(const std::string &value, const std::string &root) { return value.compare(0, root.size(), root) == 0; }
bool finitePose(const ignition::math::Pose3d &pose)
{
    const auto &q=pose.Rot();
    return pose.Pos().IsFinite() && q.IsFinite()
        && std::abs(q.W()*q.W()+q.X()*q.X()+q.Y()*q.Y()+q.Z()*q.Z()-1.) < .01;
}
bool boxBounds(physics::CollisionPtr collision, ignition::math::Vector3d &low, ignition::math::Vector3d &high)
{
    if (!collision) return false;
    const auto box = boost::dynamic_pointer_cast<physics::BoxShape>(collision->GetShape());
    const auto pose = collision->WorldPose();
    if (!box || !finitePose(pose)) return false;
    const auto half = box->Size() * .5;
    ignition::math::Vector3d extent;
    for (int axis = 0; axis < 3; ++axis) {
        ignition::math::Vector3d direction; direction[axis] = half[axis];
        const auto rotated = pose.Rot().RotateVector(direction);
        for (int j = 0; j < 3; ++j) extent[j] += std::abs(rotated[j]);
    }
    low = pose.Pos() - extent; high = pose.Pos() + extent;
    return true;
}
}
void GraspFeedbackCollector::Load(physics::ModelPtr model, const std::string &target, const std::string &support,
    bool contact_only)
{
    model_ = model; target_name_ = target; support_name_ = support; contact_only_=contact_only;
    node_.reset(new transport::Node()); node_->Init(model_->GetWorld()->Name());
    subscriber_ = node_->Subscribe("~/physics/contacts", &GraspFeedbackCollector::Contacts, this);
}
bool GraspFeedbackCollector::Resolve()
{
    target_ = model_->GetWorld()->ModelByName(target_name_);
    support_ = model_->GetWorld()->ModelByName(support_name_);
    if (target_ && support_) {
        if (!instance_latched_) {
            target_instance_=target_->GetId(); support_instance_=support_->GetId(); instance_latched_=true;
        } else if (target_instance_!=target_->GetId() || support_instance_!=support_->GetId()) instance_changed_=true;
    }
    gripper_ = model_->GetLink("so101::gripper_link");
    jaw_ = model_->GetLink("so101::moving_jaw_so101_v1_link");
    object_ = target_ ? target_->GetLink("link") : nullptr;
    support_link_ = support_ ? support_->GetLink("link") : nullptr;
    fixed_ = gripper_ ? gripper_->GetCollision("gripper_fixed_finger_collision") : nullptr;
    body_ = gripper_ ? gripper_->GetCollision("gripper_body_collision") : nullptr;
    moving_ = jaw_ ? jaw_->GetCollision("moving_jaw_collision") : nullptr;
    support_collision_ = support_link_ ? support_link_->GetCollision("collision") : nullptr;
    grasp_surface_ = object_ ? object_->GetCollision("collision") : nullptr;
    return gripper_ && fixed_ && moving_ && (!contact_only_ || grasp_surface_);
}
void GraspFeedbackCollector::ClearWindow()
{
    frames_ = fixed_frames_ = moving_frames_ = both_frames_ = 0;
    support_frames_ = other_frames_ = support_arm_frames_ = 0;
    force_[0] = force_[1] = vertical_ = 0; force_valid_ = true; force_wrenches_=0;
}
bool GraspFeedbackCollector::BenchValid() const
{
    if (instance_changed_) return false;
    auto engine=boost::dynamic_pointer_cast<physics::ODEPhysics>(model_->GetWorld()->Physics());
    if (!engine || engine->GetStepType()!="quick" || engine->GetSORPGSIters()!=40
        || std::abs(engine->GetMaxStepSize()-.001)>1e-9) return false;
    unsigned controller_count=0;
    auto model_sdf=model_->GetSDF();
    if (!model_sdf || !model_sdf->HasElement("plugin")) return false;
    for (auto plugin=model_sdf->GetElement("plugin"); plugin; plugin=plugin->GetNextElement("plugin")) {
        if (plugin->Get<std::string>("name")!="gripper_controller") continue;
        ++controller_count;
        if (!plugin->HasElement("p") || !plugin->HasElement("i") || !plugin->HasElement("d")
            || !plugin->HasElement("cmdMax") || std::abs(plugin->Get<double>("p")-.045)>1e-8
            || std::abs(plugin->Get<double>("i"))>1e-8 || std::abs(plugin->Get<double>("d")-.001)>1e-8
            || std::abs(plugin->Get<double>("cmdMax")-.01)>1e-8
            || !plugin->HasElement("commandTimeoutPolicy")
            || plugin->Get<std::string>("commandTimeoutPolicy")!="last_target") return false;
    }
    if (controller_count!=1) return false;
    for (const auto &pad : {fixed_,moving_}) {
        auto element=pad ? pad->GetSDF() : nullptr;
        for (const auto &name : {"surface","friction","torsional"}) {
            if (!element || !element->HasElement(name)) return false;
            element=element->GetElement(name);
        }
        if (!element->Get<bool>("use_patch_radius") || std::abs(element->Get<double>("patch_radius")-.004)>1e-8
            || std::abs(element->Get<double>("coefficient")-.8)>1e-8) return false;
    }
    auto base = model_->GetLink("so101::base_link");
    if (!base || !model_->GetJoint("bench_anchor") || !target_ || target_->IsStatic()
        || !support_ || !support_->IsStatic() || !object_ || !object_->GetInertial()) return false;
    const auto base_pose = base->WorldPose();
    if (!finitePose(base_pose) || (base_pose.Pos() - ignition::math::Vector3d(0,0,1.15)).Length() > .002
        || std::abs(base_pose.Rot().X()) < .999 || model_->WorldLinearVel().Length() > .001
        || model_->WorldAngularVel().Length() > .001 || std::abs(object_->GetInertial()->Mass()-.01) > 1e-6) return false;
    auto collision = object_->GetCollision("collision");
    auto box = collision ? boost::dynamic_pointer_cast<physics::BoxShape>(collision->GetShape()) : nullptr;
    return box && (box->Size()-ignition::math::Vector3d(.02,.02,.02)).Length() < 1e-6;
}
void GraspFeedbackCollector::Contacts(ConstContactsPtr &message)
{
    std::lock_guard<std::mutex> guard(mutex_);
    if (!message || !fixed_ || !moving_) return;
    const double stamp = message->time().sec()+message->time().nsec()*1e-9;
    if (stamp <= last_contact_) return;
    last_contact_ = stamp;
    bool fixed=false, moving=false, support=false, other=false, support_arm=false;
    const std::string target_root = target_name_+"::";
    const std::string support_root = support_name_+"::";
    const std::string vehicle_root = model_->GetName()+"::";
    for (int i=0; i<message->contact_size(); ++i) {
        const auto &c = message->contact(i);
        const bool first = prefix(c.collision1(),target_root), second = prefix(c.collision2(),target_root);
        support_arm |= (prefix(c.collision1(),support_root) && prefix(c.collision2(),vehicle_root))
            || (prefix(c.collision2(),support_root) && prefix(c.collision1(),vehicle_root));
        if (first == second) continue;
        const auto &against = first ? c.collision2() : c.collision1();
        const int side = against == fixed_->GetScopedName() ? 0 : against == moving_->GetScopedName() ? 1 : -1;
        support |= prefix(against,support_root); other |= side < 0 && prefix(against,vehicle_root);
        if (contact_only_ && side>=0 && (!grasp_surface_ ||
                (first ? c.collision1() : c.collision2())!=grasp_surface_->GetScopedName())) {
            other=true; continue; // Pads may touch the designated handle only.
        }
        fixed |= side == 0; moving |= side == 1;
        if (side < 0) continue;
        if (contact_only_) {
            if (!c.wrench_size()) { force_valid_=false; continue; }
            for (int j=0; j<c.wrench_size(); ++j) {
                double compression;
                if (!so101_contact::compression(c.wrench(j),first,unsigned(side),compression)) {
                    force_valid_=false; continue;
                }
                force_[side]+=compression; ++force_wrenches_;
            }
            continue;
        }
        if (pose_stamp_ < 0 || std::abs(stamp-pose_stamp_) > .02 || c.wrench_size() != c.normal_size() || !c.wrench_size()) {
            force_valid_ = false; continue;
        }
        for (int j=0; j<c.wrench_size(); ++j) {
            const auto &w = first ? c.wrench(j).body_1_wrench() : c.wrench(j).body_2_wrench();
            auto f = object_rotation_.RotateVector({w.force().x(),w.force().y(),w.force().z()});
            ignition::math::Vector3d n(c.normal(j).x(),c.normal(j).y(),c.normal(j).z());
            if (!f.IsFinite() || !n.IsFinite() || std::abs(n.Length()-1.) > .01) { force_valid_=false; continue; }
            force_[side] += std::abs(f.Dot(n)); vertical_ += f.Z();
        }
    }
    ++frames_; fixed_frames_ += fixed; moving_frames_ += moving; both_frames_ += fixed && moving;
    support_frames_ += support; other_frames_ += other; support_arm_frames_ += support_arm;
}
bool GraspFeedbackCollector::Sample(mavlink_arm_grasp_feedback_t &out)
{
    std::lock_guard<std::mutex> guard(mutex_);
    const double now = model_->GetWorld()->SimTime().Double();
    if (now < last_publish_) { ++epoch_; last_publish_=now; last_contact_=-1; ClearWindow(); }
    const bool resolved=Resolve();
    if (!contact_only_ && object_ && finitePose(object_->WorldPose())) { object_rotation_=object_->WorldPose().Rot(); pose_stamp_=now; }
    if (now-last_publish_ < .1) return false;
    out = {}; out.sample_time_us=uint64_t(now*1e6); out.source_sequence=++sequence_; out.world_epoch=epoch_;
    out.profile_id=1; out.target_id=instance_changed_ ? 2 : 1; out.source=contact_only_ ? 2 : 1; out.sample_window_s=now-last_publish_;
    out.contact_frames=frames_; const double n=std::max(1u,frames_);
    out.fixed_fraction=fixed_frames_/n; out.moving_fraction=moving_frames_/n; out.bilateral_fraction=both_frames_/n;
    out.support_fraction=support_frames_/n; out.other_fraction=other_frames_/n; out.support_arm_fraction=support_arm_frames_/n;
    out.fixed_normal_n=force_[0]/n; out.moving_normal_n=force_[1]/n; out.vertical_force_n=vertical_/n;
    const bool identity_ready=resolved && object_ && support_link_ && instance_latched_ && !instance_changed_;
    if (frames_ && now-last_contact_ < .02 &&
        (!contact_only_ || (now>=last_contact_ && identity_ready))) out.capabilities |= 1;
    if ((out.capabilities & 1) && force_valid_ &&
        (!contact_only_ || (force_wrenches_ && std::isfinite(out.fixed_normal_n) && std::isfinite(out.moving_normal_n))))
        out.capabilities |= 2;
    if (contact_only_) {
        const float unknown=std::numeric_limits<float>::quiet_NaN();
        // Unknown is explicit: no object WorldPose, bench contract or bounds.
        for (int i=0; i<3; ++i) out.object_center_gripper[i]=unknown;
        out.object_height_world=out.body_support_clearance=out.vertical_force_n=unknown;
        if (!(out.capabilities & 2)) out.fixed_normal_n=out.moving_normal_n=unknown;
        ClearWindow(); last_publish_=now;
        return true;
    }
    if (object_ && gripper_ && finitePose(object_->WorldPose()) && finitePose(gripper_->WorldPose())) {
        auto local = gripper_->WorldPose().Rot().RotateVectorReverse(object_->WorldPose().Pos()-gripper_->WorldPose().Pos());
        for (int i=0; i<3; ++i) out.object_center_gripper[i]=local[i];
        out.object_height_world=object_->WorldPose().Pos().Z(); out.capabilities |= 4;
    }
    ignition::math::Vector3d body_low,body_high,support_low,support_high;
    if (boxBounds(body_,body_low,body_high) && boxBounds(support_collision_,support_low,support_high)) {
        out.body_support_clearance=support_low.Y()-body_high.Y(); out.capabilities |= 8;
    }
    if (BenchValid()) out.capabilities |= 16;
    ClearWindow(); last_publish_=now;
    return true;
}
}
