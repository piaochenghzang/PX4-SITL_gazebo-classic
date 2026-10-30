#include <so101/contact_force.h>
#include <gazebo/msgs/joint_wrench.pb.h>
#include <iostream>
#include <limits>

namespace {
unsigned checks=0;
bool check(bool value, const char *label)
{
    ++checks;
    if (!value) std::cerr << "FAIL: " << label << '\n';
    return value;
}
void force(gazebo::msgs::Wrench *w, double x, double y=0., double z=0.)
{
    auto *v=w->mutable_force();v->set_x(x);v->set_y(y);v->set_z(z);
}
}
int main()
{
    bool passed=true;double value=0.;gazebo::msgs::JointWrench pair;
    force(pair.mutable_body_1_wrench(),-100.,5.,8.);
    force(pair.mutable_body_2_wrench(),-2.,7.,9.);
    passed &= check(so101_contact::compression(pair,true,0,value) && value==2.,"target first selects fixed finger body 2");
    force(pair.mutable_body_1_wrench(),-3.);
    passed &= check(so101_contact::compression(pair,false,0,value) && value==3.,"target second selects fixed finger body 1");
    force(pair.mutable_body_2_wrench(),4.);
    passed &= check(so101_contact::compression(pair,true,1,value) && value==4.,"moving pad compression uses positive local X");
    force(pair.mutable_body_1_wrench(),5.);
    passed &= check(so101_contact::compression(pair,false,1,value) && value==5.,"moving pad handles reversed collision order");
    passed &= check(so101_contact::compression(pair,true,0,value) && value==0.,"outward fixed force is not compression");
    force(pair.mutable_body_2_wrench(),-4.);
    passed &= check(so101_contact::compression(pair,true,1,value) && value==0.,"outward moving force is not compression");
    const double nan=std::numeric_limits<double>::quiet_NaN();
    for (unsigned axis=0;axis<3;++axis) {
        force(pair.mutable_body_2_wrench(),axis==0 ? nan : -2.,axis==1 ? nan : 0.,axis==2 ? nan : 0.);
        passed &= check(!so101_contact::compression(pair,true,0,value) && std::isnan(value),"nonfinite finger force axis is rejected");
    }
    force(pair.mutable_body_2_wrench(),-2.);force(pair.mutable_body_1_wrench(),nan);
    passed &= check(so101_contact::compression(pair,true,0,value) && value==2.,"unused target force has no effect");
    passed &= check(!so101_contact::compression(pair,true,2,value) && std::isnan(value),"invalid pad side is rejected");
    pair.clear_body_2_wrench();
    passed &= check(!so101_contact::compression(pair,true,0,value) && std::isnan(value),"missing pad wrench is not zero compression");
    pair.mutable_body_2_wrench()->mutable_force()->set_x(-2.);
    passed &= check(!so101_contact::compression(pair,true,0,value) && std::isnan(value),"incomplete force vector is rejected");
    std::cout << checks << " contact-force checks " << (passed ? "passed" : "FAILED") << '\n';
    return passed ? 0 : 1;
}
