#include <meka_omnibase_control/caster_control.hpp>
#include <algorithm>

using namespace meka_omnibase_control;

CasterControl::CasterControl(double ns, double nw, double nt, double kp):
    ns_(ns), nw_(nw), nt_(nt), kp_(kp)
{
    std::fill(&tq_[0], &tq_[2], 0.0);
}

void CasterControl::stepStatus(double e_0, double e_1)
{
    e_[0] = e_0;
    e_[1] = e_1;

    qd_[0] = e_[0] / ns_;
    qd_[1] = e_[0] / (ns_ * nw_) - e_[1] / (nt_ * nw_);
}

void CasterControl::stepCommand(double qd_des_0, double qd_des_1)
{
    static double e_des[2];

    e_des[0] = ns_ * qd_des_0;
    e_des[1] = nt_ * qd_des_0 - nt_ * nw_ * qd_des_1;

    for (int i = 0; i < 2; ++i) {
        // Proportional, additive control
        tq_[i] += kp_ * (e_des[i] - e_[i]);
    }
}

