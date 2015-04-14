#include <meka_omnibase_control/caster_control.hpp>
#include <algorithm>

using namespace meka_omnibase_control;

CasterControl::CasterControl(double ns,
                             double nw,
                             double nt,
                             double kp,
                             double kd):
    ns_(ns),
    nw_(nw),
    nt_(nt),
    kp_(kp),
    kd_(kd)
{
    std::fill(&tq_[0], &tq_[2], 0.0);
}

void CasterControl::stepStatus(double e[2], double ed[2])
{
    std::copy(&e[0], &e[2], e_);
    std::copy(&ed[0], &ed[2], ed_);

    q_[0]  = e_[0]  / ns_;
    q_[1]  = e_[0]  / (ns_ * nw_) - ed_[1] / (nt_ * nw_);
    qd_[0] = ed_[0] / ns_;
    qd_[1] = ed_[0] / (ns_ * nw_) - ed_[1] / (nt_ * nw_);
}

void CasterControl::stepCommand(double qd_des_0, double qd_des_1)
{
    static double ed_des[2];

    ed_des[0] = ns_ * qd_des_0;
    ed_des[1] = nt_ * qd_des_0 - nt_ * nw_ * qd_des_1;

    for (int i = 0; i < 2; ++i) {
        tq_[i] = kp_ * (ed_des[i] - ed_[i]) - kd_ * ed_[i];
    }
}

