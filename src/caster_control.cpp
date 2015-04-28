#include <meka_omnibase_control/caster_control.hpp>
#include <algorithm>

using namespace meka_omnibase_control;

CasterControl::CasterControl():
    pid_(2)
{
    std::fill(&tq_[0], &tq_[2], 0.0);
}

void CasterControl::readConfig(YAML::Node& doc)
{
    ns_       = doc["param"]["casters"]["Ns"].as<double>();
    nt_       = doc["param"]["casters"]["Nt"].as<double>();
    nw_       = doc["param"]["casters"]["Nw"].as<double>();

    kp_       = doc["param"]["casters"]["k_p"].as<double>();
    kd_       = doc["param"]["casters"]["k_d"].as<double>();
    ki_       = doc["param"]["casters"]["k_i"].as<double>();
    ki_range_ = doc["param"]["casters"]["k_i_range"].as<double>();
    ki_limit_ = doc["param"]["casters"]["k_i_limit"].as<double>();

}

void CasterControl::reset()
{
    pid_[0].ResetIntegrator();
    pid_[1].ResetIntegrator();
}

void CasterControl::stepStatus(double e[2], double ed[2], double edd[2])
{
    std::copy(&e[0],   &e[2],   e_);
    std::copy(&ed[0],  &ed[2],  ed_);
    std::copy(&edd[0], &edd[2], edd_);

    q_[0]  = e_[0]  / ns_;
    q_[1]  = e_[0]  / (ns_ * nw_) - ed_[1] / (nt_ * nw_);
    qd_[0] = ed_[0] / ns_;
    qd_[1] = ed_[0] / (ns_ * nw_) - ed_[1] / (nt_ * nw_);
    // TODO: qdd ?
}

void CasterControl::stepCommand(double qd_des_0, double qd_des_1)
{
    static double ed_des[2];

    ed_des[0] = ns_ * qd_des_0;
    ed_des[1] = nt_ * qd_des_0 - nt_ * nw_ * qd_des_1;

    for (int i = 0; i < 2; ++i) {
        tq_[i] = pid_[i].Step(ed_[i],
                              edd_[i],
                              ed_des[i],
                              kp(),
                              ki(),
                              kd(),
                              ki_limit(),
                              ki_range());
    }
}

