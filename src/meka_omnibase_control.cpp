#include <meka_omnibase_control/meka_omnibase_control.hpp>

using namespace meka_omnibase_control;

bool MekaOmnibaseControl::ReadConfig(const char* filename)
{
    if (!M3Component::ReadConfig(filename)) {
        return false;
    }

    param_.set_xd_max( doc["param"]["xd_max"].as<double>());
    param_.set_xdd_max(doc["param"]["xdd_max"].as<double>());
    param_.set_td_max( doc["param"]["td_max"].as<double>());
    param_.set_tdd_max(doc["param"]["tdd_max"].as<double>());

    // Find the lowest common value between all vector sizes to set the number
    // of casters:
    size_t n_casters = doc["param"]["alpha"].size();
    n_casters = std::min(n_casters, doc["param"]["l"].size());
    n_casters = std::min(n_casters, doc["param"]["d"].size());
    n_casters = std::min(n_casters, doc["param"]["r"].size());
    n_casters = std::min(n_casters, doc["param"]["beta_offset"].size());
    if (n_casters < 3) {
        std::cerr << "MekaOmnibaseControl: Config defines less than 3 casters."
                  << std::endl;
        return false;
    }
    for (int i = 0; i < n_casters; ++i) {
        param_.add_alpha(doc["param"]["alpha"][i].as<double>());
        param_.add_l(doc["param"]["l"][i].as<double>());
        param_.add_d(doc["param"]["d"][i].as<double>());
        param_.add_r(doc["param"]["r"][i].as<double>());
        param_.add_beta_offset(doc["param"]["beta_offset"][i].as<double>());
    }
}
