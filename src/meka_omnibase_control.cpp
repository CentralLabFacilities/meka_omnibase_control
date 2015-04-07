#include <meka_omnibase_control/meka_omnibase_control.hpp>

using namespace meka_omnibase_control;

bool MekaOmnibaseControl::ReadConfig(const char* filename)
{
    if (!M3Component::ReadConfig(filename)) {
        return false;
    }

    m3joints_name_ = doc["joint_array_component"].as<std::string>();
    m3pwr_name_    = doc["pwr_component"].as<std::string>();

    param_.set_xd_max( doc["param"]["xd_max"].as<double>());
    param_.set_xdd_max(doc["param"]["xdd_max"].as<double>());
    param_.set_td_max( doc["param"]["td_max"].as<double>());
    param_.set_tdd_max(doc["param"]["tdd_max"].as<double>());

    // Make sure there is at least 4 casters defined by looking at the lowest
    // count of all parameters:
    size_t n_casters = doc["param"]["alpha"].size();
    n_casters = std::min(n_casters, doc["param"]["l"].size());
    n_casters = std::min(n_casters, doc["param"]["d"].size());
    n_casters = std::min(n_casters, doc["param"]["r"].size());
    n_casters = std::min(n_casters, doc["param"]["beta_offset"].size());
    if (n_casters != NUM_CASTERS) {
        std::cerr << "MekaOmnibaseControl: Config does not define 4 casters."
                  << std::endl;
        return false;
    }

    using VectorType = omni_kinematics::Robot::VectorType;
    robot_.alpha() = doc["param"]["alpha"].as<VectorType>();
    robot_.l() = doc["param"]["l"].as<VectorType>();
    robot_.d() = doc["param"]["d"].as<VectorType>();
    robot_.r() = doc["param"]["r"].as<VectorType>();

    robot_.maxBetad() = doc["param"]["betad_max"].as<VectorType>();
    robot_.maxBetadd() = doc["param"]["betadd_max"].as<VectorType>();
    robot_.maxPhid() = doc["param"]["phid_max"].as<VectorType>();
    robot_.maxPhidd() = doc["param"]["phidd_max"].as<VectorType>();

    robot_.calcConstraints();

    return true;
}

bool MekaOmnibaseControl::LinkDependentComponents()
{
    m3joints_ = dynamic_cast<m3::M3JointArray *>(
        factory->GetComponent(m3joints_name_));
    m3pwr_    = (m3::M3Pwr *) factory->GetComponent(m3pwr_name_);

    return (m3joints_ != NULL && m3pwr_ != NULL);
}

void MekaOmnibaseControl::Startup()
{
}

void MekaOmnibaseControl::Shutdown()
{
}

void MekaOmnibaseControl::StepStatus()
{
}

void MekaOmnibaseControl::StepCommand()
{
}
