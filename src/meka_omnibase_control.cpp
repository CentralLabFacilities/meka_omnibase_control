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
   for (int i = 0; i < 3; ++i) {
       command_.add_xd_des(0.0);
       status_.add_g_pos(0.0);
       status_.add_l_vel(0.0);
   }
  
   for (int i = 0; i < NUM_CASTERS; ++i) {
       status_.add_beta(0.0);
       status_.add_beta_d(0.0);
       status_.add_phi_d(0.0);
   }

   // NOTE: Cartesian limits currently ignored.
   param_.set_xd_max(0.0);
   param_.set_xdd_max(0.0);
   param_.set_td_max(0.0);
   param_.set_tdd_max(0.0);

   command_.set_ctrl_mode(MEKA_OMNIBASE_CONTROL_OFF);
}

void MekaOmnibaseControl::Shutdown()
{
}

void MekaOmnibaseControl::StepStatus()
{
    if (IsStateError()) {
        return;
    }

    // Update state in robot model.
    using VectorType = omni_kinematics::Robot::VectorType;
    VectorType beta(4);
    VectorType betad(4);
    VectorType phid(4);
    for (int i = 0; i < NUM_CASTERS; ++i) {
        beta[i]  = m3joints_->GetJoint(i*2)->GetThetaRad();
        betad[i] = m3joints_->GetJoint(i*2)->GetThetaDotRad();
        phid[i]  = m3joints_->GetJoint(i*2 + 1)->GetThetaDotRad();
        status_.set_beta(i, beta[i]);
        status_.set_beta_d(i, betad[i]);
        status_.set_phi_d(i, phid[i]);
    }
    robot_.updateState(beta,betad,phid);

    // Update the external status (missing: odometry).
    status_.set_l_vel(0, robot_.xd());
    status_.set_l_vel(1, robot_.yd());
    status_.set_l_vel(2, robot_.td());
    
    // Step control (if enabled). 
    // TODO!

}

void MekaOmnibaseControl::StepCommand()
{
}
