#include <meka_omnibase_control/meka_omnibase_control.hpp>
#include <limits>

using namespace meka_omnibase_control;

bool MekaOmnibaseControl::ReadConfig(const char* filename)
{
    try {
        if (!M3Component::ReadConfig(filename)) {
            std::cerr << "!! An error occured reading the configuration for the"
                      << "M3Component base class."
                      << std::endl;
            return false;
        }

        m3joints_name_ = doc["joint_array_component"].as<std::string>();
        m3pwr_name_    = doc["pwr_component"].as<std::string>();

        // NOTE: Cartesian limits currently ignored.
        param_.set_xd_max( doc["param"]["xd_max"].as<double>());
        param_.set_xdd_max(doc["param"]["xdd_max"].as<double>());
        param_.set_td_max( doc["param"]["td_max"].as<double>());
        param_.set_tdd_max(doc["param"]["tdd_max"].as<double>());

        // Make sure there is at least 4 casters defined by looking at the 
        // lowest count of all parameters:
        size_t n_casters = doc["param"]["alpha"].size();
        n_casters = std::min(n_casters, doc["param"]["l"].size());
        n_casters = std::min(n_casters, doc["param"]["d"].size());
        n_casters = std::min(n_casters, doc["param"]["r"].size());
        n_casters = std::min(n_casters, doc["param"]["beta_offset"].size());
        if (n_casters != NUM_CASTERS) {
            std::cerr << "MekaOmnibaseControl: Config does not define "
                         "4 casters."
                      << std::endl;
            return false;
        }

        //using VectorType = omni_kinematics::Robot::VectorType;
        typedef omni_kinematics::Robot::VectorType VectorType;
        robot_.alpha() = doc["param"]["alpha"].as<VectorType>();
        robot_.l() = doc["param"]["l"].as<VectorType>();
        robot_.d() = doc["param"]["d"].as<VectorType>();
        robot_.r() = doc["param"]["r"].as<VectorType>();

        const std::vector<double>& beta_offset = 
            doc["param"]["beta_offset"].as<VectorType>();
        std::copy(beta_offset.begin(), beta_offset.end(), &beta_offset_[0]);
        const std::vector<double>& beta_ratio = 
            doc["param"]["beta_ratio"].as<VectorType>();
        std::copy(beta_ratio.begin(), beta_ratio.end(), &beta_ratio_[0]);
        const std::vector<double>& phid_ratio = 
            doc["param"]["phid_ratio"].as<VectorType>();
        std::copy(phid_ratio.begin(), phid_ratio.end(), &phid_ratio_[0]);
        const std::vector<double>& des_betad_ratio = 
            doc["param"]["des_betad_ratio"].as<VectorType>();
        std::copy(des_betad_ratio.begin(), 
                  des_betad_ratio.end(), 
                  &des_betad_ratio_[0]);
        const std::vector<double>& des_phid_ratio = 
            doc["param"]["des_phid_ratio"].as<VectorType>();
        std::copy(des_phid_ratio.begin(), 
                  des_phid_ratio.end(), 
                  &des_phid_ratio_[0]);

        robot_.maxBetad() = doc["param"]["betad_max"].as<VectorType>();
        robot_.maxBetadd() = doc["param"]["betadd_max"].as<VectorType>();
        robot_.maxPhid() = doc["param"]["phid_max"].as<VectorType>();
        robot_.maxPhidd() = doc["param"]["phidd_max"].as<VectorType>();
        
        double tq_max = doc["param"]["tq_max"].as<double>();
        param_.set_tq_max(tq_max);
        double tq_sum_max = doc["param"]["tq_sum_max"].as<double>();
        param_.set_tq_sum_max(tq_sum_max);

        double bdmax_ratio = doc["param"]["bdmax_ratio"].as<double>();
        param_.set_bdmax_ratio(bdmax_ratio);

        for (int i = 0; i < NUM_CASTERS; ++i) {
            casters_[i].readConfig(doc);
        }
        param_.set_k_ed_p(casters_[0].kp());
        param_.set_k_ed_i(casters_[0].ki());
        param_.set_k_ed_d(casters_[0].kd());
        param_.set_k_ed_i_limit(casters_[0].ki_range());
        param_.set_k_ed_i_range(casters_[0].ki_limit());

    } catch (std::exception e) {
        std::cerr << "!!! meka_omnibase_control: failed to read config: "
                  << std::endl
                  << e.what()
                  << std::endl;
        return false;
    }

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
       command_.add_betad_des(0.0);
       command_.add_phid_des(0.0);
       command_.add_tqr(0.0);

       status_.add_beta(0.0);
       status_.add_beta_d(0.0);
       status_.add_phi_d(0.0);

       status_.add_alpha(robot_.alpha()[i]);
       status_.add_l(robot_.l()[i]);

       status_.add_calib(false);

       param_.add_beta_offset(beta_offset_[i]);

       // -1 values for cycle counters indicate they are not active:
       unstable_start_[i] = -1;
       zero_vel_start_    = -1;
   }

   // NOTE: Cartesian limits currently ignored.
   
   command_.set_ctrl_mode(MEKA_OMNIBASE_CONTROL_OFF);
   last_ctrl_mode_ = MEKA_OMNIBASE_CONTROL_OFF;

   // Reset global odometry.
   robot_.resetPose();
   
   cycle_ = 0;
}

void MekaOmnibaseControl::Shutdown()
{
}

void MekaOmnibaseControl::StepStatus()
{
    //using VectorType = omni_kinematics::Robot::VectorType;
    typedef omni_kinematics::Robot::VectorType VectorType;
    static VectorType beta(NUM_CASTERS);
    static VectorType betad(NUM_CASTERS);
    static VectorType phi(NUM_CASTERS);
    static VectorType phid(NUM_CASTERS);

    if (IsStateError()) {
        return;
    }

    // Update state in robot model.
    for (int i = 0; i < NUM_CASTERS; ++i) {
        double e[2], ed[2], edd[2];

        m3::M3Joint* joint0 = m3joints_->GetJoint(i*2);
        m3::M3Joint* joint1 = m3joints_->GetJoint(i*2+1);

        // We rely on the first motor encoder for each pair, as a breakbeam
        // sensor is used to 'reliably' locate the zero.
        bool calib = joint0->IsEncoderCalibrated();
        if (!calib) {
            // Encoder not calibrated, turn on the breakbeam sensor.
            joint0->SetLimitSwitchNegZeroEncoder();
        } else {
            // Encoder calibrated, turn off the breakbeam sensor.
            joint0->ClrLimitSwitchNegZeroEncoder();
        }
        status_.set_calib(i, calib);

        e[0]   = joint0->GetThetaRad();
        e[1]   = joint1->GetThetaRad();
        ed[0]  = joint0->GetThetaDotRad();
        ed[1]  = joint1->GetThetaDotRad();
        edd[0] = joint0->GetThetaDotDotRad();
        edd[1] = joint1->GetThetaDotDotRad();

        // Use the caster status update to convert motor velocities to joint
        // velocities (we're interested in what's passed the gearbox).
        casters_[i].stepStatus(e, ed, edd);
        casters_[i].q(beta[i], phi[i]); // phi is unused.
        casters_[i].qd(betad[i], phid[i]);

        // Geometrical transforms to allow reversed directions and sensor
        // offsets.
        // NOTE: The beta_offset_ member is only for initial values, the rest is
        // updated with the param struct.
        beta[i]  = beta_ratio_[i] * omni_kinematics::normalizedAngle(
                                        beta[i] + param_.beta_offset(i));
        betad[i] = beta_ratio_[i] * betad[i];
        phid[i]  = phid_ratio_[i] * phid[i];

        //std::cerr << "betad: " << betad[i] << " phid:" << phid[i] << std::endl; 

        status_.set_beta(i, beta[i]);
        status_.set_beta_d(i, betad[i]);
        status_.set_phi_d(i, phid[i]);

    }

    robot_.updateState(beta, betad, phid, 1.0 / RT_TASK_FREQUENCY);

    // Update the external status.
    status_.set_l_vel(0, robot_.xd());
    status_.set_l_vel(1, robot_.yd());
    status_.set_l_vel(2, robot_.td());
    status_.set_g_pos(0, robot_.pose().x);
    status_.set_g_pos(1, robot_.pose().y);
    status_.set_g_pos(2, robot_.pose().t);
    
}

bool MekaOmnibaseControl::casterStable(int i)
{
    if (!casters_[i].stable()) {
        unstable_start_[i] = now();
        return false;
    } else if (elapsed(unstable_start_[i]) < 1000) {
        return false;
    } else {
        unstable_start_[i] = -1;
        return true;
    }
}

bool MekaOmnibaseControl::testZeroVel()
{
    static const double EPS = 1e-6;

    const double& xd = command_.xd_des(0);
    const double& yd = command_.xd_des(1);
    const double& td = command_.xd_des(2);
    return ((xd*xd + yd*yd + td*td) < EPS);
}

void MekaOmnibaseControl::StepCommand()
{
    //using VectorType = omni_kinematics::Robot::VectorType;
    //using Twist      = omni_kinematics::Twist;
    typedef omni_kinematics::Robot::VectorType VectorType;
    typedef omni_kinematics::Twist             Twist;

    static Twist twist;
    static VectorType betad(NUM_CASTERS, 0.0);
    static VectorType phid(NUM_CASTERS, 0.0);
    static VectorType tq(NUM_CASTERS*2, 0.0);


    twist.xd = command_.xd_des(0);
    twist.yd = command_.xd_des(1);
    twist.td = command_.xd_des(2);
    ctrl_.saturateTwist(twist, 1.0 / RT_TASK_FREQUENCY, true);


    if (command_.ctrl_mode() != MEKA_OMNIBASE_CONTROL_OFF) {

        if (command_.ctrl_mode() == MEKA_OMNIBASE_CONTROL_CC) {
            // Caster tuning mode, copy the incoming command.
            for (int i = 0; i < NUM_CASTERS; ++i) {
                betad[i] = command_.betad_des(i);
                phid[i] = command_.phid_des(i);
            }

        } else if (command_.ctrl_mode() == MEKA_OMNIBASE_CONTROL_ON) {
            // Standard local velocity mode.
            ctrl_.calcCommand(twist, betad, phid);
        }

        for (int i = 0; i < NUM_CASTERS; ++i) {
            betad[i] *= des_betad_ratio_[i];
            phid[i]  *= des_phid_ratio_[i];
        }

        M3JointArrayCommand* cmd = (M3JointArrayCommand*)m3joints_->GetCommand();
        for (int i = 0; i < NUM_CASTERS; ++i) {
            // Update PID parameters and bdmax (they might have changed).
            casters_[i].pidParams(param_.k_ed_p(),
                                  param_.k_ed_i(),
                                  param_.k_ed_d(),
                                  param_.k_ed_i_limit(),
                                  param_.k_ed_i_range());
            casters_[i].bdmax(param_.bdmax_ratio() * robot_.maxBetad()[i]);

            if (last_ctrl_mode_ == MEKA_OMNIBASE_CONTROL_OFF) {
                casters_[i].reset();
            }

            casters_[i].stepCommand(betad[i], phid[i]);
            casters_[i].tq(tq[2*i], tq[2*i+1]);
            for (int j = 2*i; j < (2*i+1); ++j) {
                tq[j] *= command_.tqr(i);
                tq[j] = CLAMP(tq[j], -param_.tq_max(), param_.tq_max());
            }

            // Unstable wheel test: disable torque for both motors if beta_dot 
            // is too high.
            // It will restart automatically when the speed slows down, but let
            // it off for at least 1000 cycles (1 sec).
            if (!casterStable(i)) {
                tq[2*i] = tq[2*i+1] = 0.0;
            }
        }

        // Test for zero velocity: release torque control and reset PIDs if the
        // desired velocity has been zero for 1000 cycles (1 sec).
        if (testZeroVel()) {
            // Currently at zero velocity.
            if (zero_vel_start_ > 0) {
                // If the zero velocity counter started, test the elapsed time.
                if (elapsed(zero_vel_start_) > 1000) {
                    for (int i = 0; i < NUM_CASTERS; ++i) {
                        casters_[i].reset();
                    }
                }
            } else {
                // Start zero velocity cycle counter.
                zero_vel_start_ = now();
            }
        } else {
            // Reset zero velocity cycle counter.
            zero_vel_start_ = -1;
        }

        // Check the total torque sum and normalize to avoid reaching current
        // limit.
        double tq_sum = 0.0;
        for (int j = 0; j < NUM_CASTERS*2; ++j) {
            tq_sum += fabs(tq[j]);
        }
        if (tq_sum > param_.tq_sum_max()) {
            double ratio = param_.tq_sum_max() / tq_sum;
            for (int j = 0; j < NUM_CASTERS*2; ++j) {
                tq[j] *= ratio;
            }
        }

        for (int i = 0; i < NUM_CASTERS*2; ++i) {
            m3joints_->GetJoint(i)->DisablePwmRamp();   // Make sure this is necessary
            cmd->set_ctrl_mode(i, JOINT_ARRAY_MODE_TORQUE);
            cmd->set_tq_desired(i, tq[i]);
        }

#ifdef DEBUG_OUTPUT
        if (!(now() % 100)) {
            std::cerr << "betas:     "  << robot_.beta()[0] << ", "
                                        << robot_.beta()[1] << ", "
                                        << robot_.beta()[2] << ", "
                                        << robot_.beta()[3]
                      << std::endl; 
            std::cerr << "phids:     " << robot_.phid()[0] << ", "
                                       << robot_.phid()[1] << ", "
                                       << robot_.phid()[2] << ", "
                                       << robot_.phid()[3]
                      << std::endl; 
            std::cerr << "des xd:    "  << twist.xd << ", "
                                        << twist.yd << ", "
                                        << twist.td
                      << std::endl;
            std::cerr << "des betad: "  << betad[0] << ", "
                                        << betad[1] << ", "
                                        << betad[2] << ", "
                                        << betad[3]
                      << std::endl; 
            std::cerr << "des phid:  "  << phid[0] << ", "
                                        << phid[1] << ", "
                                        << phid[2] << ", "
                                        << phid[3]
                      << std::endl; 
            std::cerr << "tq0:       "  << cmd->tq_desired(0) << ", "
                                        << cmd->tq_desired(2) << ", "
                                        << cmd->tq_desired(4) << ", "
                                        << cmd->tq_desired(6)
                      << std::endl; 
            std::cerr << "tq1:       "  << cmd->tq_desired(1) << ", "
                                        << cmd->tq_desired(3) << ", "
                                        << cmd->tq_desired(5) << ", "
                                        << cmd->tq_desired(7)
                      << std::endl; 
            std::cerr << "stability: "  << casterStable(0) << ", "
                                        << casterStable(1) << ", "
                                        << casterStable(2) << ", "
                                        << casterStable(3)
                      << std::endl;
        }
#endif

    } else {
        M3JointArrayCommand* cmd = (M3JointArrayCommand*)m3joints_->GetCommand();
        for (int i = 0; i < NUM_CASTERS*2; ++i) {
            m3joints_->GetJoint(i)->SetDesiredControlMode(JOINT_MODE_OFF);
            cmd->set_tq_desired(i, 0.0);
        }
    }

    last_ctrl_mode_ = command_.ctrl_mode();
    tick();
}

int MekaOmnibaseControl::now() const
{
    return cycle_;
}

void MekaOmnibaseControl::tick()
{
    ++cycle_;
    if (cycle_ < 0) {
        // Overflowed:
        cycle_ = 0;
    }
}

int MekaOmnibaseControl::elapsed(int begin) const
{
    // NOTE: Assumes now is always in the future and tests for overflows.
    
    int e = now() - begin;
    if (e < 0) {
        // Overflowed, recalculate: 
        return std::numeric_limits<int>::max() + e;
        
    }
}

