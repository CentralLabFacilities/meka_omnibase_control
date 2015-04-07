#include <omni_kinematics/robot.hpp>
#include <cassert>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <numeric>
#include <limits>

using namespace omni_kinematics;

Robot::Robot(unsigned int n, 
             double       l, 
             double       alpha, 
             double       r, 
             double       d, 
             double       max_betad, 
             double       max_phid,
             double       max_betadd,
             double       max_phidd):
    nb_wheels_(n),
    l_(nb_wheels_, l),
    alpha_(nb_wheels_, alpha),
    r_(nb_wheels_, r),
    d_(nb_wheels_, d),
    max_betad_(nb_wheels_, max_betad),
    max_phid_(nb_wheels_, max_phid),
    max_betadd_(nb_wheels_, max_betadd),
    max_phidd_(nb_wheels_, max_phidd),
    beta_(nb_wheels_, 0.0),
    betad_(nb_wheels_, 0.0),
    phid_(nb_wheels_, 0.0)
{
    // NOTE: We don't support robots with less than 3 wheels.
    assert(nb_wheels_ > 2);

    // Put wheel corners at regular intervals.
    for (unsigned int i = 1; i < nb_wheels_; ++i)
    {
        alpha_[i] += i * 2.0 * PI / nb_wheels_;
    }

    calcConstraints();
}

Robot::Robot(const Robot& r): 
    nb_wheels_(r.nb_wheels_),
    l_(r.l_),
    alpha_(r.alpha_),
    r_(r.r_),
    d_(r.d_),
    max_betad_(r.max_betad_),
    max_phid_(r.max_phid_),
    max_betadd_(r.max_betadd_),
    max_phidd_(r.max_phidd_),
    max_v_(r.max_v_),
    max_a_(r.max_a_),
    max_ld_(r.max_ld_),
    beta_(r.beta_),
    betad_(r.betad_),
    phid_(r.phid_)
{
}

void Robot::updateState(const VectorType& beta, const VectorType& betad, 
    const VectorType& phid)
{
    assert(beta.size() == nb_wheels_);
    assert(betad.size() == nb_wheels_);
    assert(phid.size() == nb_wheels_);

    beta_ = beta;
    betad_ = betad;
    phid_ = phid;

    calcVelocity();

}

bool Robot::isParallel(double* angle, const double eps) const
{
    static double tmp = 0;
    if (angle == NULL)
        angle = &tmp;

    VectorType abs_angle(nb_wheels_);
    /* TODO: Normalize with internal method:
    for (size_t i = 0; i < nb_wheels_; ++i)
        abs_angle[i] = angles::normalize_angle(alpha_[i] + beta_[i]);
    */

    double sum_angle = std::accumulate(abs_angle.begin(), abs_angle.end(), 0.0);
    *angle = sum_angle / double(nb_wheels_);

    double std_dev = 0.0;
    for (size_t i = 0; i < nb_wheels_; ++i)
    {
        // TODO: Replace with own method:
        double diff = 0.0; //angles::shortest_angular_distance(abs_angle[i], *angle);
        std_dev += diff * diff;
    }
    std_dev /= double(nb_wheels_);
    // ROS_DEBUG_THROTTLE(1.0, "Parallelism test stddev: %f", std_dev);
    return (std_dev <= eps);
}

void Robot::calcVelocity()
{
    // We actually only need two steerable wheels to solve this (degree of
    // steerability = 2), but we will use all wheels in a least squares 
    // approach.
    
    using namespace Eigen;

    // Build up constraints matrix.
    MatrixXd m(2 * nb_wheels_, 3);
    VectorXd rhs(2 * nb_wheels_);
    for (unsigned int i = 0; i < nb_wheels_; ++i)
    {
        double sab = sin(alpha_[i] + beta_[i]);
        double cab = cos(alpha_[i] + beta_[i]);
        double dlsb = d_[i] + l_[i] * sin(beta_[i]);
        double lcb = l_[i] * cos(beta_[i]);

        m(i, 0) = sab;
        m(i, 1) = -cab;
        m(i, 2) = -lcb;
        m(i + nb_wheels_, 0) = cab;
        m(i + nb_wheels_, 1) = sab;
        m(i + nb_wheels_, 2) = dlsb;

        rhs(i) = r_[i] * phid_[i];
        rhs(i + nb_wheels_) = -d_[i] * betad_[i];
    }

    //std::cerr << "SVD m: \n" << m << std::endl;
    //std::cerr << "SVD rhs: \n" << rhs << std::endl;

    JacobiSVD<MatrixXd> svd(m, ComputeThinU | ComputeThinV);
    Vector3d x = svd.solve(rhs);

    twist_.xd = x(0);
    twist_.yd = x(1);
    twist_.td = x(2);

}

void Robot::calcConstraints()
{
    // Chassis velocity (and acceleration) has to respect two maximum 
    // constraints:
    // 
    // | v + (l + d) * thetad | <= d * max_betad, and
    // | v + (l + d) * thetad | <= r * max_phid,
    // 
    // where v = | xd + yd |, the linear velocity.
    // Both assume worst-cases scenarios, for instance when all robot movement
    // needs to be done by a single wheel's beta motion.
    // Therefore, the | v + (l + d) * thetad | limit has to be shared between
    // the two velocity dimensions (linear and angular), as maximum linear
    // velocity can only be attained with zero angular velocity, and vice versa.
    // However, this method calculates the actual limits of both constraints,
    // and the role of saturating one or the other should be left to the user.
    
    // First, find out the actual limit, d * max_betad or r * max_phid, for all
    // wheels:
    int w_i = 0; // Index of the most constrained wheel.
    max_v_  = std::numeric_limits<double>::max();
    max_a_  = std::numeric_limits<double>::max();
    for (unsigned int i = 0; i < nb_wheels_; ++i) {
        double vw = std::min(d_[i] * max_betad_[i],  r_[i] * max_phid_[i]);
        double aw = std::min(d_[i] * max_betadd_[i], r_[i] * max_phidd_[i]);
        if (vw < max_v_) {
            w_i    = i;
            max_v_ = vw;
        }
        if (aw < max_a_) {
            max_a_ = aw;
            w_i    = i; 
        }
    }

    max_ld_                       = l_[w_i] + d_[w_i];
    max_twist_.xd = max_twist_.yd = max_v_;
    max_twist_.td                 = max_v_ / max_ld_;
}

bool Robot::overMaxVel(const Twist& cmd, const double& dt, double* v) const
{
    return overMaxVel(cmd.xd, cmd.yd, cmd.td, dt, v);
}

bool Robot::overMaxVel(const Twist&  cmd, 
                       const Twist&  cur, 
                       const double& dt, 
                       double*       v) const
{
    return overMaxVel(cmd.xd, cmd.yd, cmd.yd, cur.xd, cur.yd, cur.yd, dt, v);
}

bool Robot::overMaxVel(const double& xd,
                       const double& yd,
                       const double& td,
                       const double& dt,
                       double*       v) const
{
    return overMaxVel(xd, yd, td, twist_.xd, twist_.yd, twist_.td, dt, v);
}

bool Robot::overMaxVel(const double& xd, 
                       const double& yd, 
                       const double& td,
                       const double& cxd,
                       const double& cyd,
                       const double& ctd,
                       const double& dt,
                       double*       v) const
{
    // TODO: Acceleration test disabled, re-enable when it will be properly
    // tested and configured.

    double cmd_lin_vel = sqrt(xd * xd + yd * yd);
    double cmd_ang_vel = fabs(td);
    double cmd_v = cmd_lin_vel + maxLD() * cmd_ang_vel;
#ifdef OMNI_KINEMATICS_ACC_TEST
    double cur_lin_vel = sqrt(cxd * cxd + cyd * cyd);
    double cur_ang_vel = fabs(ctd);
    double cur_v = cur_lin_vel + maxLD() * cur_ang_vel;
#endif

    if (v) {
        *v = cmd_v;
    }

#ifdef OMNI_KINEMATICS_ACC_TEST
    double max_v = std::min(maxV(), cur_v + max_a_ * dt);
#else
    const double& max_v = maxV();
#endif
    return cmd_v > max_v;
}

