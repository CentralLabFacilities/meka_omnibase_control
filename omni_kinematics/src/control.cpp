#include <omni_kinematics/control.hpp>
#include <cmath>

using namespace omni_kinematics;

MotionControl::MotionControl(const Robot* robot): robot_(robot)
{
}

void MotionControl::saturateTwist(Twist& cmd, double dt, bool worst_case) const
{
    // TODO: Should probably be combined with calcCommand.
    
    double& xd = cmd.xd;
    double& yd = cmd.yd;
    double& td = cmd.td;
    unsigned int n = robot_->nbWheels();

    // Final scaling factor:
    double scale = 1.0;

    if (worst_case)
    {
        // NOTE: The worst-case test does not consider a specific wheel steering 
        // pivot.
        // It only test for angular velocity applied parallel to the the linear
        // velocity at a the worst (l + d) distance.
        // This is why we only work with absolute values for both velocities.
        double v;
        if (robot_->overMaxVel(cmd, dt, &v))
        {
            scale = robot_->maxV() / v;
        }

    } else {
        for (unsigned int i = 0; i < n; ++i)
        {
            const double& l = robot_->l()[i];
            const double& a = robot_->alpha()[i];
            const double& r = robot_->r()[i];
            const double& d = robot_->d()[i];
            const double& b = robot_->beta()[i];
            double sab = sin(a + b);
            double cab = cos(a + b);
            double lcb = l * cos(b);
            double lsb = l * sin(b);

            // 1. Figure out the maximum allowable twist by the current wheel.
            // This is done in the wheel's steering axis reference frame, where 
            // X is parallel to the propulsion wheel (or the d offset), and Y 
            // parallel to the propulsion axis.
            // In other words, a positive phi_dot produce motion away from the
            // steering axis (assuming the d offset is positive), and beta_dot a 
            // positive motion in Y.
            // NOTE: As we don't currently consider acceleration constraints, 
            // these are constants.
                  
            double max_x = robot_->maxPhid()[i] * r;
            double max_y = robot_->maxBetad()[i] * d;

            // 2. Project the twist along these dimensions, follows the rolling 
            // and no-slip constraints, and test for saturation.
            
            double c_phi = fabs(sab * xd - cab * yd - lcb * td);
            if (c_phi > max_x)
            {
                double ns = max_x / c_phi;
                if (ns < scale)
                    scale = ns;
            }

            double c_beta = fabs(-cab * xd - sab * yd - (d + lsb) * td);
            if (c_beta > max_y)
            {
                // Calculate a scaling factor for this constraint, only keep the
                // lowest one at each step.
                double ns = max_y / c_beta;
                if (ns < scale)
                    scale = ns;
            }

        }
    }

    // Apply final scaling factor:
    cmd.xd *= scale;
    cmd.yd *= scale;
    cmd.td *= scale;

}

void MotionControl::calcCommand(const Twist& twist,
    Robot::VectorType& betad, Robot::VectorType& phid) const
{
    unsigned int n = robot_->nbWheels();
    betad.resize(n);
    phid.resize(n);

    const double& xd = twist.xd;
    const double& yd = twist.yd;
    const double& td = twist.td;

    for (unsigned int i = 0; i < n; ++i)
    {
        const double& l = robot_->l()[i];
        const double& a = robot_->alpha()[i];
        const double& r = robot_->r()[i];
        const double& d = robot_->d()[i];
        const double& b = robot_->beta()[i];
        double sab = sin(a + b);
        double cab = cos(a + b);
        double lcb = l * cos(b);
        double lsb = l * sin(b);

        // See Springer's Robotics Handbook ch. 17 for details.
        betad[i] = (-cab * xd - sab * yd - (d + lsb) * td) / d;
        phid[i] = (sab * xd - cab * yd - lcb * td) / r;
    }

}

