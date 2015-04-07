#include <omni_kinematics/common.hpp>
#include <limits>
#include <cmath>

namespace 
{
    void saturateSingle(const double& cur, double& cmd, double dt, double acc)
    {
        double accdt = acc * dt;
        double diff  = cmd - cur;
        double adiff = fabs(diff);

        if (adiff > accdt) 
            cmd = cur + (adiff/diff) * accdt;
    }
}

namespace omni_kinematics
{

    void twistToPos(const Twist& vel, const double& d, 
        double& x, double& y, double& t)
    {
        static const double& EPS = std::numeric_limits<double>::epsilon();

        const double& xd = vel.xd;
        const double& yd = vel.yd;
        const double& td = vel.td;

        if (fabs(td) < EPS)
        {
            // Assumes zero rotation, simple straight line.
            x = xd * d;
            y = yd * d;
            t = 0.0;
            return;
        }

        // ICR polar coordinates: 
        // (rho, gamma) -> (-rho * sin(gamma), rho * cos(gamma))
        // rho = ||(xd, yd)|| / td
        // gamma = atan2(yd, xd)
        // With gamma = 0, the ICR is at (0, rho) in the robot's frame.
        // The result comes from applying displacement calculated in the ICR's
        // frame to the robot's frame, hence the rotation.
        double rho = sqrt(xd*xd + yd*yd);
        double gamma = atan2(yd, xd);
        t = td * d;
        double cg = cos(gamma);
        double sg = sin(gamma);
        double ct = cos(t);
        double stmo = sin(t) - 1.0;
        x = rho * ( cg * ct + sg * stmo); 
        y = rho * (-sg * ct + sg * stmo);
    }

    void saturateTwist(const Twist& cur_twist, Twist& cmd_twist, double dt,
        const double& xdd_max, const double& ydd_max, const double& tdd_max)
    {
        saturateSingle(cur_twist.xd, cmd_twist.xd, dt, xdd_max);
        saturateSingle(cur_twist.yd, cmd_twist.yd, dt, ydd_max);
        saturateSingle(cur_twist.td, cmd_twist.td, dt, tdd_max);
    }
}

