#ifndef OMNI_KINEMATICS_COMMON_HPP
#define OMNI_KINEMATICS_COMMON_HPP

#include <cmath>

namespace omni_kinematics
{
    static const double PI = M_PI;

    struct Twist
    {
        Twist(): xd(0), yd(0), td(0) {}

        double xd; // x_dot
        double yd; // y_dot
        double td; // theta_dot
    };

    struct Pose
    {
        Pose(): x(0), y(0), t(0) {}

        double x;
        double y;
        double t; // theta
    };

    /// \brief Integrate a twist (vel) over a duration d.
    ///
    /// Figure out the ICR and where it should end.
    /// Output is relative, i.e. (x,y,t) always starts at (0,0,0).
    /// The given values are ignored.
    void twistToPos(const Twist& vel, const double& d, double& x, double& y, 
        double& t);

    /// \brief Saturate a twist according to the given acceleration limits.
    ///
    /// \param cur_twist Current twist to base acceleration on.
    /// \param cmd_twist The twist to saturate.
    /// \param dt        Time basis.
    /// \param xdd_max   Maximum absolute acceleration in X.
    /// \param ydd_max   Maximum absolute acceleration in Y.
    /// \param tdd_max   Maximum absolute acceleration around Z (theta).
    void saturateTwist(const Twist& cur_twist, Twist& cmd_twist, double dt,
        const double& xdd_max, const double& ydd_max, const double& tdd_max);

    /// \brief Normalize from -PI to PI.
    static inline double normalizedAngle(const double& a)
    {
        // NOTE: We assume that angles are generally close to the normalized
        // range, so we'll rarely go each test more than once.
        double r = a;
        while (r < -M_PI) {
            r += M_PI;
        }
        while (r > M_PI) {
            r -= M_PI;
        }

        return r;
    }

    /// \brief Return the shortest distance from a to b.
    static inline double shortestAngularDistance(const double& a, 
                                                 const double& b)
    {
        return normalizedAngle(b-a);
    }
}



#endif

