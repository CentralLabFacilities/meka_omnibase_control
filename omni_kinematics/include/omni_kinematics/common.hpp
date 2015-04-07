#ifndef OMNI_KINEMATICS_COMMON_HPP
#define OMNI_KINEMATICS_COMMON_HPP

namespace omni_kinematics
{
    static const double PI = 3.14159;

    struct Twist
    {
        Twist(): xd(0), yd(0), td(0) {}

        double xd; // x_dot
        double yd; // y_dot
        double td; // theta_dot
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

}



#endif

