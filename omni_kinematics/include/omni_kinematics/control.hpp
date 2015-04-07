#ifndef CONTROL_HPP
#define CONTROL_HPP

#include "robot.hpp"

namespace omni_kinematics
{
    /// \brief Motion control class for the mobile platform.
    ///
    /// Produces the ideal beta_dot and phi_dot command based on a robot's
    /// definition and desired Twist.
    class MotionControl 
    {
    public:
        /// \brief Constructor
        ///
        /// \param robot A pointer to a robot model and state.
        MotionControl(const Robot* robot);

        /// \brief Saturate a twist according to the robot's state and actuator
        /// contraints.
        ///
        /// Scale the given twist command linearly on all axes if a constraint
        /// is not respected.
        /// Go through each wheel's set of constraints.
        /// Currently only consider velocity constraints for both steering and
        /// propulsion axes.
        ///
        /// \param twist      The command to saturate.
        /// \param dt         Time period to use in acceleration limit test.
        /// \param worst_case Only saturate based on worst-case velocity
        ///                   constraints.
        ///                   This is useful to avoid slowdowns, for instance 
        ///                   when a wheel has to perform a 180 degrees motion, 
        ///                   as the worst-case calculations consider these 
        ///                   possibilities.
        void saturateTwist(Twist&     cmd, 
                           double     dt, 
                           const bool worst_case = false) const;

        /// \brief Produce a mobile base command (beta_dot, phi_dot) from a
        /// given twist.
        ///
        /// Both vectors given will be resized / cleared.
        void calcCommand(const Twist& twist, 
            Robot::VectorType& betad, Robot::VectorType& phid) const;

    private:
        const Robot* robot_;

    };
}

#endif

