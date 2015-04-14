#ifndef CASTER_CONTROL_HPP
#define CASTER_CONTROL_HPP

namespace meka_omnibase_control
{
    /// \brief Meka Omnibase caster controller.
    ///
    /// Takes desired joint velocities (steer and roll, or beta_dot and phi_dot)
    /// and output control torques for each motor.
    /// The gear train joining the two holomni motors and the caster are
    /// is in a differential (possibly an epicyclic gear train?) configuration
    /// obeying to this:
    ///
    ///   q_dot = [ beta_dot, phi_dot ]^T,
    ///
    ///   q_dot = -1 * N * e_dot,
    ///
    ///   N     = [ -Ns, 0; -Nt, Nt*Nw ] 
    ///
    /// and
    ///
    ///   e_dot = -1 * N^-1 * q_dot
    ///
    ///   N^-1  = [ -1/Ns, 0; -1/(Ns*Nw), 1/(Nt*Nw) ]
    ///
    /// Where e_dot are the motor velocities and [Ns,Nw,Nt] are gear ratios.
    /// The equations have been extracted from the original, undocumented PCV
    /// control code.
    ///
    /// Note that the steer angle (beta) is not managed from this controller, as
    /// it is obtained from a different sensor.
    ///
    class CasterControl
    {
    private:
        double ns_;
        double nw_;
        double nt_;
        double kp_;

        double e_[2];   // current motor angles.
        double ed_[2];  // current motor velocities.
        double q_[2];   // Current joint angles
        double qd_[2];  // Current joint velocities.
        double tq_[2];  // Current torque output.

    public:
        /// \brief Constructor.
        ///
        /// \param ns Gear ratio (see original PCV config). 
        /// \param nw Gear ratio (see above).
        /// \param nt Gear ratio (see above).
        /// \param kp Proportional gain for torque output.
        CasterControl(double ns, double nw, double nt, double kp);

        /// \brief Default constructor.
        CasterControl(): ns_(1), nw_(1), nt_(1), kp_(1) {}

        /// \brief Update the current joint velocities from the motor
        ///        velocities.
        void stepStatus(double e[2], double ed[2]);

        /// \brief Update the current command based on the desired joint
        ///        velocities.
        void stepCommand(double qd_des_0, double qd_des_1);

        /// \brief Copy the current torque to the referenced array.
        void tq(double& tq_0, double& tq_1)
        {
            tq_0 = tq_[0];
            tq_1 = tq_[1];
        }

        /// \brief Copy the current joint angles to the referenced array.
        void q(double& q_0, double& q_1)
        {
            q_0 = q_[0];
            q_1 = q_[1];
        }

        /// \brief Copy the current joint velocities to the referenced array.
        void qd(double& qd_0, double& qd_1)
        {
            qd_0 = qd_[0];
            qd_1 = qd_[1];
        }
    };
}

#endif
