#ifndef CASTER_CONTROL_HPP
#define CASTER_CONTROL_HPP

#include <m3/toolbox/toolbox.h>

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
        double ki_;
        double kd_;
        double ki_range_;
        double ki_limit_;

        double e_[2];   // current motor angles.
        double ed_[2];  // current motor velocities.
        double edd_[2]; // current motor accelerations.
        double q_[2];   // Current joint angles
        double qd_[2];  // Current joint velocities.
        double tq_[2];  // Current torque output.

        m3::M3PID pid_;

    public:
        /// \brief Constructor.
        ///
        /// "readConfig" has to be called (with the proper YAML document) to
        /// fully configure an instance.
        CasterControl();

        /// \brief Read parameters from a YAML node (ns, nw, nt and PID params).
        void readConfig(YAML::Node& doc);

        /// \brief Update the current joint velocities from the motor
        ///        state.
        void stepStatus(double e[2], double ed[2], double edd[2]);

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
