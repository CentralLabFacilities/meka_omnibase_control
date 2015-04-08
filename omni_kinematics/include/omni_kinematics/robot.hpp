#ifndef OMNI_KINEMATICS_ROBOT_HPP
#define OMNI_KINEMATICS_ROBOT_HPP

#include "common.hpp"
#include <vector>

namespace omni_kinematics
{
    /// \brief Robot geometry description and state class.
    ///
    /// Geometry variables:
    /// 
    ///  - l: Distance from the chassis' center and a wheel's pivot point.
    ///  - alpha: Angle between X_r (pointing straight ahead) and the vector
    ///    between the chassis' center and a wheel's pivot point.
    ///  - beta: Angle between a wheel's pivot point and its propulsion
    ///    axis.
    ///    Measured from the vector formed by alpha and l.
    ///  - r: Wheel radius.
    ///  - d: Wheel caster offset.
    class Robot
    {
    public:
        typedef std::vector<double> VectorType;

        /// \brief Constructor.
        ///
        /// Sets default values for a symmetrical robot.
        /// Wheel alphas are distributed to all have the same offset.
        /// Velocity constraints (maxTwist()) are generated from max_phid and
        /// max_betad.
        ///
        /// \param nb_wheels Number of wheels (default: 4).
        /// \param l Default l value (default: 0.25).
        /// \param alpha Default alpha of first wheel (default: pi / 8).
        /// \param r Default r value (default: 0.10).
        /// \param d Default d value (default: 0.05).
        /// \param max_betad Default d value (default: 0.05).
        /// \param max_phid Default maximum phi_dot value (default: 1.00).
        /// \param max_betad Default maximum beta_dot value (default: 1.00).
        Robot(unsigned int nb_wheels  = 4,
              double       l          = 0.25,
              double       alpha      = PI / 4.0,
              double       r          = 0.10,
              double       d          = 0.05,
              double       max_betad  = 1.00,
              double       max_phid   = 1.00,
              double       max_betadd = 1.00,
              double       max_phidd  = 1.00);

        /// \brief Copy constructor.
        Robot(const Robot& r);

        unsigned int nbWheels() const { return nb_wheels_; }

        const VectorType& l() const { return l_; }
        VectorType& l()  { return l_; }
        const VectorType& alpha() const { return alpha_; }
        VectorType& alpha()  { return alpha_; }
        const VectorType& r() const { return r_; }
        VectorType& r()  { return r_; }
        const VectorType& d() const { return d_; }
        VectorType& d()  { return d_; }

        const VectorType& maxBetad() const { return max_betad_; }
        VectorType& maxBetad() { return max_betad_; }
        const VectorType& maxPhid() const  { return max_phid_; }
        VectorType& maxPhid() { return max_phid_; }
        const VectorType& maxBetadd() const { return max_betadd_; }
        VectorType& maxBetadd() { return max_betadd_; }
        const VectorType& maxPhidd() const  { return max_phidd_; }
        VectorType& maxPhidd() { return max_phidd_; }

        /// \brief Maximum linear velocity at the most constrained wheel's
        /// steering pivot.
        ///
        /// In the worst-case scenario, this corresponds to either r * max_phid
        /// or d * max_betad.
        /// This depends on the robot's geometry.
        /// So, the chassis' maximum linear velocity is max_v, and the maximum
        /// angular velocity is max_v / (l + d).
        /// The actual (l + d) value used to calculate max_v can be obtained
        /// with maxLD().
        double maxV() const { return max_v_; }

        /// \breif Maximum linear acceleration at the most constrained wheel's
        /// steering pivot.
        ///
        /// Calculated in the same way as maxV.
        double maxA() const { return max_a_; }

        /// \brief Return the maximum (l + d) value used to calculate velocity
        /// and acceleration constraints.
        double maxLD() const { return max_ld_; }

        /// \brief Evaluate if a given twist is over the worst-case constraints
        /// based on maxV, maxLD and the given current velocity.
        ///
        /// \param cmd Twist to evaluate.
        /// \param cur The current velocity.
        /// \param dt  Time period to use in acceleration limit test.
        /// \param v   Optional pointer to a double that will receive the
        ///            calculated velocity at the worst-case pivot point. 
        /// \return True if the given twist is too fast.
        bool overMaxVel(const Twist&  cmd, 
                        const Twist&  cur, 
                        const double& dt,
                        double*       v = 0) const;

        /// \brief Evaluate if a given twist is over the worst-case constraints,
        /// based on maxV, maxLD and the current velocity in the model.
        bool overMaxVel(const Twist&  cmd, 
                        const double& dt, 
                        double*       v = 0) const;

        /// \brief Evaluate if a given twist is over the worst-case constraints
        /// based on maxV, maxLD and the given current velocity.
        ///
        /// Same as overMaxVel(const Twist&).
        bool overMaxVel(const double& xd,
                        const double& yd,
                        const double& td,
                        const double& cxd,
                        const double& cyd,
                        const double& ctd,
                        const double& dt,
                        double*       v = 0) const;

        /// \brief Evaluate if a given twist is over the worst-case constraints
        /// based on maxV, maxLD and the current velocity in the model.
        ///
        /// Same as overMaxVel(const Twist&).
        bool overMaxVel(const double& xd,
                        const double& yd,
                        const double& td,
                        const double& dt,
                        double*       v = 0) const;

        /// \brief Return the absolute, pre-calculated velocity constraints.
        ///
        /// Note that these values are independent in each dimension, meaning
        /// that maximum velocity is not possible in two or more dimensions
        /// simultaneously.
        const Twist& maxTwist() const { return max_twist_; }

        const VectorType& beta() const { return beta_; }
        const VectorType& betad() const { return betad_; }
        const VectorType& phid() const { return phid_; }

        double xd() const { return twist_.xd; }
        double yd() const { return twist_.yd; }
        double td() const { return twist_.td; }

        /// \brief Return the current measured twist.
        const Twist& twist() const { return twist_; }

        /// \brief Return the accumulated global pose.
        ///
        /// Calculated by integrating the current twist on each updateState
        /// cycle.
        const Pose&  pose()  const { return pose_; }

        /// \brief Calculate the constraints based on max_betad_, max_phid_
        /// and the robot's geometry.
        ///
        /// Has to be called if any geometry values changed, but is called
        /// automatically by the default values constructor.
        void calcConstraints();

        /// \brief Update the internal state and recalculate velocity.
        ///
        /// Parameter dt is the time delta (in second) since last update (used
        /// to accumulate the pose in the global frame).
        void updateState(
            const VectorType& beta, 
            const VectorType& betad, 
            const VectorType& phid,
            const double      dt);

        /// \brief Reset the pose in the global frame to (0,0,0).
        void resetPose() { pose_ = Pose(); }

        /// \brief Indicates if the robot's wheels are considered parallel or 
        /// not.
        ///
        /// \param angle Will contain the average absolute angle of the wheels
        /// in the robot's frame (optional).
        /// \param eps Threshold for parallelism test. 
        /// \return True if the angle difference is below threshold.
        bool isParallel(double* angle = 0, const double eps = 0.1) const; 

    private:
        /// \brief Calculate velocity based on the current internal state.
        void calcVelocity();

        // Static geometry
        unsigned int nb_wheels_;
        VectorType l_;
        VectorType alpha_;
        VectorType r_;
        VectorType d_;
        
        // Constraints
        VectorType max_betad_;
        VectorType max_phid_;
        VectorType max_betadd_;
        VectorType max_phidd_;
        double     max_v_;      // Maximum linear velocity at a wheel's pivot.
        double     max_a_;      // Maximum linear acceleration, same pivot.
        double     max_ld_;
        Twist      max_twist_;

        // Joint states
        VectorType beta_;
        VectorType betad_;
        VectorType phid_;

        // Velocity
        Twist twist_;

        // Pose in the global frame
        Pose pose_;

    };
}

#endif

