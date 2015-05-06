#ifndef MEKA_OMNIBASE_CONTROL_HPP
#define MEKA_OMNIBASE_CONTROL_HPP

#include <m3rt/base/component_factory.h>
#include <m3rt/base/component.h>
#include <m3/chains/joint_array.h>
#include <m3/hardware/pwr.h>
#include <omni_kinematics/robot.hpp>
#include <omni_kinematics/control.hpp>
#include <meka_omnibase_control/caster_control.hpp>
#include "meka_omnibase_control.pb.h"

namespace meka_omnibase_control
{
    static const int NUM_CASTERS = 4; 

    class MekaOmnibaseControl: public m3rt::M3Component
    {
    private:
        MekaOmnibaseControlCommand              command_;
        MekaOmnibaseControlStatus               status_;
        MekaOmnibaseControlParam                param_;

        omni_kinematics::Robot                  robot_;
        omni_kinematics::MotionControl          ctrl_;

        CasterControl                           casters_[NUM_CASTERS];

        // NOTE: beta offsets only used at init, see param_.
        double                                  beta_offset_[NUM_CASTERS];
        double                                  beta_ratio_[NUM_CASTERS];
        double                                  phid_ratio_[NUM_CASTERS];
        double                                  des_betad_ratio_[NUM_CASTERS];
        double                                  des_phid_ratio_[NUM_CASTERS];

        std::string                             m3joints_name_;
        std::string                             m3pwr_name_;

        m3::M3JointArray*                       m3joints_;
        m3::M3Pwr*                              m3pwr_;

        int                                     last_ctrl_mode_;

        int                                     cycle_;
        int                                     unstable_start_[NUM_CASTERS];

    public:
        MekaOmnibaseControl():
            m3rt::M3Component(ROBOT_PRIORITY),
            robot_(NUM_CASTERS),
            ctrl_(&robot_)
        {
            RegisterVersion("default", DEFAULT);
            RegisterVersion("iss",     ISS);
        }

        google::protobuf::Message* GetCommand() { return &command_; }
        google::protobuf::Message* GetStatus()  { return &status_; }
        google::protobuf::Message* GetParam()   { return &param_; }

        M3BaseStatus* GetBaseStatus()
        {
            return status_.mutable_base();
        }

    private:
        enum {DEFAULT, ISS};

        bool ReadConfig(const char* filename);
        bool LinkDependentComponents();

        void Startup();
        void Shutdown();
        void StepStatus();
        void StepCommand();

        // Tells if the caster is stable (or has been stable for at least 1000
        // cycles).
        bool casterStable(int i);

    };
}

#endif

