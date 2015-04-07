#ifndef MEKA_OMNIBASE_CONTROL_HPP
#define MEKA_OMNIBASE_CONTROL_HPP

#include <m3rt/base/component_factory.h>
#include <m3rt/base/component.h>
#include <omni_kinematics/robot.hpp>
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

    public:
        MekaOmnibaseControl():
            m3rt::M3Component(ROBOT_PRIORITY),
            robot_(NUM_CASTERS)
        {
            RegisterVersion("default", DEFAULT);
            RegisterVersion("iss",     ISS);
        }

        google::protobuf::Message* GetCommand() { return &command_; }
        google::protobuf::Message* GetStatus()  { return &status_; }
        google::protobuf::Message* GetParam()   { return &param_; }

    private:
        enum {DEFAULT, ISS};

        bool ReadConfig(const char* filename);
        bool LinkDependentComponents();

        void Startup();
        void Shutdown();
        void StepStatus();
        void StepCommand();

    };
}

#endif

