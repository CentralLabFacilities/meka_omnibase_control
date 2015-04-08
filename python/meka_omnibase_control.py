import m3.toolbox as m3t
from m3.component import M3Component
import meka_omnibase_control_pb2 as mob

class MekaOmnibaseControl(M3Component):
    
    def __init__(self, name, type):
        M3Component.__init__(self, name, type)

        self.num_casters = 4

        self.command = mob.MekaOmnibaseControlCommand()
        self.status  = mob.MekaOmnibaseControlStatus()
        self.param   = mob.MekaOmnibaseControlParam()

        for i in range(3):
            self.command.xd_des.append(0)
        self.command.ctrl_mode = mob.MEKA_OMNIBASE_CONTROL_OFF

    def get_current_twist(self):
        return (self.status.l_vel[0],
                self.status.l_vel[1],
                self.status.l_vel[2])

    def get_global_position(self):
        return (self.status.g_pos[0],
                self.status.g_pos[1],
                self.status.g_pos[2])

    def set_mode_on(self):
        self.command.ctrl_mode = mob.MEKA_OMNIBASE_CONTROL_ON
    def set_mode_off(self):
        self.command.ctrl_mode = mob.MEKA_OMNIBASE_CONTROL_OFF

    def set_desired_twist(self, xd, yd, td):
        self.command.xd_des[0] = xd
        self.command.xd_des[1] = yd
        self.command.xd_des[2] = td
