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

if __name__ == "__main__":
    moc = MekaOmnibaseControl("meka_omnibase_control_mb2", "meka_omnibase_control")

