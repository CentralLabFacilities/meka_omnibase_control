#! /usr/bin/python

import time
import m3.gui as m3g
import m3.rt_proxy as m3p
import m3.actuator as m3c
import m3.actuator_ec as m3ec
import m3.joint as m3
import m3.toolbox as m3t
import m3.unit_conversion as m3u
import m3.pwr
import m3.component_factory as m3f
import m3.actuator_pb2 as mrt 
import m3.meka_omnibase_control as m3o
import math
from m3.component_factory import create_component
from pprint import pprint

class M3Proc:
    def __init__(self):
        self.proxy = m3p.M3RtProxy()
        self.gui = m3g.M3Gui(stride_ms=125)#125
    def stop(self):
        self.proxy.stop()
    def start(self):
        self.proxy.start()

        self.omni = m3o.MekaOmnibaseControl('meka_omnibase_control_mb2', 'meka_omnibase_control')
        self.proxy.subscribe_status(self.omni)
        self.proxy.publish_param(self.omni)
        self.proxy.publish_command(self.omni)

        joint_names=self.proxy.get_joint_components()
        if len(joint_names)==0:
            print 'No joint components available'
            self.proxy.stop()
            exit()
        joint_names= ['m3joint_mb2_j0',
                      'm3joint_mb2_j1',
                      'm3joint_mb2_j2',
                      'm3joint_mb2_j3',
                      'm3joint_mb2_j4',
                      'm3joint_mb2_j5',
                      'm3joint_mb2_j6',
                      'm3joint_mb2_j7']

        actuator_ec_names=[]
        actuator_names=[]
        ctrl_names=[]
        for n in joint_names:
            ctrl = m3t.get_joint_ctrl_component_name(n)
            if ctrl != "":
                ctrl_names.append(ctrl)
            actuator = m3t.get_joint_actuator_component_name(n)
            if actuator != "":
                actuator_names.append(actuator)
                actuator_ec = m3t.get_actuator_ec_component_name(actuator)
                if actuator_ec != "":
                    actuator_ec_names.append(actuator_ec)

       

        self.joint=[]
        self.actuator_ec=[]
        self.actuator=[]
        self.ctrl=[]

        for n in actuator_ec_names:
            c=m3f.create_component(n)
            if c is not None:
                try:
                    self.actuator_ec.append(c)
                    self.proxy.subscribe_status(self.actuator_ec[-1])
                    self.proxy.publish_param(self.actuator_ec[-1]) 
                except:
                    print 'Component',n,'not available'

        for n in actuator_names:
            c=m3f.create_component(n)
            if c is not None:
                self.actuator.append(c)
                self.proxy.subscribe_status(self.actuator[-1])
                self.proxy.publish_param(self.actuator[-1]) 
                
        for n in ctrl_names:
            c=m3f.create_component(n)
            if c is not None:
                self.ctrl.append(c)
                self.proxy.subscribe_status(self.ctrl[-1])
                self.proxy.publish_param(self.ctrl[-1]) 

        for n in joint_names:
            c=m3f.create_component(n)
            if c is not None:
                self.joint.append(c)
                self.proxy.subscribe_status(self.joint[-1])
                self.proxy.publish_command(self.joint[-1])
                self.proxy.publish_param(self.joint[-1]) 

        #Enable motor power
        pwr_names = self.proxy.get_available_components('m3pwr')
        self.pwr = [0]*len(pwr_names)
        for i in range(0,len(pwr_names)):
            print i, pwr_names[i]
            pwr_rt = pwr_names[i]
            self.pwr[i]=m3f.create_component(pwr_rt)
            if self.pwr[i] is not None:
                self.proxy.publish_command(self.pwr[i])
                self.pwr[i].set_motor_power_on()

        #Start them all up
        self.proxy.make_operational_all()

        #Force safe-op of robot, etc are present
        types=['m3humanoid','m3hand','m3gripper']
        for t in types:
            cc=self.proxy.get_available_components(t)
            for ccc in cc:
                self.proxy.make_safe_operational(ccc)

        #Force safe-op of chain so that gravity terms are computed
        self.chain=None    
        if len(joint_names)>0:
            for j in joint_names:
                chain_name=m3t.get_joint_chain_name(j)
                if chain_name!="":
                    self.proxy.make_safe_operational(chain_name)
                    #self.chain=m3f.create_component(chain_name)
                    #self.proxy.publish_param(self.chain) #allow to set payload
                    #Force safe-op of chain so that gravity terms are computed
                    dynamatics_name = m3t.get_chain_dynamatics_component_name(chain_name)
                    if dynamatics_name != "":        
                        self.proxy.make_safe_operational(dynamatics_name)
                        self.dyn=m3f.create_component(dynamatics_name)
                        self.proxy.publish_param(self.dyn) #allow to set payload


        #Force safe-op of robot so that gravity terms are computed
        robot_name = m3t.get_robot_name()
        if robot_name != "":
            try:
                self.proxy.make_safe_operational(robot_name)
                self.robot=m3f.create_component(robot_name)
                self.proxy.subscribe_status(self.robot)
                self.proxy.publish_param(self.robot) #allow to set payload  
            except:
                print 'Component',robot_name,'not available'  

        ## Plots
        self.scope_torque=[]
        self.scope_theta=[]
        self.scope_thetadot=[]
        self.scope_thetadotdot=[]
        self.scope_torquedot=[]
        
        for i,name in zip(xrange(len(joint_names)),joint_names):
            self.scope_torque.append(       m3t.M3ScopeN(xwidth=100,yrange=None,title='Torque')     )
            self.scope_theta.append(        m3t.M3ScopeN(xwidth=100,yrange=None,title='Theta')      )
            self.scope_thetadot.append(     m3t.M3ScopeN(xwidth=100,yrange=None,title='ThetaDot')   )
            self.scope_thetadotdot.append(  m3t.M3ScopeN(xwidth=100,yrange=None,title='ThetaDotDot'))
            self.scope_torquedot.append(    m3t.M3ScopeN(xwidth=100,yrange=None,title='TorqueDot')  )
            
        #Create gui

        # Velocity limits (in rad/s)
        vmin = -6.0
        vmax =  6.0

        try:
            self.mode   = [0]
            self.betad_ = [0.0]*4
            self.phid_  = [0.0]*4
            self.xd_    = [0.0]
            self.yd_    = [0.0]
            self.td_    = [0.0]
            self.tqr_   = [0.0]*4

            self.do_scope_torque=False
            self.do_scope_torquedot=False
            self.do_scope_theta=False
            self.do_scope_thetadot=False
            self.do_scope_thetadotdot=False
            self.status_dict=self.proxy.get_status_dict()
            self.param_dict=self.proxy.get_param_dict()
            self.gui.add('M3GuiTree',   'Status',    (self,'status_dict'),[],[],m3g.M3GuiRead,column=2)
            self.gui.add('M3GuiTree',   'Param',   (self,'param_dict'),[],[],m3g.M3GuiWrite,column=3)
            self.gui.add('M3GuiModes',  'Mode',
                    (self,'mode'),[0],[['Off','CC', 'Normal'],1],m3g.M3GuiWrite)
            self.gui.add('M3GuiSliders','Beta dot', 
                    (self,'betad_'),range(0,4),[vmin,vmax]*4,m3g.M3GuiWrite)
            self.gui.add('M3GuiSliders','Phi dot',
                    (self,'phid_'),range(0,4),[vmin,vmax]*4,m3g.M3GuiWrite)
            self.gui.add('M3GuiSliders','Caster Ratio',
                    (self,'tqr_'),range(0,4),[0.0,1.0]*4,m3g.M3GuiWrite)
            self.gui.add('M3GuiSliders','xdot', 
                    (self,'xd_'),[0],[-0.3,0.3],m3g.M3GuiWrite)
            self.gui.add('M3GuiSliders','ydot', 
                    (self,'yd_'),[0],[-0.3,0.3],m3g.M3GuiWrite)
            self.gui.add('M3GuiSliders','tdot', 
                    (self,'td_'),[0],[-1.0,1.0],m3g.M3GuiWrite)

            self.gui.add('M3GuiToggle', 'ScopeTorque',      (self,'do_scope_torque'),[],[['On','Off']],m3g.M3GuiWrite)
            self.gui.add('M3GuiToggle', 'ScopeTorqueDot',      (self,'do_scope_torquedot'),[],[['On','Off']],m3g.M3GuiWrite)
            self.gui.add('M3GuiToggle', 'ScopeTheta',      (self,'do_scope_theta'),[],[['On','Off']],m3g.M3GuiWrite)
            self.gui.add('M3GuiToggle', 'ScopeThetaDot',      (self,'do_scope_thetadot'),[],[['On','Off']],m3g.M3GuiWrite)
            self.gui.add('M3GuiToggle', 'ScopeThetaDotDot',      (self,'do_scope_thetadotdot'),[],[['On','Off']],m3g.M3GuiWrite)
            self.gui.start(self.step)
        except Exception,e:
            print "ERROR: Could not properly create GUI:", e
            return

    def step(self):
        self.proxy.step()
        self.status_dict=self.proxy.get_status_dict()
        self.proxy.set_param_from_dict(self.param_dict)
        idx=0
        current=0

        if (self.mode[0] == 1): # CC
            self.omni.set_mode_caster()
            for i in range(0,4):
                self.omni.set_caster_vel(i, self.betad_[i], self.phid_[i])
                self.omni.set_caster_tqr(i, self.tqr_[i])
        elif (self.mode[0] == 2): # Normal
            self.omni.set_mode_on()
            self.omni.set_desired_twist(self.xd_[0], self.yd_[0], self.td_[0])
            for i in range(0,4):
                self.omni.set_caster_tqr(i, self.tqr_[i])
        else:
            self.omni.set_mode_off()

        #self.proxy.pretty_print_component(self.actuator_ec[0].name)
        #self.proxy.pretty_print_component(self.joint[0].name)
        '''if self.do_scope_torque and self.scope_torque is None and len(self.joint)==1:
            self.scope_torque.plot()
    
        if self.do_scope_theta and self.scope_theta is None and len(self.joint)==1:
            self.scope_theta=m3t.M3Scope2(xwidth=100,yrange=None,title='Theta')
    
        if self.do_scope_torquedot and self.scope_torquedot is None and len(self.joint)==1:
            self.scope_torquedot=m3t.M3Scope(xwidth=100,yrange=None,title='TorqueDot')
    
        if self.do_scope_thetadot and self.scope_thetadot is None and len(self.joint)==1:
            self.scope_thetadot=m3t.M3Scope2(xwidth=100,yrange=None,title='ThetaDot')
    
        if self.do_scope_thetadotdot and self.scope_thetadotdot is None and len(self.joint)==1:
            self.scope_thetadotdot=m3t.M3Scope(xwidth=100,yrange=None,title='ThetaDotDot')'''


        for c in self.joint:
            try:
                for scope_theta in self.scope_theta:
                    if self.do_scope_theta and self.scope_theta is not None:
                         scope_theta.plot(c.get_theta_deg())
                         
                for scope_thetadot in self.scope_thetadot:
                    if self.do_scope_thetadot and self.scope_thetadot is not None:
                         scope_thetadot.plot(c.get_thetadot_deg())
    
                for scope_thetadotdot in self.scope_thetadotdot:
                    if self.do_scope_thetadotdot and self.scope_thetadotdot is not None:
                         scope_thetadotdot.plot(c.get_thetadotdot_deg())
    
                for scope_torque in self.scope_torque:
                    if self.do_scope_torque and self.scope_torque is not None:
                         scope_torque.plot(c.get_torque_mNm())
        
                for scope_torquedot in self.scope_torquedot:
                    if self.do_scope_torquedot and self.scope_torquedot is not None:
                         scope_torquedot.plot(c.get_torquedot_mNm())
            except Exception,e:
                print e

if __name__ == '__main__':
    t=M3Proc()
    try:
        t.start()
    except (KeyboardInterrupt,EOFError):
        pass
    t.stop()


