#!/usr/bin/env python

# Enables breakbeam sensors on each caster and wait for the relevant actuators
# to be calibrated.

import m3.rt_proxy as m3p
import m3.meka_omnibase_control as m3o
import m3.component_factory  as m3f
import math
import numpy
import time

if __name__=='__main__':

    proxy = m3p.M3RtProxy()
    proxy.start()

    pwr_name = proxy.get_available_components('m3pwr')
    pwr  = m3f.create_component(pwr_name[1])
    omni = m3o.MekaOmnibaseControl('meka_omnibase_control_mb2', 'meka_omnibase_control')

    proxy.make_operational_all()

    proxy.subscribe_status(omni)
    proxy.publish_command(omni)
    proxy.subscribe_status(pwr)
    proxy.publish_command(pwr)

    #pwr.set_motor_power_on()

    proxy.step()


    print "Omnibase calibration - Please slowly move the mobile base so that", \
          "each caster makes a full turn."
    print "The emergency stop button can stay pressed."
    print "Press enter to start..."
    raw_input()

    calib_count = 0
    try:
        while (calib_count != 4):
            calib_count = 0
            calib = [False,False,False,False]
            for i in range(0,4):
                calib[i] = omni.status.calib[i]
                if (calib[i]):
                    calib_count = calib_count + 1
            print "Calibration status:", calib
            proxy.step()
            time.sleep(0.2)

        print "All calibrated, stopping proxy..."

    except Exception as e:
        print "Error:", e
        print "Cancelling and stopping proxy..."

    proxy.stop()
    print "Done." 




    

