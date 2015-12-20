#!/usr/bin/env python
'''
/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Created for the Pi Robot Project: http://www.pirobot.org
 *  Created for the XM Robot Project: http://www.github/xmproject
 *  Copyright (c) 2012 Patrick Goebel.  All rights reserved.
 *  Copyright (c) 2015 The XM Robot Team. All rights reserved
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of XM Robot Project nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
'''

# Description: Relax all servos by disabling the torque and setting the speed
# and torque limit to a moderate values.

# Create Date: 2015.11.1

# Authors: Patrick Goebel, myyerrol


import rospy
import time
from dynamixel_controllers.srv import TorqueEnable, SetTorqueLimit, SetSpeed


class Relax():
    def __init__(self):
        rospy.init_node('relax_all_servos')

        # The namespace and joints parameter needs to be set by the servo controller
        # (The namespace is usually null.)
        namespace = rospy.get_namespace()
        self.joints = rospy.get_param(namespace + '/joints', '')

        default_dynamixel_speed = rospy.get_param('~default_dynamixel_speed', 0.5)
        default_dynamixel_torque = rospy.get_param('~default_dynamixel_torque', 0.5)

        speed_services = list()
        torque_services = list()
        set_torque_limit_services = list()

        for controller in sorted(self.joints):
            torque_service = '/' + controller + '/torque_enable'
            rospy.wait_for_service(torque_service)
            torque_services.append(rospy.ServiceProxy(torque_service, TorqueEnable))

            set_torque_limit_service = '/' + controller + '/set_torque_limit'
            rospy.wait_for_service(set_torque_limit_service)
            set_torque_limit_services.append(rospy.ServiceProxy(set_torque_limit_service, SetTorqueLimit))

            speed_service = '/' + controller + '/set_speed'
            rospy.wait_for_service(speed_service)
            speed_services.append(rospy.ServiceProxy(speed_service, SetSpeed))

        # Set the default speed to something small
        for set_speed in speed_services:
            try:
                set_speed(default_dynamixel_speed)
            except:
                pass

        # Set the torque limit to a moderate value
        for set_torque_limit in set_torque_limit_services:
            try:
                set_torque_limit(default_dynamixel_torque)
            except:
                pass

        # Relax all servos to give them a rest.
        for torque_enable in torque_services:
            try:
                torque_enable(False)
            except:
                pass


if __name__=='__main__':
    Relax()
