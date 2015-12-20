#!/usr/bin/env python
"""
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
"""

# Description: Relax all servos by disabling the torque.

# Create Date: 2015.11.1

# Authors: Patrick Goebel, myyerrol


import rospy
from std_srvs.srv import Empty
from arbotix_msgs.srv import SetSpeed

class Relax():
    def __init__(self):
        rospy.init_node('relax_all_servos')

        # The list of joints is stored in the /arbotix/joints parameter
        self.joints = rospy.get_param('/arbotix/joints', '')

        # Set a moderate default servo speed
        default_speed = rospy.get_param('~default_speed', 0.5)

        # A list to hold the relax services
        relax_services = list()

        # A list to hold the set_speed services
        set_speed_services = list()

        # Connect to the relax and set_speed service for each joint
        for joint in self.joints:
            relax_service = '/' + joint + '/relax'
            rospy.wait_for_service(relax_service)
            relax_services.append(rospy.ServiceProxy(relax_service, Empty))

            speed_service = '/' + joint + '/set_speed'
            rospy.wait_for_service(speed_service)
            set_speed_services.append(rospy.ServiceProxy(speed_service, SetSpeed))

        # Set the servo speed to the default value
        for set_speed in set_speed_services:
            set_speed(default_speed)

        # Relax each servo
        for relax in relax_services:
            relax()

        # Do it again just to be sure
        for relax in relax_services:
            relax()

if __name__=='__main__':
    try:
        Relax()
        rospy.loginfo("All servos relaxed.")
    except rospy.ROSInterruptException:
        rospy.loginfo("Oops! Exception occurred while trying to relax servos...")
