# Copyright (c) 2014, Konstantinos Chatzilygeroudis
# All rights reserved.

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import roslib;roslib.load_manifest('nao_dcm_rqt_dashboard')
import rospy
from nao_dcm_msgs.srv import BoolService, BoolServiceRequest
from std_msgs.msg import Float32

from rqt_robot_dashboard.widgets import IconToolButton

class NaoDCMStiffnessWidget(IconToolButton):
    def __init__(self, name='stiffness'):
        self.name = name
        icons = []
        icons.append(['bg-red.svg','ic-motors.svg'])
        icons.append(['bg-green.svg','ic-motors.svg'])
        super(NaoDCMStiffnessWidget,self).__init__(name=name,icons=icons)
        self.update_state(0)
        
        self.stiffness = 1.0
        self.clicked.connect(self.changeStiffness)
        self._sub = rospy.Subscriber('/nao_dcm/stiffnesses',Float32,self.callback)
        
    def changeStiffness(self):
        stiff = rospy.ServiceProxy('/nao_dcm/Stiffnesses/Enable',BoolService)
        req = BoolServiceRequest()
        if(self.stiffness==1.0):
            req.enable = False
        else:
            req.enable = True
        self.stiffness = 1.0-self.stiffness
        stiff(req)

    def callback(self, msg):
        self.stiffness = msg.data

        if(msg.data == 1.0):
            self.update_state(0)
        else:
            self.update_state(1)