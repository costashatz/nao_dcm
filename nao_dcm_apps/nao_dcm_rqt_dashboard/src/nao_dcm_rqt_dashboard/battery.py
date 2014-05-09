# Copyright (c) 2014, Konstantinos Chatzilygeroudis
# All rights reserved.

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import roslib;roslib.load_manifest('nao_dcm_rqt_dashboard')
import rospy

from rqt_robot_dashboard.widgets import BatteryDashWidget

class NaoDCMBatteryWidget(BatteryDashWidget):
    def __init__(self, name='battery'):
        icons = []
        charge_icons = []
        icons.append(['ic-battery-0.svg'])
        icons.append(['ic-battery-20.svg'])
        icons.append(['ic-battery-40.svg'])
        icons.append(['ic-battery-60-green.svg'])
        icons.append(['ic-battery-80-green.svg'])
        icons.append(['ic-battery-100-green.svg'])
        charge_icons.append(['ic-battery-charge-0.svg'])
        charge_icons.append(['ic-battery-charge-20.svg'])
        charge_icons.append(['ic-battery-charge-40.svg'])
        charge_icons.append(['ic-battery-charge-60-green.svg'])
        charge_icons.append(['ic-battery-charge-80-green.svg'])
        charge_icons.append(['ic-battery-charge-100-green.svg'])
        super(NaoDCMBatteryWidget, self).__init__(name=name, icons=icons, charge_icons=charge_icons)

        self._pct = 0.0
        self._vlt = 0.0

    def set_power_state(self, msg):
        self._pct = float(msg)
        
        self.update_perc(self._pct)
        self.update_time(self._pct)
