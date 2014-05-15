# Copyright (c) 2014, Konstantinos Chatzilygeroudis
# All rights reserved.

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer 
#     in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived 
#     from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
# SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import roslib;roslib.load_manifest('nao_dcm_rqt_dashboard')
import rospy

import diagnostic_msgs

from rqt_robot_dashboard.dashboard import Dashboard
from rqt_robot_dashboard.widgets import MonitorDashWidget, ConsoleDashWidget, MenuDashWidget, IconToolButton, NavViewDashWidget
from QtGui import QMessageBox, QAction
from python_qt_binding.QtCore import QSize

from .battery import NaoDCMBatteryWidget
from .camera import CameraViewDashWidget
from .stiffness import NaoDCMStiffnessWidget

import rospkg
import os.path

rp = rospkg.RosPack()

class NaoDCMDashboard(Dashboard):
    def setup(self, context):
        self._dashboard_message = None
        self._last_dashboard_message_time = 0.0
        self.name = "NaoDCM Dashboard"

        rospy.sleep(0.5)

        self.motor_bat = NaoDCMBatteryWidget("Battery")
        self.console = ConsoleDashWidget(self.context)
        self.monitor = MonitorDashWidget(self.context)
        self.monitor._show_monitor()
        self.nav = NavViewDashWidget(self.context)
        self.cam = CameraViewDashWidget(self.context, 'nao_dcm/CameraTop/image_raw', 'CameraTop')
        self.cam._show_cam_view()
        self.camBot = CameraViewDashWidget(self.context, 'nao_dcm/CameraBottom/image_raw', 'CameraBottom')
        self.camBot._show_cam_view()
        self.stiff = NaoDCMStiffnessWidget("Stiffness")

        # This is what gets dashboard_callback going eagerly
        self._dashboard_agg_sub = rospy.Subscriber('diagnostics_agg', diagnostic_msgs.msg.DiagnosticArray, self.dashboard_callback)

    def get_widgets(self):
        return [[self.monitor, self.console],[self.motor_bat],[self.nav],[self.cam, self.camBot],[self.stiff]]

    def dashboard_callback(self, msg):
        self._dashboard_message = msg
        self._last_dashboard_message_time = rospy.get_time()
  
        battery_status = {}  # Used to store Battery status info
        for status in msg.status:
            if status.name == "/Nao/Battery/Battery":
                for value in status.values:
                    battery_status[value.key]=value.value

        # If battery diagnostics were found, calculate percentages and stuff  
        if (battery_status):
            self.motor_bat.set_power_state(battery_status['Battery Charge'])

    def shutdown_dashboard(self):
        self._dashboard_agg_sub.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        self.console.save_settings(plugin_settings, instance_settings)
        self.monitor.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self.console.restore_settings(plugin_settings, instance_settings)
        self.monitor.restore_settings(plugin_settings, instance_settings)
