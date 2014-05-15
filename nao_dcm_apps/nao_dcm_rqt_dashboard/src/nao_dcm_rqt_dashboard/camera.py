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

from python_qt_binding.QtCore import Signal, QMutex, QMutexLocker
from python_qt_binding.QtGui import QWidget, QPixmap, QImage, QGraphicsView, QGraphicsScene, QVBoxLayout

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rqt_robot_dashboard.widgets import IconToolButton
import numpy

class CameraViewDashWidget(IconToolButton):
    def __init__(self, context, camera_topic='/image_raw', name='camera'):
        self._icons = [['bg-grey.svg', 'ol-play-badge.svg']]
        super(CameraViewDashWidget, self).__init__("View"+name, icons=self._icons, icon_paths=None)
        self.context = context
        self.update_state(0)
        
        self._cam_view = CameraViewWidget(camera_topic,name)
        
        self._cam_view_shown = False
        self.clicked.connect(self._show_cam_view)
        self._show_mutex = QMutex()

    def _show_cam_view(self):
        with QMutexLocker(self._show_mutex):
            try:
                if self._cam_view_shown:
                    self.context.remove_widget(self._cam_view)
                    self._cam_view_shown = not self._cam_view_shown
                else:
                    self.context.add_widget(self._cam_view)
                    self._cam_view_shown = not self._cam_view_shown
            except Exception:
                self._cam_view_shown = not self._cam_view_shown
                self._show_cam_view()

    def shutdown_widget(self):
        if self._cam_view:
            self._cam_view.close()
    

class CameraViewWidget(QWidget):
    def __init__(self, camera_topic='/image_raw', name='camera'):
        super(CameraViewWidget,self).__init__()
        
        self._layout = QVBoxLayout()
        self.name = name
        self._colors = [(238, 34, 116), (68, 134, 252), (236, 228, 46), (102, 224, 18), (242, 156, 6), (240, 64, 10), (196, 30, 250)]        
        
        
        self.setWindowTitle('Camera \''+name+'\' Viewer')
        
        self._cam_view = CameraView(camera_topic)
        
        self._layout.addWidget(self._cam_view)
        
        self.setLayout(self._layout)
    
    def close(self):
        if self._cam_view:
            self._cam_view.close()
        

class CameraView(QGraphicsView):
    image_changed = Signal()
    
    def __init__(self, camera_topic='/image_raw'):
        super(CameraView,self).__init__()
        
        self._scene = QGraphicsScene()
        self.bridge = CvBridge()
        self._map_item = 0
        
        self.image_changed.connect(self._update)
        
        self._sub = rospy.Subscriber(camera_topic,Image,self.callback)
        
        self.setScene(self._scene)
        
    def callback(self, msg):
        self.w = msg.width
        self.h = msg.height
        
        a = self.bridge.imgmsg_to_cv(msg, "rgb8")
        a = numpy.array(a)
        image = QImage(a, self.w, self.h, QImage.Format_RGB888)
        self._map = image
        self._scene.setSceneRect(0,0,self.w,self.h)
        self.image_changed.emit()

    def _update(self):
        if self._map_item:
            self._scene.removeItem(self._map_item)
        pixmap = QPixmap.fromImage(self._map)

        self._map_item = self._scene.addPixmap(pixmap)

        self.centerOn(self._map_item)
        self.show()
        

    def _mirror(self, item):
        item.scale(-1, 1)
        item.translate(-self.w, 0)
        
    def close(self):
        if self._sub:
            self._sub.unregister()
        super(CameraView, self).close()