/**
Copyright (c) 2014, Konstantinos Chatzilygeroudis
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer 
    in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived 
    from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#ifndef NAO_CAMERA_H
#define NAO_CAMERA_H

// Boost Headers
#include <boost/shared_ptr.hpp>
#include <boost/assign.hpp>
#include <boost/thread/mutex.hpp>

// NAOqi Headers
#include <alcommon/almodule.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <qi/os.hpp>

// ROS Headers
#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <nao_dcm_msgs/BoolService.h>

#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>

#include <camera_info_manager/camera_info_manager.h>

#include <dynamic_reconfigure/server.h>
#include <nao_dcm_camera/NaoDCMCameraConfig.h>

using std::string;
using std::cerr;
using std::endl;

namespace AL
{
class ALBroker;
}

class NaoCamera : public AL::ALModule
{
private:
    // ROS Standard Variables
    ros::NodeHandle node_handle_, top_node_handle_, bottom_node_handle_;

    // ROS Topics/Messages
    image_transport::CameraPublisher camera_top_image_pub_, camera_bottom_image_pub_;
    sensor_msgs::CameraInfo camera_top_, camera_bottom_;
    sensor_msgs::Image camera_top_image_, camera_bottom_image_;

    ros::ServiceServer camera_top_switch_, camera_bottom_switch_;

    boost::shared_ptr<camera_info_manager::CameraInfoManager> camera_top_info_, camera_bottom_info_;

    // ROS Dynamic Reconfigure
    dynamic_reconfigure::Server<nao_dcm_camera::NaoDCMCameraConfig> dynamic_req_server_;
    nao_dcm_camera::NaoDCMCameraConfig cameras_config_;
    volatile bool reconfiguring_;

    // Member Variables

    // Camera Specific
    bool camera_top_enabled_, camera_bottom_enabled_, camera_enabled_;

    // Helper
    bool is_connected_;
    int topic_queue_;
    string prefix_;

    // AL Proxies
    AL::ALVideoDeviceProxy video_proxy_;
public:
    // Constructor/Destructor
    NaoCamera(boost::shared_ptr<AL::ALBroker> broker, const std::string& name);
    ~NaoCamera();

    bool initialize();

    // Connect/Disconnet to ALProxies
    bool connect(const ros::NodeHandle nh, const ros::NodeHandle top, const ros::NodeHandle bottom);
    void disconnect();

    // Subscribe to ROS Topics/Services
    void subscribe();

    // Parameter Server
    void loadParams();

    // Helper
    void brokerDisconnected(const string& event_name, const string &broker_name, const string& subscriber_identifier);
    string getURLFromConfiguration(const string& camera, const int& resolution, const int &color = 0);

    // ALVideoDevice Methods
    AL::ALValue getImage(const string &cam_module="NaoCameraTop");

    // General Methods
    void loop(const ros::Time& time, const ros::Duration& period);

    bool connected();

    // ROS Callbacks/Related Methods

    void publishCameras(const ros::Time &ts);

    bool switchCameraTopState(nao_dcm_msgs::BoolService::Request &req, nao_dcm_msgs::BoolService::Response &res);
    bool switchCameraBottomState(nao_dcm_msgs::BoolService::Request &req, nao_dcm_msgs::BoolService::Response &res);

    void dynamicReconfigureCb(nao_dcm_camera::NaoDCMCameraConfig &config, uint32_t level);

};

#endif // NAO_H
