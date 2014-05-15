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

#include <iostream>
#include "nao_dcm_camera/nao_camera.h"
#include <alcommon/albroker.h>
#include <alcommon/albrokermanager.h>

using std::vector;

NaoCamera::NaoCamera(boost::shared_ptr<AL::ALBroker> broker, const string &name)
    : AL::ALModule(broker,name),is_connected_(false),reconfiguring_(false)
{
    functionName("brokerDisconnected", getName(), "Callback when broker disconnects!");
    BIND_METHOD(NaoCamera::brokerDisconnected);
}

NaoCamera::~NaoCamera()
{
    if(is_connected_)
        disconnect();
}

bool NaoCamera::initialize()
{
    dynamic_req_server_.setCallback(boost::bind(&NaoCamera::dynamicReconfigureCb, this, _1, _2));
}

bool NaoCamera::connect(const ros::NodeHandle nh, const ros::NodeHandle top, const ros::NodeHandle bottom)
{
    node_handle_ = nh;
    top_node_handle_ = top;
    bottom_node_handle_ = bottom;
    is_connected_ = false;

    // Loada parameter server parameters
    loadParams();

    // If camera not enabled, skip initialization
    if(!camera_enabled_)
    {
        ROS_WARN("Cameras not enabled! Shutting down NaoCamera node..");
        is_connected_ = true;
        return true;
    }
    try
    {
        video_proxy_ = AL::ALVideoDeviceProxy(getParentBroker());

        if(camera_top_enabled_)
        {
            video_proxy_.subscribeCamera("NaoCameraTop", 0, 1, 11, 30);
            video_proxy_.setActiveCamera("NaoCameraTop",0);
        }

        if(camera_bottom_enabled_)
        {
            video_proxy_.subscribeCamera("NaoCameraBottom", 1, 1, 11, 30);
            video_proxy_.setActiveCamera("NaoCameraBottom",1);
        }
    }
    catch (const AL::ALError& e)
    {
        ROS_ERROR("Failed to connect to Camera Proxy!\n\tTrace: %s",e.what());
        return false;
    }

    // Initialize Cameras Info
    camera_top_info_ = boost::shared_ptr<camera_info_manager::CameraInfoManager>
            (new camera_info_manager::CameraInfoManager(top_node_handle_));
    camera_top_info_->setCameraName("NaoCameraTop");

    camera_bottom_info_ = boost::shared_ptr<camera_info_manager::CameraInfoManager>
            (new camera_info_manager::CameraInfoManager(bottom_node_handle_));
    camera_bottom_info_->setCameraName("NaoCameraBottom");

    // Load Default Camera Configurations
    cameras_config_.top_frame_rate = video_proxy_.getFrameRate(0);
    cameras_config_.top_resolution = video_proxy_.getResolution(0);

    cameras_config_.bottom_frame_rate = video_proxy_.getFrameRate(1);
    cameras_config_.top_resolution = video_proxy_.getResolution(1);

    camera_top_info_->loadCameraInfo(getURLFromConfiguration("top",cameras_config_.top_resolution));
    camera_bottom_info_->loadCameraInfo(getURLFromConfiguration("bottom",cameras_config_.bottom_resolution));

    is_connected_ = true;
    subscribe();

    if(!initialize())
        return false;
    ROS_INFO("NaoCamera Module initialized!");

    return true;
}

void NaoCamera::disconnect()
{
    if(!is_connected_)
        return;
    try
    {
        video_proxy_.unsubscribe("NaoCameraTop");
        video_proxy_.unsubscribe("NaoCameraBottom");
    }
    catch (const AL::ALError& e)
    {
        ROS_ERROR("Failed to unsubscribe from cameras!\n\tTrace: %s",e.what());
    }
    is_connected_ = false;
}

void NaoCamera::loadParams()
{
    ros::NodeHandle n_p("~");
    n_p.param("UseCamera", camera_enabled_, true);
    n_p.param("TopCameraEnabled", camera_top_enabled_, camera_enabled_);
    n_p.param("BottomCameraEnabled", camera_bottom_enabled_, camera_enabled_);

    n_p.param("TopicQueue", topic_queue_, 50);

    n_p.param("Prefix", prefix_, string("nao_dcm"));
    prefix_ = prefix_+"/";
}

void NaoCamera::brokerDisconnected(const string& event_name, const string &broker_name,
                                   const string& subscriber_identifier)
{
    if(broker_name == "Nao Camera Broker")
        is_connected_ = false;
}

string NaoCamera::getURLFromConfiguration(const string &camera, const int &resolution, const int &color)
{
    // Color is not used as we allow only for rgb8 mode of the cameras (we could provide more in the future)

    string res = "640x480";
    switch (resolution) {
    case 0:
        res = "160x120";
        break;
    case 1:
        res = "320x240";
        break;
    case 2:
        res = "640x480";
        break;
    case 3:
        res = "1280x960";
        break;
    default:
        res = "640x480";
        break;
    }
    return "package://nao_dcm_camera/share/camera_"+camera+"_rgb8_"+res+".yaml";
}

void NaoCamera::subscribe()
{
    image_transport::ImageTransport it(node_handle_);

    camera_top_image_pub_ = it.advertiseCamera(prefix_+"CameraTop/image_raw", topic_queue_);

    camera_bottom_image_pub_ = it.advertiseCamera(prefix_+"CameraBottom/image_raw", topic_queue_);

    camera_top_switch_ = node_handle_.advertiseService<NaoCamera, nao_dcm_msgs::BoolService::Request,
            nao_dcm_msgs::BoolService::Response>(prefix_+"CameraTop/Enable", &NaoCamera::switchCameraTopState, this);

    camera_bottom_switch_ = node_handle_.advertiseService<NaoCamera, nao_dcm_msgs::BoolService::Request,
            nao_dcm_msgs::BoolService::Response>(prefix_+"CameraBottom/Enable",
                                                 &NaoCamera::switchCameraBottomState, this);
}

AL::ALValue NaoCamera::getImage(const string& cam_module)
{
    AL::ALValue results;
    bool err=false;
    try
    {
        results = video_proxy_.getImageRemote(cam_module);
    }
    catch(const AL::ALError& e)
    {
        err=true;
        ROS_ERROR("Could not get/write image.\n\tTrace: %s",e.what());
    }
    if(!err)
    {
        try
        {
            video_proxy_.releaseImage(cam_module);
        }
        catch(const AL::ALError& e)
        {
            ROS_ERROR("Could not release image.\n\tTrace: %s",e.what());
        }
    }
    return results;
}

void NaoCamera::loop(const ros::Time &time, const ros::Duration &period)
{
    if(!camera_enabled_)
    {
        is_connected_ = false;
        return;
    }
    if(reconfiguring_)
        return;
    try
    {
        publishCameras(time);
    }
    catch(ros::Exception& e)
    {
        ROS_WARN("NaoCamera Error: %s",e.what());
    }
    catch(AL::ALError& e)
    {
        ROS_ERROR("ALError!\n\tTrace: %s",e.what());
    }

    try
    {
        video_proxy_.ping();
    }
    catch(const AL::ALError& e)
    {
        ROS_ERROR("Could not ping Video proxy.\n\tTrace: %s",e.what());
        is_connected_ = false;
    }
}

bool NaoCamera::connected()
{
    return is_connected_;
}

void NaoCamera::publishCameras(const ros::Time &ts)
{
    if(camera_top_enabled_)
    {
        AL::ALValue im_info = getImage("NaoCameraTop");
        if(im_info.getSize()!=12)
        {
            return;
        }
        const unsigned char* im = static_cast<const unsigned char*>(im_info[6].GetBinary());
        if(im==NULL)
            return;

        int w, h, nbLayers;
        w = (int)im_info[0];
        h = (int)im_info[1];
        nbLayers = (int)im_info[2];
        vector<unsigned char> v(im, (im+w*h*nbLayers));
        camera_top_.header.frame_id = "CameraTop";
        camera_top_.header.stamp = ts;

        camera_top_ = camera_top_info_->getCameraInfo();
        camera_top_.header.frame_id = "CameraTop";
        camera_top_.header.stamp = ts;

        camera_top_image_.header.frame_id = "CameraTop";
        camera_top_image_.header.stamp = ts;

        camera_top_image_.data = v;
        camera_top_image_.width = w;
        camera_top_image_.height = h;
        camera_top_image_.step = w*nbLayers;
        // TO-DO: Need to find encoding from Nao Image
        camera_top_image_.encoding = "rgb8";
        camera_top_image_pub_.publish(camera_top_image_, camera_top_);
    }

    if(camera_bottom_enabled_)
    {
        AL::ALValue im_info = getImage("NaoCameraBottom");
        if(im_info.getSize()!=12)
        {
            return;
        }
        const unsigned char* im = static_cast<const unsigned char*>(im_info[6].GetBinary());
        if(im==NULL)
            return;

        int w = (int)im_info[0];
        int h = (int)im_info[1];
        int nbLayers = (int)im_info[2];
        vector<unsigned char> v = vector<unsigned char> (im, (im+w*h*nbLayers));

        camera_bottom_.header.frame_id = "CameraBottom";
        camera_bottom_.header.stamp = ts;

        camera_bottom_ = camera_bottom_info_->getCameraInfo();
        camera_bottom_.header.frame_id = "CameraBottom";
        camera_bottom_.header.stamp = ts;

        camera_bottom_image_.header.frame_id = "CameraBottom";
        camera_bottom_image_.header.stamp = ts;

        camera_bottom_image_.data = v;
        camera_bottom_image_.width = w;
        camera_bottom_image_.height = h;
        camera_bottom_image_.step = w*nbLayers;
        // TO-DO: Need to find encoding from Nao Image
        camera_bottom_image_.encoding = "rgb8";
        camera_bottom_image_pub_.publish(camera_bottom_image_, camera_bottom_);
    }
}

bool NaoCamera::switchCameraTopState(nao_dcm_msgs::BoolService::Request &req, nao_dcm_msgs::BoolService::Response &res)
{
    if(!camera_enabled_)
        return true;
    camera_top_enabled_ = req.enable;
    return true;
}

bool NaoCamera::switchCameraBottomState(nao_dcm_msgs::BoolService::Request &req,
                                        nao_dcm_msgs::BoolService::Response &res)
{
    if(!camera_enabled_)
        return true;
    camera_bottom_enabled_ = req.enable;
    return true;
}

void NaoCamera::dynamicReconfigureCb(nao_dcm_camera::NaoDCMCameraConfig &config, uint32_t level)
{
    if(!is_connected_)
        return;
    reconfiguring_ = true;
    // Change configurations if needed
    if(camera_top_enabled_)
    {
        if(cameras_config_.top_resolution != config.top_resolution)
        {
            video_proxy_.setResolution("NaoCameraTop", config.top_resolution);
            camera_top_info_->loadCameraInfo(getURLFromConfiguration("top", config.top_resolution));
        }
        if(cameras_config_.top_frame_rate != config.top_frame_rate)
        {
            video_proxy_.setFrameRate("NaoCameraTop", config.top_frame_rate);
        }
    }

    if(camera_bottom_enabled_)
    {
        if(cameras_config_.bottom_resolution != config.bottom_resolution)
        {
            video_proxy_.setResolution("NaoCameraBottom", config.bottom_resolution);
            camera_bottom_info_->loadCameraInfo(getURLFromConfiguration("bottom",config.bottom_resolution));
        }
        if(cameras_config_.bottom_frame_rate != config.bottom_frame_rate)
        {
            video_proxy_.setFrameRate("NaoCameraBottom", config.bottom_frame_rate);
        }
    }

    // Store new configurations
    cameras_config_ = config;

    reconfiguring_ = false;
}

int main( int argc, char** argv )
{
    int pport = 9559;
    string pip = "127.0.0.1";
    ros::init(argc, argv, "nao_dcm_camera");
    ros::NodeHandle n;
    ros::NodeHandle n_p("~");
    ros::NodeHandle top("CameraTopNode"), bottom("CameraBottomNode");
    if(!ros::master::check())
    {
        cerr<<"Could not contact master!\nQuitting... "<<endl;
        return -1;
    }
    // A broker needs a name, an IP and a port:
    string broker_name = "Nao Camera Broker";
    // FIXME: would be a good idea to look for a free port first
    int broker_port = 54001;
    // listen port of the broker (here an anything)
    string broker_ip = "0.0.0.0";
    double communication_rate;

    // Load Params from Parameter Server
    n_p.param("RobotIP", pip, string("127.0.0.1"));
    n_p.param("RobotPort", pport,9559);
    n_p.param("CameraBrokerPort", broker_port, 54001);
    n_p.param("CameraBrokerIP", broker_ip, string("0.0.0.0"));
    n_p.param("CameraFrequency", communication_rate, 30.0);

    // Create your own broker
    boost::shared_ptr<AL::ALBroker> broker;
    try
    {
        broker = AL::ALBroker::createBroker(broker_name,broker_ip,broker_port,pip,pport,0);
    }
    catch(...)
    {
        ROS_ERROR("Failed to connect to Broker at %s:%d!",pip.c_str(),pport);
        AL::ALBrokerManager::getInstance()->removeBroker(broker);
        return -1;
    }

    // Deal with ALBrokerManager singleton (add your broker into NAOqi)
    AL::ALBrokerManager::setInstance(broker->fBrokerManager.lock());
    AL::ALBrokerManager::getInstance()->addBroker(broker);

    // Now it's time to load your module
    boost::shared_ptr<NaoCamera> nao_cam = AL::ALModule::createModule<NaoCamera>(broker, "NaoCamera");
    nao_cam->connect(n,top,bottom);
    if(!nao_cam->connected())
    {
        ROS_ERROR("Could not connect to Nao robot!");
        AL::ALBrokerManager::getInstance()->killAllBroker();
        AL::ALBrokerManager::kill();
        return -1;
    }
    ros::Rate rate(communication_rate);
    // Run the spinner in a separate thread to prevent lockups
    ros::AsyncSpinner spinner(1);
    spinner.start();
    if(broker->isModulePresent("NaoCamera"))
        ROS_INFO("NaoCamera Module loaded succesfully!");
    else
    {
        ROS_ERROR("NaoCamera Module is not loaded!");
        return -1;
    }
    while(ros::ok())
    {
        nao_cam->loop(ros::Time::now(), rate.expectedCycleTime());
        if(!nao_cam->connected())
        {
            ROS_WARN("NaoCamera unexpectedly disconnected!");
            break;
        }
        rate.sleep();
    }
    AL::ALBrokerManager::getInstance()->killAllBroker();
    AL::ALBrokerManager::kill();
    spinner.stop();
    ROS_INFO( "Quitting... " );
    return 0;
}
