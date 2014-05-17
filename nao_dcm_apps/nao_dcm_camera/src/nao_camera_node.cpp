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
