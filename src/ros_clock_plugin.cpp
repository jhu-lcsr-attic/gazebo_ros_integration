/* Copyright (c) 2013, Jonathan Bohren, The Johns Hopkins University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed by the Johns Hopkins University.
 * 4. Neither the name of the Johns Hopkins University nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

namespace gazebo
{   
  class ROSClockPlugin : public WorldPlugin
  {

  public:
    ROSClockPlugin() :
      nh_(NULL),
      node_argc_(0),
      node_argv_(NULL),
      node_name_("gazebo")
    {
    }

    ~ROSClockPlugin()
    {
      ros::shutdown();
    }

    void Load(physics::WorldPtr world, sdf::ElementPtr sdf)
    {
      // Store the pointer to the world
      this->world_ = world;
      
      // Initialize ROS interface, if necessary
      if(!ros::isInitialized()) {
        // Note: If we don't disable the SIGINT handler, gazebo will not exit
        ros::init(node_argc_, NULL, node_name_, ros::init_options::NoSigintHandler);
      }

      // ROS Nodehandle
      this->nh_.reset(new ros::NodeHandle());

      // ROS Subscriber
      this->pub_ = this->nh_->advertise<rosgraph_msgs::Clock>("clock", 1);

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateStart(boost::bind(&ROSClockPlugin::OnUpdate, this));
    }

    // Called by the world update start event
    void OnUpdate()
    {
      // Convert gazebo time into rostime
      rosgraph_msgs::Clock clock_msg;
      common::Time sim_time = world_->GetSimTime();
      clock_msg.clock.nsec = sim_time.nsec;
      clock_msg.clock.sec = sim_time.sec;

      // Publish the time message
      this->pub_.publish(clock_msg);
      ros::spinOnce();
    }

  private:
    // ROS Nodehandle
    boost::scoped_ptr<ros::NodeHandle> nh_;

    // ROS Init Args
    int node_argc_;
    char **node_argv_;
    std::string node_name_;

    // ROS Clock Publisher
    ros::Publisher pub_;

    // Pointer to the world
    physics::WorldPtr world_;
    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(ROSClockPlugin)
}
