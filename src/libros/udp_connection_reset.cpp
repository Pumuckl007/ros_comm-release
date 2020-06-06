#include "ros/udp_connection_reset.h"
#include "ros/publisher.h"
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>

namespace ros
{
UDPConnectionReset::UDPConnectionReset() : valid(false){

}
UDPConnectionReset::UDPConnectionReset(short port) : valid(true){;
  struct sockaddr_in name;

  ROS_WARN("Starting UDPConnectionReset\n");

  this->socketId = socket(AF_INET, SOCK_DGRAM, 0);
  if (this->socketId < 0)   {
    ROS_WARN("Could not open reset socket!\n");
    this->valid = false;
    return;
  }

  memset(&name, 0, sizeof(name));
  name.sin_family = AF_INET;
  name.sin_addr.s_addr = htonl(INADDR_ANY);
  name.sin_port = htons(port);

  if (bind(this->socketId, (struct sockaddr *) &name, sizeof(name))) {
    ROS_WARN("Could not bind reset socket! Reason: %d\n", errno);
    this->valid = false;
    return;
  }

  ROS_WARN("Starting server function\n");
  ROS_WARN("SocketId = %d\n", this->socketId);
  boost::thread(boost::bind(&UDPConnectionReset::serverThreadFunc, this));
}

void UDPConnectionReset::serverThreadFunc()
{
  if(!uDPConnectionResetInstance.valid){
    ROS_WARN("Bad server function, portid = %d\n", this->socketId);
  }
  ROS_WARN("Running server function\n");
  while(true){
    int bytes_read;
    char message[1024];
    while ((bytes_read = read(uDPConnectionResetInstance.socketId, message, 1023)) > 0) {
      message[bytes_read] = '\0';
      std::set<Publisher*> set_copy;
      ROS_WARN("Resetting, got message: %s\n", message);
      {
        boost::mutex::scoped_lock lock(this->reset_set_mutex);
        set_copy = this->reset_set;
        this->reset_set.clear();
      }
      ROS_WARN("Resetting %d publishers.", (int)set_copy.size());
      for(Publisher *publisher : set_copy){
        ROS_WARN("Trying Reset udp_connection_reset.cpp:%d", __LINE__);
        try{
          publisher->reset();
        } catch(const std::bad_alloc& bad){
          ROS_WARN("Got a bad error %s\n", bad.what());
        }
      }
    }
  }
}

UDPConnectionReset::~UDPConnectionReset(){
  this->valid = false;
}

void UDPConnectionReset::addPublisher(Publisher &publisher){
  boost::mutex::scoped_lock lock(this->reset_set_mutex);
  this->reset_set.insert(&publisher);
}

void UDPConnectionReset::removePublisher(Publisher &publisher){
  boost::mutex::scoped_lock lock(this->reset_set_mutex);
  this->reset_set.erase(&publisher);
}

void initUDPConnectionReset(short port){
  new (&uDPConnectionResetInstance) UDPConnectionReset(port);
}

UDPConnectionReset uDPConnectionResetInstance;
}
