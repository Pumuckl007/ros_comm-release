#include "common.h"
#include <set>
#include <functional>
#include <boost/thread/mutex.hpp>

#ifndef ROSCPP_UDP_CONNECTION_RESET_H
#define ROSCPP_UDP_CONNECTION_RESET_H


namespace ros
{

//forward declaration
class Publisher;

void initUDPConnectionReset(short port);

class ROSCPP_DECL UDPConnectionReset {
public:
UDPConnectionReset();
UDPConnectionReset(short port);
~UDPConnectionReset();

void addPublisher(Publisher &publisher);
void removePublisher(Publisher &publisher);

private:
  bool valid;
  int socketId;
  void serverThreadFunc();
  std::set<Publisher*> reset_set;
  boost::mutex reset_set_mutex;
};

extern UDPConnectionReset uDPConnectionResetInstance;
}

#endif
