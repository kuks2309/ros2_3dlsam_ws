#ifndef k_mapmutex_h__
#define k_mapmutex_h__

#include "util/MapLockerInterface.h"

#include <boost/thread/mutex.hpp>

class K_MapMutex : public MapLockerInterface
{
public:
  virtual void lockMap()
  {
    mapModifyMutex_.lock();
  }

  virtual void unlockMap()
  {
    mapModifyMutex_.unlock();
  }

  boost::mutex mapModifyMutex_;
};

#endif
