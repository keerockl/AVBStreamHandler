/*
  @COPYRIGHT_TAG@
*/
/**
 *  @file    IasAvbNetworkDriver.cpp
 *  @brief   Library for the Network Drivers.
 *  @date    2018
 */

#include "avb_networkdriver/IasAvbPacketPool.hpp"
#include "avb_networkdriver/IasAvbNetworkDriver.hpp"
#include "avb_streamhandler/IasAvbStreamHandlerEnvironment.hpp"

namespace IasMediaTransportAvb {

IasAvbNetworkDriver::IasAvbNetworkDriver(DltContext &dltContext)
  : mLog(&dltContext)
  , mInterfaceName()
{
}

IasAvbNetworkDriver::~IasAvbNetworkDriver()
{
  cleanup();
}

void IasAvbNetworkDriver::cleanup()
{
  (void) derivedCleanup();
}

void IasAvbNetworkDriver::derivedCleanup()
{
}

IasAvbProcessingResult IasAvbNetworkDriver::init()
{
  IasAvbProcessingResult ret = eIasAvbProcErr;

  const std::string* ifname = IasAvbStreamHandlerEnvironment::getNetworkInterfaceName();
  if (ifname)
  {
    mInterfaceName = *ifname;
    ret = derivedInit();
  }

  return ret;
}


}
