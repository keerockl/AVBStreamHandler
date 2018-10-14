/*
 @COPYRIGHT_TAG@
 */
/**
 * @file    IasAvbIgbPacketPool.cpp
 * @brief   The implementation of the IasAvbIgbPacketPool class.
 * @date    2018
 */

#include "avb_networkdriver/IasAvbSocketPacketPool.hpp"
#include <dlt_cpp_extension.hpp>

namespace IasMediaTransportAvb {

static const std::string cClassName = "IasAvbSocketPacketPool::";
#define LOG_PREFIX cClassName + __func__ + "(" + std::to_string(__LINE__) + "):"

IasAvbSocketPacketPool::IasAvbSocketPacketPool(DltContext &dltContext)
  : IasAvbPacketPool(dltContext)
  , mPages()
{
}

IasAvbSocketPacketPool::~IasAvbSocketPacketPool()
{
  cleanup();
}

IasAvbProcessingResult IasAvbSocketPacketPool::derivedInit()
{
  IasAvbProcessingResult ret = eIasAvbProcOK;

  if (getPoolBase())
  {
    const size_t packetSize = getPacketSize();
    const uint32_t poolSize = getPoolSize();

    for (uint32_t i = 0; i < poolSize; i++)
    {
      IasAvbPacket & packet = getPoolBase()[i];
      packet.vaddr = calloc(packetSize, 1);

      if (nullptr == packet.vaddr)
      {
        ret = eIasAvbProcNotEnoughMemory;
      }
      else
      {
        mPages.push_back(static_cast<Page*>(packet.vaddr));
      }

      packet.len = static_cast<uint32_t>(packetSize);
      packet.setHomePool( this );

      ret = returnPacket(&packet);
      if (eIasAvbProcOK != ret)
      {
        DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, " Unable to prepare a packet");
        break;
      }
    }
  }

  return eIasAvbProcOK;
}

void IasAvbSocketPacketPool::derivedCleanup()
{
  while (!mPages.empty())
  {
    Page* page = mPages.back();
    mPages.pop_back();

    AVB_ASSERT( NULL != page );

    (void) free(static_cast<void*>(page));
  }
}


} // namespace IasMediaTransportAvb
