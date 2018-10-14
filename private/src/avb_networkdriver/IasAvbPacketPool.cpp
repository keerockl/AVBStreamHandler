/*
 @COPYRIGHT_TAG@
 */
/**
 * @file    IasAvbPacketPool.cpp
 * @brief   The implementation of the IasAvbPacketPool class.
 * @date    2013
 */

#include "avb_networkdriver/IasAvbPacketPool.hpp"
#include "avb_streamhandler/IasAvbStreamHandlerEnvironment.hpp"
#include <cstring>
#include <unistd.h>
#include <dlt_cpp_extension.hpp>

namespace IasMediaTransportAvb {

static const std::string cClassName = "IasAvbPacketPool::";
#define LOG_PREFIX cClassName + __func__ + "(" + std::to_string(__LINE__) + "):"

/*
 *  Constructor.
 */
IasAvbPacketPool::IasAvbPacketPool(DltContext &dltContext) :
  mLog(&dltContext),
  mLock(),
  mPacketSize(0u),
  mPoolSize(0u),
  mFreeBufferStack(),
  mBase(NULL),
  mDirection(IasAvbStreamDirection::eIasAvbTransmitToNetwork)
{
  // do nothing
}


/*
 *  Destructor.
 */
IasAvbPacketPool::~IasAvbPacketPool()
{
  doCleanup();
}


IasAvbProcessingResult IasAvbPacketPool::init(const size_t packetSize, const uint32_t poolSize, IasAvbStreamDirection direction)
{
  (void)direction;

  IasAvbProcessingResult ret = eIasAvbProcOK;

  if (NULL != mBase)
  {
    DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, "Already initialized");
    ret = eIasAvbProcInitializationFailed;
  }
  else
  {
    if ((0u == packetSize) || (packetSize > cMaxBufferSize))
    {
      DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX,
              "packetSize = 0 or packetSize > cMaxBufferSize. packetSize =",
              uint64_t(packetSize), "cMaxBufferSize =", uint64_t(cMaxBufferSize));
      ret = eIasAvbProcInvalidParam;
    }
    else if ((0u == poolSize) || (poolSize > cMaxPoolSize))
    {
      DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX,
              "poolSize = 0 or poolSize > cMaxPoolSize. poolSize =",
              uint64_t(poolSize), "cMaxPoolSize =", uint64_t(cMaxPoolSize));
      ret = eIasAvbProcInvalidParam;
    }
    else
    {
      // all params OK
    }

    if (eIasAvbProcOK == ret)
    {
      mBase = new (nothrow) IasAvbPacket[poolSize];
      if (NULL == mBase)
      {
        /*
         * @log Not enough memory: Packet Pool not created.
         */
        DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX,
                  "Not enough memory to allocate IasAvbPacket! Poolsize=", poolSize);
        ret = eIasAvbProcNotEnoughMemory;
      }
      else
      {
        mPacketSize = packetSize;
        mPoolSize = poolSize;
      }
    }

    if (eIasAvbProcOK == ret)
    {
      ret = derivedInit();
    }

    if (eIasAvbProcOK != ret)
    {
      cleanup();
    }
  }

  return ret;
}

void IasAvbPacketPool::cleanup()
{
  if (mFreeBufferStack.size() < mPoolSize)
  {
    DLT_LOG_CXX(*mLog, DLT_LOG_INFO, LOG_PREFIX,
                " waiting for remaining buffers before pool destruction.",
                uint32_t(mFreeBufferStack.size()),
                "/",
                mPoolSize);

    // wait up to 40ms in 4ms intervals
    for (uint32_t i = 0u; i < 10u; i++)
    {
      ::usleep(5000u);
      if (mFreeBufferStack.size() >= mPoolSize)
      {
        break;
      }
    }
  }

  if (mFreeBufferStack.size() < mPoolSize)
  {
    DLT_LOG_CXX(*mLog, DLT_LOG_WARN, LOG_PREFIX,
                " warning: not all buffers returned before pool destruction!",
                uint32_t(mFreeBufferStack.size()), "/", mPoolSize);
  }

  derivedCleanup();

  doCleanup();
}

IasAvbProcessingResult IasAvbPacketPool::derivedInit()
{
  return eIasAvbProcOK;
}

void IasAvbPacketPool::derivedCleanup()
{
}

void IasAvbPacketPool::doCleanup()
{
  if (mBase)
  {
    delete[] mBase;
    mBase = NULL;
  }
}

IasAvbPacket* IasAvbPacketPool::getPacket()
{
  IasAvbPacket* ret = NULL;

  mLock.lock();

  if (NULL != mBase)
  {
    if (!mFreeBufferStack.empty())
    {
      ret = mFreeBufferStack.back();
      mFreeBufferStack.pop_back();

      AVB_ASSERT(NULL != ret);
      ret->flags = 0u;
      ret->dmatime = 0u;
    }
  }

  mLock.unlock();

  return ret;
}


IasAvbProcessingResult IasAvbPacketPool::doReturnPacket(IasAvbPacket* const packet)
{
  IasAvbProcessingResult ret = eIasAvbProcOK;

  mLock.lock();

  if (NULL == mBase)
  {
    ret = eIasAvbProcNotInitialized;
  }
  else
  {
    AVB_ASSERT( NULL != packet );
    AVB_ASSERT( packet->getHomePool() == this );

    packet->mDummyFlag = false;

    if (mFreeBufferStack.size() < mPoolSize)
    {
      mFreeBufferStack.push_back(packet);

      if (mFreeBufferStack.size() == mPoolSize)
      {
        DLT_LOG_CXX(*mLog, DLT_LOG_DEBUG, LOG_PREFIX, " All buffers returned\n");
      }
    }
    else if (mFreeBufferStack.size() == mPoolSize)
    {
      DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, " Too many packets returned\n");
      // put it on the stack to avoid further LAT messages - stack is inconsistent anyway
      mFreeBufferStack.push_back(packet);
    }
    else
    {
      // do nothing to avoid further stack growth
    }
  }

  mLock.unlock();

  return ret;
}


IasAvbProcessingResult IasAvbPacketPool::initAllPacketsFromTemplate(const IasAvbPacket* templatePacket)
{
  IasAvbProcessingResult ret = eIasAvbProcOK;

  if (NULL == mBase)
  {
    ret = eIasAvbProcNotInitialized;
  }
  else if ((NULL == templatePacket) || (NULL == templatePacket->vaddr) || (0u == templatePacket->len))
  {
    ret = eIasAvbProcInvalidParam;
  }
  else
  {
    for (PacketStack::iterator it = mFreeBufferStack.begin(); it != mFreeBufferStack.end(); it++)
    {
      IasAvbPacket * packet = *it;
      AVB_ASSERT( NULL != packet);

      *packet = *templatePacket;
    }
  }

  return ret;
}


IasAvbProcessingResult IasAvbPacketPool::reset()
{
  IasAvbProcessingResult ret = eIasAvbProcOK;

  mLock.lock();

  if (NULL == mBase)
  {
    ret = eIasAvbProcNotInitialized;
  }
  else
  {
    DLT_LOG_CXX(*mLog, DLT_LOG_DEBUG, LOG_PREFIX, " clear FreeBufferStack and push back all buffer");
    mFreeBufferStack.clear();
    for (uint32_t packetIdx = 0u; packetIdx < mPoolSize; packetIdx++)
    {
      mFreeBufferStack.push_back( &mBase[packetIdx] );
    }
  }

  mLock.unlock();

  return ret;
}

IasAvbProcessingResult IasAvbPacketPool::returnPacket(IasAvbPacket* packet)
{
  IasAvbProcessingResult ret = eIasAvbProcOK;

  if (NULL == packet)
  {
    DltContext &dltCtx = IasAvbStreamHandlerEnvironment::getDltContext("_AAS");
    DLT_LOG_CXX(dltCtx,  DLT_LOG_ERROR, LOG_PREFIX, "packet is NULL!");
    ret = eIasAvbProcInvalidParam;
  }
  else
  {
    // probe if really an IasAvbPacket
    if  (!packet->isValid())
    {
      DltContext &dltCtx = IasAvbStreamHandlerEnvironment::getDltContext("_AAS");
      DLT_LOG_CXX(dltCtx,  DLT_LOG_ERROR, LOG_PREFIX, "invalid packet!");
      ret = eIasAvbProcInvalidParam;
    }
    else
    {
      IasAvbPacketPool * const home = packet->getHomePool();
      AVB_ASSERT(NULL != home);
      ret = home->doReturnPacket(packet);
    }
  }

  return ret;
}


} // namespace IasMediaTransportAvb
