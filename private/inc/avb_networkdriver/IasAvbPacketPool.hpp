/*
 @COPYRIGHT_TAG@
 */
/**
 * @file    IasAvbPacketPool.hpp
 * @brief   The definition of the IasAvbPacketPool class.
 * @details This is the pool for AVB packets
 * @date    2013
 */

#ifndef IASAVBPACKETPOOL_HPP_
#define IASAVBPACKETPOOL_HPP_

#include "IasAvbPacket.hpp"
#include "avb_streamhandler/IasAvbTypes.hpp"
#include "avb_streamhandler/IasAvbStreamHandlerEnvironment.hpp"
#include <vector>
#include <linux/if_ether.h>
#include <mutex>

extern "C"
{
  #include "igb.h"
}


namespace IasMediaTransportAvb
{

class IasAvbPacketPool
{
  public:

    /**
     *  @brief Constructor.
     */
    explicit IasAvbPacketPool(DltContext &dltContext);

    /**
     *  @brief Destructor, virtual by default.
     */
    virtual ~IasAvbPacketPool();

    // Operations

    IasAvbProcessingResult init(size_t packetSize, uint32_t poolSize, IasAvbStreamDirection direction = IasAvbStreamDirection::eIasAvbTransmitToNetwork);
    void cleanup();
    IasAvbPacket* getPacket();
    inline IasAvbPacket* getDummyPacket();
    IasAvbProcessingResult initAllPacketsFromTemplate(const IasAvbPacket* templatePacket);
    inline static IasAvbProcessingResult returnPacket(igb_packet* packet);
    static IasAvbProcessingResult returnPacket(IasAvbPacket* packet);
    inline size_t getPacketSize() const;
    inline uint32_t getPoolSize() const;
    inline IasAvbPacket* getPoolBase() const;
    IasAvbProcessingResult reset();

  protected:
    virtual IasAvbProcessingResult derivedInit();
    virtual void derivedCleanup();

    inline IasAvbStreamDirection getDirection() const;

    DltContext *mLog;

  private:
    /**
     * @brief Copy constructor, private unimplemented to prevent misuse.
     */
    IasAvbPacketPool(IasAvbPacketPool const &other);

    /**
     * @brief Assignment operator, private unimplemented to prevent misuse.
     */
    IasAvbPacketPool& operator=(IasAvbPacketPool const &other);

  private:
    // Local Types
    typedef std::vector<IasAvbPacket*> PacketStack;

    // Constants

    static const uint32_t cMaxPoolSize = 2048u;                // derived from max TX ring size / 2
    static const size_t cMaxBufferSize = 2048u;              // fixed value by libigb

    // helpers
    IasAvbProcessingResult doReturnPacket(IasAvbPacket* packet);
    void doCleanup();

    // Members
    std::mutex mLock;
    size_t mPacketSize;
    uint32_t mPoolSize;
    PacketStack mFreeBufferStack;
    IasAvbPacket* mBase;
    IasAvbStreamDirection mDirection;
};


inline IasAvbProcessingResult IasAvbPacketPool::returnPacket(igb_packet* packet)
{
  // simply downcast - probing will be done in the called function
  return returnPacket(static_cast<IasAvbPacket*>(packet));
}

inline size_t IasAvbPacketPool::getPacketSize() const
{
  return mPacketSize;
}

inline uint32_t IasAvbPacketPool::getPoolSize() const
{
  return mPoolSize;
}

inline IasAvbPacket* IasAvbPacketPool::getDummyPacket()
{
  IasAvbPacket* ret = getPacket();

  if (NULL != ret)
  {
    ret->mDummyFlag = true;
  }

  return ret;
}

inline IasAvbPacket* IasAvbPacketPool::getPoolBase() const
{
  return mBase;
}

inline IasAvbStreamDirection IasAvbPacketPool::getDirection() const
{
  return mDirection;
}


} // namespace IasMediaTransportAvb

#endif /* IASAVBCLOCKDOMAIN_HPP_ */
