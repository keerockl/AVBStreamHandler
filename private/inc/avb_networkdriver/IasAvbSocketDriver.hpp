/*
@COPYRIGHT_TAG@
*/
/**
 * @file    IasAvbSocketDriver.hpp
 * @brief   This class contains all base methods to handle a network driver based on cmesg socket
 * @details
 * @date    2018
 */

#ifndef IASAVBSOCKETDRIVER_HPP_
#define IASAVBSOCKETDRIVER_HPP_

#include "avb_networkdriver/IasAvbNetworkDriver.hpp"

#include <linux/if_packet.h>

namespace IasMediaTransportAvb {

class IasAvbSocketDriver : public IasAvbNetworkDriver
{
  public:
    /*!
     * @brief constructor.
     */
    IasAvbSocketDriver(DltContext &dltContext);

    /**
     *  @brief destructor, virtual by default.
     */
    virtual ~IasAvbSocketDriver();

    virtual IasAvbProcessingResult createPacketPool(DltContext &dltContext, IasAvbStreamDirection direction,
                                                    const size_t packetSize, const uint32_t poolSize, IasAvbPacketPool **pool);
    virtual void destroyPacketPool(IasAvbPacketPool *pool);
    virtual int32_t xmit(IasAvbPacket *packet, uint32_t queue_index);
    virtual IasAvbProcessingResult receive(IasAvbPacket **packet, uint32_t queue_index);
    virtual uint32_t reclaimPackets();
    virtual IasAvbProcessingResult reclaimRcvPacket(IasAvbPacket *packet);
    virtual IasAvbProcessingResult setShaperMode(bool enable);
    virtual void updateShaper(double reqBandWidth, uint32_t queue_index, uint32_t maxFrameSizeHigh);
    virtual IasAvbProcessingResult getPtpToClockX(uint64_t &ptpTime, uint64_t &sysTime, const clockid_t clockId,
                                                  uint64_t maxRetry, uint64_t maxMeasurementInterval);
    virtual void emergencyShutdown();

  protected:
    // methods called from base class on init/destruction.
    virtual IasAvbProcessingResult derivedInit();
    virtual void derivedCleanup();

  private:
    typedef std::vector<IasAvbPacket*> PacketList;

    /**
     *  @brief open a socket to send packets with kernel time based packet transmission feature
     */
    IasAvbProcessingResult openTransmitSocket();

    /**
     *  @brief helper to close the transmit socket
     */
    inline void closeTransmitSocket();

    /**
     *  @brief open a socket to receive packets
     */
    IasAvbProcessingResult openReceiveSocket();

    /**
     *  @brief helper to close the receive socket
     */
    inline void closeReceiveSocket();

    IasAvbPacketPool    *mRcvPacketPool;
    PacketList           mXmitPacketList;
    int32_t              mTransmitSocket;
    int32_t              mReceiveSocket;
    struct sockaddr_ll   mDescSockAddr;
    uint32_t             mReceivePacketSize;
};

inline void IasAvbSocketDriver::closeTransmitSocket()
{
  if (mTransmitSocket >= 0)
  {
    (void) ::close(mTransmitSocket);
    mTransmitSocket = -1;
  }
}

inline void IasAvbSocketDriver::closeReceiveSocket()
{
  if (mReceiveSocket >= 0)
  {
    (void) ::close(mReceiveSocket);
    mReceiveSocket = -1;
  }
}


} // namespace IasMediaTransportAvb

#endif /* IASAVBSOCKETDRIVER_HPP_ */
