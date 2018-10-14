/*
@COPYRIGHT_TAG@
*/
/**
 * @file    IasAvbNetworkDriver.hpp
 * @brief   This class contains all base methods to handle a network driver
 * @details
 * @date    2018
 */

#ifndef IASAVBNETWORKDRIVER_HPP_
#define IASAVBNETWORKDRIVER_HPP_

#include "avb_streamhandler/IasAvbTypes.hpp"
#include <dlt.h>

namespace IasMediaTransportAvb {

class IasAvbPacket;
class IasAvbPacketPool;

class IasAvbNetworkDriver
{
  public:
    /**
     * @brief clock ID to be used whenever dealing with the local system time
     *
     * Note that, unfortunately,  CLOCK_MONOTONIC_RAW cannot be used
     * as it is not supported by some functions (e.g. clock_nanosleep)
     */
    static const clockid_t cSysClockId = CLOCK_MONOTONIC;
    static const clockid_t cRawClockId = CLOCK_MONOTONIC_RAW;

    enum RxQueueId
    {
      eRxQueue0 = 0u,
      eRxQueue1
    };

    /*!
     * @brief default constructor
     */
    IasAvbNetworkDriver(DltContext &dltContext);

    /**
     *  @brief destructor, virtual by default
     */
    virtual ~IasAvbNetworkDriver();

    /**
     *  @brief initialize method.
     *
     */
    IasAvbProcessingResult init();

    /**
     *  @brief clean up all allocated resources
     */
    void cleanup();

    /**
     *  @brief send a packet
     */
    virtual int32_t xmit(IasAvbPacket *packet, uint32_t queue_index) = 0;

    /**
     *  @brief receive a packet
     */
    virtual IasAvbProcessingResult receive(IasAvbPacket **packet, uint32_t queue_index) = 0;

    /**
     *  @brief create a packet pool
     */
    virtual IasAvbProcessingResult createPacketPool(DltContext &dltContext, IasAvbStreamDirection direction,
                                                    const size_t packetSize, const uint32_t poolSize, IasAvbPacketPool **pool) = 0;

    /**
     *  @brief destroy a packet pool
     */
    virtual void destroyPacketPool(IasAvbPacketPool *pool) = 0;

    /**
     *  @brief return used transmit packets to a packet pool
     */
    virtual uint32_t reclaimPackets() = 0;

    /**
     *  @brief return used receive packet to a packet pool
     */
    virtual IasAvbProcessingResult reclaimRcvPacket(IasAvbPacket *packet) = 0;

    /**
     * @brief enable/disable traffic shaper
     */
    virtual IasAvbProcessingResult setShaperMode(bool enable) = 0;

    /**
     * @brief update traffic shaper
     */
    virtual void updateShaper(double reqBandWidth, uint32_t queue_index, uint32_t maxFrameSizeHigh) = 0;

    /**
     *  @brief get current gPTP time and time of clock specified with clockId (cSysClockId or cRawClockId)
     */
    virtual IasAvbProcessingResult getPtpToClockX(uint64_t &ptpTime, uint64_t &sysTime, const clockid_t clockId,
                                                  uint64_t maxRetry, uint64_t maxMeasurementInterval) = 0;

    /**
     *  @brief clean up minimum driver resources
     */
    virtual void emergencyShutdown() = 0;

  protected:
    // methods called from base class on init/destruction. to be implemeted by derived classes
    virtual IasAvbProcessingResult derivedInit() = 0;
    virtual void derivedCleanup();

    // members shared with derived class
    DltContext    *mLog;
    std::string    mInterfaceName;
};


} // namespace IasMediaTransportAvb

#endif /* IASAVBNETWORKDRIVER_HPP_ */
