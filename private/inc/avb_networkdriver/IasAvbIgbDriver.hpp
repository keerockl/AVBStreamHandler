/*
@COPYRIGHT_TAG@
*/
/**
 * @file    IasAvbIgbDriver.hpp
 * @brief   This class implements a driver class based on libigb and igb_avb drivers
 * @details
 * @date    2018
 */

#ifndef IASAVBIGBDRIVER_HPP_
#define IASAVBIGBDRIVER_HPP_

#include "avb_networkdriver/IasAvbNetworkDriver.hpp"

extern "C" {
#include "igb.h"
}

namespace IasMediaTransportAvb {

class IasAvbIgbDriver : public IasAvbNetworkDriver
{
  public:
    /*!
     * @brief constructor
     */
    IasAvbIgbDriver(DltContext &dltContext);

    /**
     *  @brief destructor, virtual by default
     */
    virtual ~IasAvbIgbDriver();

    virtual int32_t xmit(IasAvbPacket *packet, uint32_t queue_index);
    virtual IasAvbProcessingResult receive(IasAvbPacket **packet, uint32_t queue_index);
    virtual IasAvbProcessingResult createPacketPool(DltContext &dltContext, IasAvbStreamDirection direction,
                                                    const size_t packetSize, const uint32_t poolSize, IasAvbPacketPool **pool);
    virtual void destroyPacketPool(IasAvbPacketPool *pool);
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

    struct Diag
    {
      Diag();
      uint64_t rawXCount;
      uint64_t rawXFail;
      uint64_t rawXMaxInt;
      uint64_t rawXMinInt;
      uint64_t rawXTotalInt;
      uint64_t rawXunlock;
      uint64_t rawXUnlockCount;
    };

    //
    // constants
    //

    static const uint32_t cIgbAccessSleep = 100000u; // in us: 100 ms
    static const uint32_t cTxMaxInterferenceSize = 1522u; ///< assumed maximum frame size of Non-SR packets

    static const size_t cReceiveFilterDataSize = 128u;  // flexible filter maximum data length
    static const size_t cReceiveFilterMaskSize = 16u;   // flexible filter maximum mask length
    static const size_t cReceivePoolSize       = 256u;  // The number of RX packet buffers
    static const size_t cReceiveBufferSize     = 2048u; // fixed value by libigb

    enum RxFilterId
    {
      eRxFilter0 = 0u,
      eRxFilter1,
      eRxFilter2,
      eRxFilter3,
      eRxFilter4,
      eRxFilter5,
      eRxFilter6,
      eRxFilter7
    };

    /**
     *  @brief attach to igb_avb driver
     */
    IasAvbProcessingResult Attach();

    /**
     *  @brief enable/disable the flex filter
     */
    IasAvbProcessingResult setFlexFilterMode(bool enable);

    /**
     *  @brief set dma capable buffers on I210's receive descriptor ring
     */
    IasAvbProcessingResult prepareRcvPkts(void);

    /**
     *  @brief get current CLOCK_MONOTONIC time in nanoseconds
     */
    inline uint64_t getSystemTime();

    /**
     *  @brief get current tsc time in nanoseconds
     */
    inline uint64_t getTscTime();

    /**
     *  @brief convert timespec time to value in nanoseconds
     */
    inline uint64_t convertTimespecToNs(const struct timespec &tp);

    device_t         *mIgbDevice;
    Diag              mDiag;
    IasAvbPacketPool *mRcvPacketPool;
    PacketList        mPacketList;
    bool              mRecoverIgbReceiver;
};

inline uint64_t IasAvbIgbDriver::getSystemTime()
{
  struct timespec tp;
  (void) clock_gettime(cSysClockId, &tp);
  return convertTimespecToNs(tp);
}

inline uint64_t IasAvbIgbDriver::getTscTime()
{
  struct timespec tp;
  (void) clock_gettime(cRawClockId, &tp);
  return convertTimespecToNs(tp);
}

inline uint64_t IasAvbIgbDriver::convertTimespecToNs(const struct timespec &tp)
{
    return (uint64_t(int64_t(tp.tv_sec)) * uint64_t(1000000000u)) +
            uint64_t(int64_t(tp.tv_nsec));
}


} // namespace IasMediaTransportAvb

#endif /* IASAVBIGBDRIVER_HPP_ */

