/*
  @COPYRIGHT_TAG@
*/
/**
 *  @file    IasAvbNetworkDriver.cpp
 *  @brief   Library for the libigb/igb_avb Network Drivers.
 *  @date    2018
 */

#include "avb_networkdriver/IasAvbIgbDriver.hpp"
#include "avb_networkdriver/IasAvbIgbPacketPool.hpp"
#include "avb_streamhandler/IasAvbStreamHandlerEnvironment.hpp"
#include <dlt_cpp_extension.hpp>
#include <cstring>
#include <sstream>
#include <fstream>
#include <arpa/inet.h>

namespace IasMediaTransportAvb {

static const std::string cClassName = "IasAvbIgbDriver::";
#define LOG_PREFIX cClassName + __func__ + "(" + std::to_string(__LINE__) + "):"

#ifndef ETH_P_IEEE1722
#define ETH_P_IEEE1722 0x22F0
#endif

#define TSAUXC    0x0B640
#define TSAUXC_SAMP_AUTO 0x00000008
#define AUXSTMPH0 0x0B660
#define AUXSTMPL0 0x0B65C

#define TQAVCTRL      0x03570
#define TQAVHC(_n)    (0x0300C + ((_n) * 0x40))
#define TQAVCC(_n)    (0x03004 + ((_n) * 0x40))

#define TQAVCH_ZERO_CREDIT 0x80000000 /* not configured and always defaults to this value */
#define TQAVCC_LINKRATE    0x7735     /* not configured and always defaults to this value */
#define TQAVCC_QUEUEMODE   0x80000000 /* queue mode, 0=strict, 1=SR mode */
#define TQAVCTRL_TX_ARB    0x00000100 /* data transmit arbitration */

#define RCTL        0x00100
#define RCTL_RXEN   (1 << 1)
#define SRRCTL(_n)  ((_n) < 4 ? (0x0280C + ((_n) * 0x100)) : \
         (0x0C00C + ((_n) * 0x40)))
#define RXDCTL(_n)  ((_n) < 4 ? (0x02828 + ((_n) * 0x100)) : \
         (0x0C028 + ((_n) * 0x40)))

IasAvbIgbDriver::IasAvbIgbDriver(DltContext &dltContext)
  : IasAvbNetworkDriver(dltContext)
  , mIgbDevice(nullptr)
  , mDiag()
  , mRcvPacketPool(nullptr)
  , mPacketList()
  , mRecoverIgbReceiver(true)
{
}

IasAvbIgbDriver::~IasAvbIgbDriver()
{
  cleanup();
}

IasAvbIgbDriver::Diag::Diag()
  : rawXCount(0u)
  , rawXFail(0u)
  , rawXMaxInt(0u)
  , rawXMinInt(0u)
  , rawXTotalInt(0u)
{
}

void IasAvbIgbDriver::derivedCleanup()
{
  if (mIgbDevice)
  {
    igb_detach(mIgbDevice);
    delete mIgbDevice;
    mIgbDevice = nullptr;
  }
}

IasAvbProcessingResult IasAvbIgbDriver::derivedInit()
{
  IasAvbProcessingResult ret = eIasAvbProcErr;
  if (mIgbDevice)
  {
    ret = eIasAvbProcOK;  // already initialized
  }
  else
  {
    mIgbDevice = new (nothrow) device_t;

    if (NULL == mIgbDevice)
    {
      DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, "Not enough memory to allocate device_t");
      ret = eIasAvbProcNotEnoughMemory;
    }
    else
    {
      bool result = false;

      mIgbDevice->private_data = NULL;

      std::ostringstream filePath;
      filePath << "/sys/class/net/" << mInterfaceName.c_str() << "/device/uevent";
      std::cout << "Interface name: " << mInterfaceName.c_str() << std::endl;

      std::string fileName = filePath.str();
      std::ifstream configFile(fileName);
      std::string configLine;

      if (configFile.good())
      {
        while (std::getline(configFile, configLine))
        {
          std::stringstream line(configLine);
          std::string value;
          std::vector<std::string> values;

          if (configLine.find("PCI_ID") != std::string::npos)
          {
            while (std::getline(line, value, '='))
            {
              while (std::getline(line, value, ':'))
              {
                values.push_back(value);
              }
            }

            mIgbDevice->pci_device_id = uint16_t(std::stoi(values.back(), nullptr, 16));
            values.pop_back();

            mIgbDevice->pci_vendor_id = uint16_t(std::stoi(values.back(), nullptr, 16));
            values.pop_back();
          }
          else if (configLine.find("PCI_SLOT_NAME") != std::string::npos)
          {
            while (std::getline(line, value, '='))
            {
              while (std::getline(line, value, ':'))
              {
                std::stringstream tmpLine(value);
                while (std::getline(tmpLine, value, '.'))
                {
                  values.push_back(value);
                }
              }
            }

            mIgbDevice->func = uint8_t(std::stoi(values.back(), nullptr, 16));
            values.pop_back();

            mIgbDevice->dev = uint8_t(std::stoi(values.back(), nullptr, 16));
            values.pop_back();

            mIgbDevice->bus = uint8_t(std::stoi(values.back(), nullptr, 16));
            values.pop_back();

            mIgbDevice->domain = uint16_t(std::stoi(values.back(), nullptr, 16));
            values.pop_back();
          }
          result = true;
        }
      }

      if (result)
      {
        ret = Attach();
      }
      else
      {
        DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, "Could not find configuration file for interface");
        ret = eIasAvbProcInitializationFailed;
      }
    }
  }

  if (ret == eIasAvbProcOK)
  {
    int32_t err = -1;
    uint32_t errCount = 0u;
    uint32_t timeoutCnt  = 0u;
    (void) IasAvbStreamHandlerEnvironment::getConfigValue(IasRegKeys::cIgbAccessTimeoutCnt, timeoutCnt);

    // Retry until igb_avb is ready.
    while ((0 != err) && (timeoutCnt > errCount))
    {
      err = setShaperMode(false); /* Qav disabled*/
      if (0u != err)
      {
        ::usleep(cIgbAccessSleep);
        errCount ++;
      }
      else
      {
        if (0u != errCount)
        {
          DLT_LOG_CXX(*mLog, DLT_LOG_WARN, LOG_PREFIX, "Couldn't configure shaper!(",
              strerror(err), " error count:", (errCount), ")");
          // do not abort, try running without config
        }
      }
    }
  }

  (void) IasAvbStreamHandlerEnvironment::getConfigValue(IasRegKeys::cRxRecoverIgbReceiver, mRecoverIgbReceiver);
  DLT_LOG_CXX(*mLog, DLT_LOG_DEBUG, LOG_PREFIX, "Rx IGB Recovery:", mRecoverIgbReceiver ? "on" : "off");

  if (ret != eIasAvbProcOK)
  {
    (void) cleanup();
  }

  return ret;
}

IasAvbProcessingResult IasAvbIgbDriver::Attach()
{
  IasAvbProcessingResult ret = eIasAvbProcOK;

  if (mIgbDevice && mIgbDevice->private_data)
  {
    ret = eIasAvbProcOK;  // already attached
  }
  else
  {
    const uint16_t cPciPathMaxLen = 24u;
    char devPath[cPciPathMaxLen]  = {0};

    (void) std::snprintf(devPath, cPciPathMaxLen, "%04x:%02x:%02x.%d",
                      mIgbDevice->domain, mIgbDevice->bus, mIgbDevice->dev, mIgbDevice->func);

    int32_t err = -1;
    err = igb_attach(devPath, mIgbDevice);
    if (err)
    {
      DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, "igb_attach for",
          mInterfaceName.c_str(), "at",
          devPath, "failed (", int32_t(err),
          ",", strerror(err), ")");
      ret = eIasAvbProcInitializationFailed;
    }
    else
    {
      DLT_LOG_CXX(*mLog,  DLT_LOG_INFO, LOG_PREFIX, "igb_attach for", mInterfaceName.c_str(), "OK");

      err = igb_attach_rx(mIgbDevice);
      if (err)
      {
        DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, "igb_attach_rx failed (",
            int32_t(err), ")");
        ret = eIasAvbProcInitializationFailed;
      }
      else
      {
        err = igb_attach_tx(mIgbDevice);
        if (err)
        {
          DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, "igb_attach_tx failed (",
              int32_t(err), ")");
          ret = eIasAvbProcInitializationFailed;
        }
      }

      if (eIasAvbProcOK == ret)
      {
        err = igb_init(mIgbDevice);
        if (err)
        {
          DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, "igb_init failed (",
              int32_t(err), ")");
          ret = eIasAvbProcInitializationFailed;
        }
        else
        {
          ret = eIasAvbProcOK;
        }
      }

      if (eIasAvbProcOK != ret)
      {
        DLT_LOG_CXX(*mLog,  DLT_LOG_ERROR, LOG_PREFIX, "igb_detach (init failed)");
        (void) cleanup();
      }
    }
  }

  return ret;
}

IasAvbProcessingResult IasAvbIgbDriver::createPacketPool(DltContext &dltContext, IasAvbStreamDirection direction,
                                                         const size_t packetSize, const uint32_t poolSize, IasAvbPacketPool **pool)
{
  IasAvbProcessingResult ret = eIasAvbProcOK;
  IasAvbPacketPool *tmpPool = nullptr;

  if (nullptr == pool)
  {
    ret = eIasAvbProcInvalidParam;
  }

  if (eIasAvbReceiveFromNetwork == direction)
  {
    if (mRcvPacketPool)
    {
      ret = eIasAvbProcInvalidParam;
    }
  }

  if (eIasAvbProcOK == ret)
  {
    *pool = nullptr;

    tmpPool = new (nothrow) IasAvbIgbPacketPool(dltContext, mIgbDevice);
    if (tmpPool)
    {
      ret = tmpPool->init(packetSize, poolSize, direction);
    }
    else
    {
      ret = eIasAvbProcNotEnoughMemory;
    }
  }

  if (eIasAvbProcOK == ret)
  {
    if (eIasAvbReceiveFromNetwork == direction)
    {
      mRcvPacketPool = tmpPool;

      if (eIasAvbProcOK == ret)
      {
        ret = prepareRcvPkts();
      }

      if (eIasAvbProcOK == ret)
      {
        ret = setFlexFilterMode(true);
      }

      if (eIasAvbProcOK == ret)
      {

      }
      else
      {
        setFlexFilterMode(false);
        mRcvPacketPool = nullptr;
      }
    }
  }

  if (eIasAvbProcOK == ret)
  {
    *pool = tmpPool;
  }
  else
  {
    if (tmpPool)
    {
      delete tmpPool;
    }
  }

  return ret;
}

void IasAvbIgbDriver::destroyPacketPool(IasAvbPacketPool *pool)
{
  if (pool)
  {
    if (mRcvPacketPool == pool)
    {
      setFlexFilterMode(false);

      IasAvbPacket* packet = NULL;

      while (!mPacketList.empty())
      {
        packet = mPacketList.back();
        if (NULL != packet)
        {
          (void) IasAvbPacketPool::returnPacket(packet);
        }
        (void) mPacketList.pop_back();
      }

      mRcvPacketPool = nullptr;
    }
    delete pool;
  }
}


int32_t IasAvbIgbDriver::xmit(IasAvbPacket *packet, uint32_t queue_index)
{
  return igb_xmit(mIgbDevice, queue_index, reinterpret_cast<struct igb_packet *>(packet));
}

uint32_t IasAvbIgbDriver::reclaimPackets()
{
  uint32_t ret = 0u;
  igb_packet* packetList = NULL;

  // check and return packets that are not used any longer
  // NOTE: this is done for all sequencers, not only for this one!
  igb_clean(mIgbDevice, &packetList);
  while (NULL != packetList)
  {
    IasAvbPacketPool::returnPacket(packetList);
    ret++;
    packetList = packetList->next;
  }

  return ret;
}

void IasAvbIgbDriver::updateShaper(double reqBandWidth, uint32_t queue_index, uint32_t maxFrameSizeHigh)
{
  double bandWidth = 0.0;
  int32_t  linkSpeed = 0;
  int32_t  idleSlope = -1;
  uint32_t linkRate  = TQAVCC_LINKRATE;
  uint32_t maxInterferenceSize = cTxMaxInterferenceSize;

  uint32_t tqavhcReg   = 0u; // Tx Qav Hi Credit TQAVHC
  uint32_t tqavccReg   = 0u; // Tx Qav Credit Control TQAVCC
  uint32_t tqavctrlReg = 0u; // Tx Qav Control TQAVCTRL

  // get current link speed
  linkSpeed = IasAvbStreamHandlerEnvironment::getLinkSpeed();

  if (100 == linkSpeed)
  {
    // the percentage bandwith out of full line rate @ 100Mbps (currBandWidth = kBit/s)
    bandWidth = reqBandWidth * 1000.0 / 100000000.0;
    idleSlope = static_cast<uint32_t>(bandWidth * 0.2 * (double)linkRate + 0.5);
  }
  else if (1000 == linkSpeed)
  {
    // the percentage bandwith out of full line rate @ 1Gbps (currBandWidth = kBit/s)
    bandWidth = reqBandWidth * 1000.0 / 1000000000.0;
    idleSlope = static_cast<uint32_t>(bandWidth * 2.0 * (double)linkRate + 0.5);
  }
  else
  {
    DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, "unknown link speed", linkSpeed);
    AVB_ASSERT(false);
  }

  if (0 == idleSlope)
  {
    // reset the registers with default value
    tqavhcReg = TQAVCH_ZERO_CREDIT;
    tqavccReg = TQAVCC_QUEUEMODE; // no idle slope
  }
  else if ((0 < idleSlope) && (static_cast<uint32_t>(idleSlope) < linkRate))
  {
    // idleSlope must be smaller than linkRate = 0x7735 credits/byte.

    if (0u == queue_index)
    {
      tqavhcReg = TQAVCH_ZERO_CREDIT + (idleSlope * maxInterferenceSize / linkRate);
    }
    else
    {
      uint32_t idleSlopeClassA = 0u;

      (void) igb_readreg(mIgbDevice, TQAVHC(0), &idleSlopeClassA);
      idleSlopeClassA ^= TQAVCC_QUEUEMODE;

      if ((0 == idleSlopeClassA) || (0xFFFFu == static_cast<uint16_t>(idleSlopeClassA)))
      {
        // idleSlope has not been set. i.e. Class A (Queue0) is not used.
        tqavhcReg = TQAVCH_ZERO_CREDIT + (idleSlope * maxInterferenceSize / linkRate);
      }
      else if (static_cast<int32_t>(linkRate - idleSlopeClassA) > 0)
      {
        /*
         * Add 43 bytes to mMaxFrameSizeHigh since it is the size of AVTP payload without media overhead nor header.
         * (43 = 8 bytes preamble + SFD, 14 bytes Ethernet header, 4 bytes VLAN tag, 4 bytes CRC, 12 bytes IPG, 1 byte SRP)
         */
        maxInterferenceSize = maxInterferenceSize + (maxFrameSizeHigh + 43u);
        linkRate = linkRate - idleSlopeClassA;
        tqavhcReg = TQAVCH_ZERO_CREDIT + (idleSlope * maxInterferenceSize / linkRate);
      }
    }

    tqavccReg = (TQAVCC_QUEUEMODE | idleSlope);
  }

  if ((0u != tqavhcReg) && (0u != tqavccReg))
  {
    // HiCredit
    (void) igb_writereg(mIgbDevice, TQAVHC(queue_index), tqavhcReg);

    // QueueMode and IdleSlope
    (void) igb_writereg(mIgbDevice, TQAVCC(queue_index), tqavccReg);

    // implicitly enable the Qav shaper
    (void) igb_readreg(mIgbDevice, TQAVCTRL, &tqavctrlReg);
    tqavctrlReg |= TQAVCTRL_TX_ARB;
    (void) igb_writereg(mIgbDevice, TQAVCTRL, tqavctrlReg);

    DLT_LOG_CXX(*mLog, DLT_LOG_INFO, LOG_PREFIX, "set shaping params (",
        "Queue:", queue_index,
        "Bandwidth:", reqBandWidth, "kBit/s",
        "HiCredit:", tqavhcReg,
        "IdleSlope:", idleSlope,
        "");
  }
}

IasAvbProcessingResult IasAvbIgbDriver::setShaperMode(bool enable)
{
  IasAvbProcessingResult ret = eIasAvbProcOK;
  if (enable)
  {

  }
  else
  {
    int32_t err = igb_set_class_bandwidth( mIgbDevice, 0u, 0u, 1500u, 64u ); /* Qav disabled*/
    if (err)
    {
      ret = eIasAvbProcErr;
    }
  }

  return ret;
}

IasAvbProcessingResult IasAvbIgbDriver::getPtpToClockX(uint64_t &ptpTime, uint64_t &sysTime, const clockid_t clockId,
                                                       uint64_t maxRetry, uint64_t maxMeasurementInterval)
{
  IasAvbProcessingResult result = eIasAvbProcOK;

  uint64_t sysTimeMeasurementInterval = 0u;
  uint64_t sysTimeMeasurementIntervalMin = std::numeric_limits<uint64_t>::max();

  ptpTime = 0u;
  sysTime = 0u;

  for (uint64_t i = 0u; i < maxRetry; i++)
  {
    /*
     * The value of cMaxCrossTimestampSamples is 3 by default.
     *
     * This method gets the ptp/monotonic cross-timestamp maximum 3 times then returns the most accurate one.
     * More iteration could provide higher accuracy. But since it has to lock the IGB device to access the registers
     * and the lock might block the TX sequencer which calls igb_xmit(), the number of iteration should be limited.
     *
     * It took one third cpu time to read the I210 clock from the registers compared to the general approach
     * using the clock_gettime(). Then we may do the iteration at least 3 times to get better accuracy without
     * increasing cpu load. (approx time needed on MRB: direct register access 3 us, clock_gettime 10 us)
     */
    if ((NULL == mIgbDevice) || (0 != igb_lock(mIgbDevice)) ||
        ((cSysClockId != clockId) && (cRawClockId != clockId)) )
    {
      result = eIasAvbProcErr;
      break;
    }
    else
    {
      uint64_t sys1 = 0u;
      uint64_t sys2 = 0u;

      uint32_t tsauxcReg = 0u;
      uint32_t stmph0Reg = 0u;
      uint32_t stmpl0Reg = 0u;

      (void) igb_readreg(mIgbDevice, TSAUXC, &tsauxcReg);
      tsauxcReg |= TSAUXC_SAMP_AUTO;

      // clear the value stored in AUXSTMPH/L0
      (void) igb_readreg(mIgbDevice, AUXSTMPH0, &stmph0Reg);

      sys1 = (cSysClockId == clockId) ? getSystemTime() : getTscTime();

      // set the SAMP_AUT0 flag to latch the SYSTIML/H registers
      (void) igb_writereg(mIgbDevice, TSAUXC, tsauxcReg);

      // memory fence to avoid reading the registers before writing the SAMP_AUT0 flag
      __asm__ __volatile__("mfence;"
                  :
                  :
                  : "memory");

      sys2 = (cSysClockId == clockId) ? getSystemTime() : getTscTime();

      // read the stored values
      (void) igb_readreg(mIgbDevice, AUXSTMPH0, &stmph0Reg);
      (void) igb_readreg(mIgbDevice, AUXSTMPL0, &stmpl0Reg);

      (void) igb_unlock(mIgbDevice);

      sysTimeMeasurementInterval = sys2 - sys1;
      if (sysTimeMeasurementInterval < sysTimeMeasurementIntervalMin)
      {
        sysTime = (sys1 >> 1) + (sys2 >> 1);
        ptpTime = stmph0Reg * uint64_t(1000000000u) + stmpl0Reg;
        sysTimeMeasurementIntervalMin = sysTimeMeasurementInterval;

        if (sysTimeMeasurementIntervalMin <= maxMeasurementInterval)
        {
          // immediately exit the loop once we get cross-timestamp with the target accuracy
          break;
        }
      }
    }
  }

  if (cRawClockId == clockId)
  {
    mDiag.rawXCount++;
    if (maxMeasurementInterval < sysTimeMeasurementIntervalMin)
    {
      mDiag.rawXFail++;
      result = eIasAvbProcErr;
    }

    // statistics for analysis
    if (mDiag.rawXMaxInt < sysTimeMeasurementInterval)
    {
      mDiag.rawXMaxInt = sysTimeMeasurementInterval;
    }
    if ((0u == mDiag.rawXMinInt) || (sysTimeMeasurementInterval < mDiag.rawXMinInt))
    {
      mDiag.rawXMinInt = sysTimeMeasurementInterval;
    }
    mDiag.rawXTotalInt += sysTimeMeasurementInterval;

    const double rawXSuccessRate = double(mDiag.rawXCount - mDiag.rawXFail)/double(mDiag.rawXCount);
    const double rawXAvgInterval = double(mDiag.rawXTotalInt) / double(mDiag.rawXCount);
    DLT_LOG_CXX(*mLog, DLT_LOG_DEBUG, LOG_PREFIX, "raw-x-tstamp diag: success rate avg =",
                rawXSuccessRate, "interval avg =", rawXAvgInterval, "max =", mDiag.rawXMaxInt, "min =", mDiag.rawXMinInt);
  }

  return result;
}

IasAvbProcessingResult IasAvbIgbDriver::setFlexFilterMode(bool enable)
{
  IasAvbProcessingResult result = eIasAvbProcOK;

  if (enable)
  {
    uint8_t flexFilterData[cReceiveFilterDataSize];
    uint8_t flexFilterMask[cReceiveFilterMaskSize];

    struct ethhdr * ethHdr = reinterpret_cast<struct ethhdr*>(flexFilterData);
    uint16_t * vlanEthType = reinterpret_cast<uint16_t*>(flexFilterData + ETH_HLEN + 2u);

    // setup the filter
    std::memset((void*)flexFilterData, 0u, cReceiveFilterDataSize);
    std::memset((void*)flexFilterMask, 0u, cReceiveFilterMaskSize);

    // accept the IEEE1722 packets with a VLAN tag
    ethHdr->h_proto = htons(ETH_P_8021Q);
    *vlanEthType = htons(ETH_P_IEEE1722);

    flexFilterMask[1] = 0x30; /* 00110000b = ethtype */
    flexFilterMask[2] = 0x03; /* 00000011b = ethtype after a vlan tag */

    // length must be 8 byte-aligned
    uint32_t len = (uint32_t)sizeof(struct ethhdr) + 4u;
    len = ((len + (8u - 1u)) / 8u) * 8u;

    if (0 != igb_lock(mIgbDevice))
    {
      DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, "IGB lock failure");
      result = eIasAvbProcErr;
    }
    else
    {
      // enable the filter on queue 0 with filter_id 0
      if (igb_setup_flex_filter(mIgbDevice, eRxQueue0, eRxFilter0, len, flexFilterData, flexFilterMask) != 0)
      {
        DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, "error enabling the flex filter",
                    int32_t(errno), " (", strerror(errno), ")");
        result = eIasAvbProcErr;
      }
      else
      {
        /*
         * SRRCTL.Drop_En : Drop Enabled.
         *
         * If set, packets received to the queue when no descriptors are available to store them are dropped.
         * Default is 0b for queue 0. While AVBSH is at the stop state upon receiving the SIGSTOP signal, RX
         * engine can't fetch packets from queue 0. Although we separate queue 0 from another queue for the
         * best-effort packets, all queues share single packet buffer at the entry point of I210. Therefore if
         * we do not permit dropping AVTP packets assigned to queue 0, the packet buffer will eventually be
         * occupied by AVTP packets which interferes with the reception of the best-effort packets at the stop
         * state.
         *
         * Enabling this bit will secure bandwidth for the best-effort packets at the stop state but on the
         * other hand it may increase the threat of packet dropping of AVTP packets during normal operation.
         * Since SIGSTOP is a non-catchable signal, it is impossible to switch the mode on the fly. Thus we have
         * to decide the mode at the startup time with the cRxDiscardOverrun registry key.
         */

        uint64_t rxDiscardOverrun = 0u;
        (void) IasAvbStreamHandlerEnvironment::getConfigValue(IasRegKeys::cRxDiscardOverrun, rxDiscardOverrun);

        uint32_t srrCtlReg = 0u;
        const uint32_t cSrrCtlDropEn = 0x80000000;

        (void) igb_readreg(mIgbDevice, SRRCTL(eRxQueue0), &srrCtlReg);

        if (0u == rxDiscardOverrun)
        {
          srrCtlReg &= ~cSrrCtlDropEn;  // disable
        }
        else
        {
          srrCtlReg |= cSrrCtlDropEn;   // enable
        }

        (void) igb_writereg(mIgbDevice, SRRCTL(eRxQueue0), srrCtlReg);

        DLT_LOG_CXX(*mLog, DLT_LOG_INFO, LOG_PREFIX, "Rx Overrun Drop:", (0u != rxDiscardOverrun) ? "on" : "off");
        DLT_LOG_CXX(*mLog, DLT_LOG_DEBUG, LOG_PREFIX, "SRRCTL:", srrCtlReg);

        /* success */
        result = eIasAvbProcOK;
      }

      (void) igb_unlock(mIgbDevice);
    }
  }
  else
  {
    if (NULL != mIgbDevice)
    {
      if (0 != igb_lock(mIgbDevice))
      {
        DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, "IGB lock failure");
        result = eIasAvbProcErr;
      }
      else
      {
        (void) igb_clear_flex_filter(mIgbDevice, eRxFilter0);

        /*
         * Disable the queue before releasing the memory allocated to this
         * queue. Also it will make sure that the filter is disabled and
         * all received packets are routed to the normal queue.
         */
        (void) igb_writereg(mIgbDevice, RXDCTL(eRxQueue0), 0u);

        (void) igb_unlock(mIgbDevice);
      }
    }
  }

  return result;
}

IasAvbProcessingResult IasAvbIgbDriver::prepareRcvPkts(void)
{
  IasAvbProcessingResult result = eIasAvbProcOK;

  IasAvbPacket* packet = NULL;

  if (mRcvPacketPool)
  {
    while ((packet = mRcvPacketPool->getPacket()) != NULL)
    {
      if (igb_refresh_buffers(mIgbDevice, eRxQueue0, reinterpret_cast<igb_packet**>(&packet), 1u) != 0)
      {
        DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, "failed to refresh RX buffer",
                    int32_t(errno), " (", strerror(errno), ")");
        break;
      }

      mPacketList.push_back(packet);
    }
  }
  else
  {
    result = eIasAvbProcErr;
  }

  return result;
}

IasAvbProcessingResult IasAvbIgbDriver::receive(IasAvbPacket **packet, uint32_t queue_index)
{
  (void)queue_index;

  IasAvbProcessingResult result = eIasAvbProcOK;

  uint32_t count = 1;
  if (igb_receive(mIgbDevice, eRxQueue0, reinterpret_cast<struct igb_packet **>(packet), &count) == 0)
  {

  }
  else
  {
    /*
     * RCTL.RXEN bit could mistakenly be turned off as initializing i210's direct rx mode if some programs
     * such as ifconfig or commnand concurrently access network interface on i210. This will drop all
     * incoming packets. Root cause is synchronization problem between libigb (user-side) and igb_avb
     * (kernel-side). As a workaround, monitor the bit if there is no incoming packet and enable it in case.
     * (defect: 201518)
     */
    if (mRecoverIgbReceiver)
    {
      uint32_t rctlReg = 0u;
      (void) igb_readreg(mIgbDevice, RCTL, &rctlReg);
      if (!(rctlReg & RCTL_RXEN))
      {
        rctlReg |= RCTL_RXEN;
        (void) igb_writereg(mIgbDevice, RCTL, rctlReg);

        DLT_LOG_CXX(*mLog, DLT_LOG_DEBUG, LOG_PREFIX, "Rx IGB Recovery: enabled RCTL.RXEN ( regval =", rctlReg, ")");
      }
    }

    result = eIasAvbProcErr;
  }

  return result;
}

IasAvbProcessingResult IasAvbIgbDriver::reclaimRcvPacket(IasAvbPacket *packet)
{
  IasAvbProcessingResult result = eIasAvbProcErr;

  if (packet)
  {
    if (igb_refresh_buffers(mIgbDevice, eRxQueue0, reinterpret_cast<struct igb_packet **>(&packet), 1u) == 0)
    {
      result = eIasAvbProcOK;
    }
  }

  return result;
}

void IasAvbIgbDriver::emergencyShutdown()
{
  if (NULL != mIgbDevice)
  {
    if (0 == igb_lock(mIgbDevice))
    {
      // route all packets to the best-effort queue
      (void) igb_clear_flex_filter(mIgbDevice, eRxFilter0);

      /*
       * Disable queue 0 so that AVTP packets remaining in the I210's packet buffer can be discarded.
       * Otherwise pending packets might keep occupying the buffer space if the SRRCTL.Drop_En bit is 0 and
       * interfere with the reception of the best-effort packets even after the shutdown of AVBSH.
       */
      (void) igb_writereg(mIgbDevice, RXDCTL(eRxQueue0), 0u);

      (void) igb_unlock(mIgbDevice);

      igb_detach(mIgbDevice); // @@WARN: This will seg fault without the correct capabilities
    }
  }
}


}
