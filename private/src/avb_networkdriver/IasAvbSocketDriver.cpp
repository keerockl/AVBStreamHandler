/*
  @COPYRIGHT_TAG@
*/
/**
 *  @file    IasAvbSocketDriver.cpp
 *  @brief   Library for the cmesg socket based Network Drivers.
 *  @date    2018
 */

#include "avb_networkdriver/IasAvbSocketDriver.hpp"
#include "avb_networkdriver/IasAvbSocketPacketPool.hpp"
#include "avb_streamhandler/IasAvbStreamHandlerEnvironment.hpp"
#include <dlt_cpp_extension.hpp>
#include <cstring>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <net/if.h>


namespace IasMediaTransportAvb {

static const std::string cClassName = "IasAvbSocketDriver::";
#define LOG_PREFIX cClassName + __func__ + "(" + std::to_string(__LINE__) + "):"

#ifndef SO_TXTIME
#define SO_TXTIME 61
#endif

#ifndef SCM_DROP_IF_LATE
#define SCM_DROP_IF_LATE 62
#endif

#ifndef ETH_P_IEEE1722
#define ETH_P_IEEE1722 0x22F0
#endif

IasAvbSocketDriver::IasAvbSocketDriver(DltContext &dltContext)
  : IasAvbNetworkDriver(dltContext)
  , mRcvPacketPool(nullptr)
  , mXmitPacketList()
  , mTransmitSocket(-1)
  , mReceiveSocket(-1)
  , mReceivePacketSize(0u)
{
}

IasAvbSocketDriver::~IasAvbSocketDriver()
{
  cleanup();
}

void IasAvbSocketDriver::derivedCleanup()
{
  (void) closeTransmitSocket();
  (void) closeReceiveSocket();
}

IasAvbProcessingResult IasAvbSocketDriver::derivedInit()
{
  IasAvbProcessingResult ret = eIasAvbProcErr;
  ret = setShaperMode(false);

  if (ret == eIasAvbProcOK)
  {
    ret = openTransmitSocket();
  }

  if (ret == eIasAvbProcOK)
  {
    ret = openReceiveSocket();
  }

  if (ret != eIasAvbProcOK)
  {
    (void) cleanup();
  }

  return ret;
}

IasAvbProcessingResult IasAvbSocketDriver::createPacketPool(DltContext &dltContext, IasAvbStreamDirection direction,
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

    tmpPool = new (nothrow) IasAvbSocketPacketPool(dltContext);
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
      mReceivePacketSize = static_cast<int32_t>(packetSize);
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

void IasAvbSocketDriver::destroyPacketPool(IasAvbPacketPool *pool)
{
  if (pool)
  {
    if (mRcvPacketPool == pool)
    {
      mRcvPacketPool = nullptr;
    }
    delete pool;
  }
}


int32_t IasAvbSocketDriver::xmit(IasAvbPacket *packet, uint32_t queue_index)
{
  (void)queue_index;

  int32_t result = EINVAL;
  uint32_t frameLen = 0u;
  uint8_t *buffer = nullptr;

  if (packet)
  {
    frameLen = packet->len;
    buffer = (uint8_t *)packet->getBasePtr();
  }

  if (nullptr != buffer)
  {
    uint8_t *dest_addr = buffer;

    struct cmsghdr *cmsg;
    struct iovec iov;
    struct msghdr msg = {};
#if 0
    char control[CMSG_SPACE(sizeof(uint64_t)) + CMSG_SPACE(sizeof(uint8_t))] = {};
#else
    char control[CMSG_SPACE(sizeof(uint64_t))] = {};
#endif

    //const uint8_t drop_if_late = 1;

    /* non-reentrant since mutex control is not given to mDescSockAddr */
    memcpy(mDescSockAddr.sll_addr, dest_addr, ETH_ALEN);

    msg.msg_name = (struct sockaddr *)&mDescSockAddr;
    msg.msg_namelen = sizeof(struct sockaddr_ll);

    /* A struct iovec is a base pointer/length pair that is one element of a
    scatter/gather vector; we only need a 1-element vector */
    iov.iov_base = buffer;
    iov.iov_len = frameLen;
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;

    /* Our control message is stored in the weird union value u, which has some odd size and
    alignment requirements I don't fully understand yet */
    msg.msg_control = control;
    msg.msg_controllen = sizeof(control);

    /* The cmsg API is full of weird unfortunate macros that are poorly documented. */
    cmsg = CMSG_FIRSTHDR(&msg); // This extracts the first cmsghdr address from the msg
    cmsg->cmsg_level = SOL_SOCKET; // handled in socket driver for AF_PACKET sockets
    cmsg->cmsg_type = SO_TXTIME; // This is the new message type we added
    cmsg->cmsg_len = CMSG_LEN(sizeof(uint64_t));
    *((__u64 *) CMSG_DATA(cmsg)) = packet->attime;

	//keerock
    //cmsg = CMSG_NXTHDR(&msg, cmsg);
    //cmsg->cmsg_level = SOL_SOCKET;
    //cmsg->cmsg_type = SCM_DROP_IF_LATE;
    //cmsg->cmsg_len = CMSG_LEN(sizeof(uint8_t));
    //*((__u64 *) CMSG_DATA(cmsg)) = drop_if_late;

    /* Send the packet */
    errno = 0;

    ssize_t xmitBytes = sendmsg(mTransmitSocket, &msg, 0);
    if (0 > xmitBytes)
    {
      result = errno;
    }
    else
    {
      mXmitPacketList.push_back(packet);
      result = 0;
    }
  }

  return result;
}

uint32_t IasAvbSocketDriver::reclaimPackets()
{
  uint32_t ret = 0u;
  IasAvbPacket *packet = NULL;

  // check and return packets that are not used any longer
  // NOTE: this is done for all sequencers, not only for this one!
  while (!mXmitPacketList.empty())
  {
    packet = mXmitPacketList.back();
    if (NULL != packet)
    {
      (void) IasAvbPacketPool::returnPacket(packet);
      ret++;
    }
    (void) mXmitPacketList.pop_back();
  }

  return ret;
}

IasAvbProcessingResult IasAvbSocketDriver::setShaperMode(bool enable)
{
  // TO-DO : enable/disable the shaper by using CBS qdisc
  IasAvbProcessingResult ret = eIasAvbProcOK;
  if (enable)
  {

  }
  else
  {
  }

  return ret;
}

void IasAvbSocketDriver::updateShaper(double reqBandWidth, uint32_t queue_index, uint32_t maxFrameSizeHigh)
{
  // TO-DO : update the shaper based on required bandwidth
  (void)reqBandWidth;
  (void)queue_index;
  (void)maxFrameSizeHigh;
}

IasAvbProcessingResult IasAvbSocketDriver::getPtpToClockX(uint64_t &ptpTime, uint64_t &sysTime, const clockid_t clockId,
                                                       uint64_t maxRetry, uint64_t maxMeasurementInterval)
{
  (void)ptpTime;
  (void)sysTime;
  (void)clockId;
  (void)maxRetry;
  (void)maxMeasurementInterval;

  // TO-DO : get current gPTP time and CLOCK_MONOTONIC or TSC time by using PTP_TO_SYS or PTP_SYS_OFFSET_PRECISE ioctl
  IasAvbProcessingResult result = eIasAvbProcErr;
  return result;
}

IasAvbProcessingResult IasAvbSocketDriver::receive(IasAvbPacket **packet, uint32_t queue_index)
{
  (void)queue_index;

  IasAvbProcessingResult result = eIasAvbProcErr;
  int32_t recv_length = 0;
  IasAvbPacket *rcvPkt = nullptr;

  if (mRcvPacketPool)
  {
    rcvPkt = mRcvPacketPool->getPacket();
  }

  if (rcvPkt)
  {
    recv_length = static_cast<int32_t>(recvfrom(mReceiveSocket, rcvPkt->getBasePtr(), mReceivePacketSize, MSG_DONTWAIT, NULL, NULL ));
    if (recv_length > 0)
    {
      rcvPkt->len = recv_length;
      *packet = rcvPkt;
      result = eIasAvbProcOK;
    }
    else
    {
      IasAvbSocketDriver::reclaimRcvPacket(rcvPkt);
    }
  }

  return result;
}

IasAvbProcessingResult IasAvbSocketDriver::reclaimRcvPacket(IasAvbPacket *packet)
{
  IasAvbProcessingResult result = eIasAvbProcErr;

  if (packet)
  {
    (void) IasAvbPacketPool::returnPacket(packet);
    result = eIasAvbProcOK;
  }

  return result;
}


//keerockl
static int use_deadline_mode = 0;
static int receive_errors = 0;
struct sock_txtime {
	clockid_t clockid;
	uint16_t flags;
}; //keerockl

IasAvbProcessingResult IasAvbSocketDriver::openTransmitSocket()
{
  IasAvbProcessingResult result = eIasAvbProcErr;
  static struct sock_txtime sk_txtime; //keerockl

  if (-1 != mTransmitSocket)
  {
    result = eIasAvbProcInitializationFailed;
  }
  else
  {
    mTransmitSocket = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (0 > mTransmitSocket)
    {
      DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, "socket open error", errno);
    }
    else
    {
    	sk_txtime.clockid = CLOCK_TAI;
		sk_txtime.flags = (uint16_t)(use_deadline_mode | receive_errors);
      //int on = 1;
      if (setsockopt(mTransmitSocket, SOL_SOCKET, SO_TXTIME, &sk_txtime, sizeof(sk_txtime)) < 0) // to be updated
      {
        DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, "socket SO_TXTIME option error 1", strerror(errno));
      }
      else
      {
        int priority = 3; // to be adjustable
        if (setsockopt(mTransmitSocket, SOL_SOCKET, SO_PRIORITY, &priority, sizeof(priority)) < 0)
        {
          DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, "socket SO_PRIORITY option error 2", strerror(errno));
        }
        else
        {
          mDescSockAddr.sll_family = AF_PACKET;
          mDescSockAddr.sll_protocol = htons(ETH_P_ALL);

          std::string ifname = mInterfaceName;
//          const uint64_t vlanid = 3u; // to be adjustable
          const uint64_t vlanid = 0u; // to be adjustable
          if (0u != vlanid)
          {
            ifname = ifname + "." + std::to_string(vlanid);
          }

          mDescSockAddr.sll_ifindex = if_nametoindex(ifname.c_str());
          mDescSockAddr.sll_halen = ETH_ALEN;
          DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, "ifname:", ifname.c_str(), " ifindex:", mDescSockAddr.sll_ifindex );
          result = eIasAvbProcOK;
        }
      }
    }
  }

  return result;
}

IasAvbProcessingResult IasAvbSocketDriver::openReceiveSocket()
{
  IasAvbProcessingResult result = eIasAvbProcOK;

  struct sockaddr_ll recv_sa;
  struct ifreq ifr;
  int32_t ifindex;
  typedef int Int; // avoid complaints about naked fundamental types
  Int bufSize = 0u;
  socklen_t argSize = sizeof bufSize;
  const std::string* recvport = IasAvbStreamHandlerEnvironment::getNetworkInterfaceName();

  DLT_LOG_CXX(*mLog, DLT_LOG_VERBOSE, LOG_PREFIX);

  // already opened or error querying network interface name
  if ((-1 != mReceiveSocket) || (NULL == recvport))
  {
    result = eIasAvbProcInitializationFailed;
  }

  if (eIasAvbProcOK == result)
  {
    // open socket in raw mode
    mReceiveSocket = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_IEEE1722));

    if (mReceiveSocket == -1)
    {
      /**
       * @log Init failed: Raw receive socket could not be opened.
       */

      DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX,  "mIgbDevice == NULL!");
      result = eIasAvbProcInitializationFailed;
    }
  }

  if (eIasAvbProcOK == result)
  {
    // verify if network interface exists
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, recvport->c_str(), (sizeof ifr.ifr_name) - 1u);
    ifr.ifr_name[sizeof(ifr.ifr_name) - 1] = '\0';

    if (ioctl(mReceiveSocket, SIOCGIFINDEX, &ifr) == -1)
    {
      /**
       * @log Init failed: Interface index not recognised.
       */
      DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, "No such interface: ",
          recvport->c_str());
      result = eIasAvbProcInitializationFailed;
    }
    else
    {
      ifindex = ifr.ifr_ifindex;
    }
  }

  if (eIasAvbProcOK == result)
  {
    // Read flags from network interface...
    strncpy(ifr.ifr_name, recvport->c_str(), (sizeof ifr.ifr_name) - 1u);
    ifr.ifr_name[(sizeof ifr.ifr_name) - 1u] = '\0';

    if (ioctl(mReceiveSocket, SIOCGIFFLAGS, &ifr) < 0)
    {
      /**
       * @log Init failed: IOCTL could not read out the flags from the NIC.
       */
      DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX,  "ioctl fails read out the current flags: ",
          int32_t(errno), " (", strerror(errno), ")");
      result = eIasAvbProcInitializationFailed;
    }
    else if ((ifr.ifr_flags & IFF_UP) == 0)
    {
      DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, "Warning: Interface ",
          recvport->c_str(), " is down");
    }
    else if ((ifr.ifr_flags & IFF_RUNNING) == 0)
    {
      DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, "Warning: Interface ",
          recvport->c_str(), "  is up, but not connected");
    }
    else
    {
      DLT_LOG_CXX(*mLog, DLT_LOG_INFO, LOG_PREFIX, "flags: ",
          ((ifr.ifr_flags & IFF_PROMISC) ? "PROMISC ON" : "PROMISC OFF"),
          ((ifr.ifr_flags & IFF_MULTICAST) ? "MULTICAST ON" : "MULTICAST OFF"),
          ((ifr.ifr_flags & IFF_ALLMULTI) ? "ALLMULTI ON" : "ALLMULTI OFF")
          );
    }
  }

  if (eIasAvbProcOK == result)
  {
    if (IasAvbStreamHandlerEnvironment::getConfigValue(IasRegKeys::cRxSocketRxBufSize, bufSize))
    {
      // This is not needed for the direct RX mode and even worse it requires the cap_net_admin privilege.
      if (setsockopt(mReceiveSocket, SOL_SOCKET, SO_RCVBUFFORCE, &bufSize, sizeof bufSize) < 0)
      {
        /**
         * @log Init failed: The receive buffer size could not be set.
         */
        DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, "failed to set RCV buffer size: ",
            int32_t(errno), " (", strerror(errno), ")");
        result = eIasAvbProcInitializationFailed;
      }
    }
  }

  if (eIasAvbProcOK == result)
  {
    if (getsockopt(mReceiveSocket, SOL_SOCKET, SO_RCVBUF, &bufSize, &argSize) < 0)
    {
      DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, "warning: failed to read RCV buffer size: ",
          int32_t(errno), " (", strerror(errno), ")");
    }
    else
    {
      DLT_LOG_CXX(*mLog, DLT_LOG_INFO, LOG_PREFIX, "socket RCV buffer size is ",
          (bufSize/2), " (", bufSize , "real)");
    }
  }

  if (eIasAvbProcOK == result)
  {
    memset(&recv_sa, 0, sizeof recv_sa);
    recv_sa.sll_family = AF_PACKET;
    recv_sa.sll_ifindex = ifindex;
    recv_sa.sll_protocol = htons(ETH_P_IEEE1722);
    recv_sa.sll_hatype = PACKET_MULTICAST;

    if (bind(mReceiveSocket, reinterpret_cast<sockaddr*>(&recv_sa), sizeof recv_sa) == -1)
    {
      /**
       * @log Init failed: Unable to bind socket to the interface.
       */
      DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, "error binding socket to if",
          int32_t(ifindex), "(", ifr.ifr_name, ":", strerror(errno), ")");
      result = eIasAvbProcInitializationFailed;
    }
    else
    {
      DLT_LOG_CXX(*mLog, DLT_LOG_INFO, LOG_PREFIX, "bound socket to if",
          int32_t(ifindex), "(", ifr.ifr_name, ")");
    }
  }

  if (eIasAvbProcOK != result)
  {
    closeReceiveSocket();
  }

  return result;
}

void IasAvbSocketDriver::emergencyShutdown()
{

}


}
