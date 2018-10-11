/*
 @COPYRIGHT_TAG@
 */
/**
 * @file    IasAvbIgbPacketPool.hpp
 * @brief   The definition of the IasAvbIgbPacketPool class.
 * @details This is the pool for AVB packets designated to igb_avb driver
 * @date    2018
 */

#ifndef IASAVBIGBPACKETPOOL_HPP_
#define IASAVBIGBPACKETPOOL_HPP_

#include "IasAvbPacketPool.hpp"

extern "C"
{
  #include "igb.h"
}

namespace IasMediaTransportAvb
{

class /*IAS_DSO_PUBLIC*/ IasAvbIgbPacketPool : public IasAvbPacketPool
{
  public:
    /*!
     * @brief constructor
     */
    IasAvbIgbPacketPool(DltContext &dltContext, device_t* igbdev);

    /**
     * @brief destructor
     */
    virtual ~IasAvbIgbPacketPool();

  protected:
    /**
     * @brief bind DMA capable memory buffers to packet objects
     */
    virtual IasAvbProcessingResult derivedInit();

    /**
     * @brief free allocated DMA capable memory buffers
     */
    virtual void derivedCleanup();

  private:
    // Local Types
    typedef igb_dma_alloc Page;
    typedef std::vector<Page*> PageList;

    // helpers
    IasAvbProcessingResult initPage(Page * page, const uint32_t packetsPerPage, uint32_t & packetCountTotal);

    device_t* mIgbDevice;
    PageList mDmaPages;
};


} // namespace IasMediaTransportAvb

#endif /* IASAVBIGBPACKETPOOL_HPP_ */
