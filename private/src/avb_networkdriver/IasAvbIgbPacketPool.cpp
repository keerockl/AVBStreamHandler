/*
 @COPYRIGHT_TAG@
 */
/**
 * @file    IasAvbIgbPacketPool.cpp
 * @brief   The implementation of the IasAvbIgbPacketPool class.
 * @date    2018
 */

#include "avb_networkdriver/IasAvbIgbPacketPool.hpp"
#include <dlt_cpp_extension.hpp>

namespace IasMediaTransportAvb {

static const std::string cClassName = "IasAvbIgbPacketPool::";
#define LOG_PREFIX cClassName + __func__ + "(" + std::to_string(__LINE__) + "):"

IasAvbIgbPacketPool::IasAvbIgbPacketPool(DltContext &dltContext, device_t* igbdev)
  : IasAvbPacketPool(dltContext)
  , mIgbDevice(igbdev)
  , mDmaPages()
{
}

IasAvbIgbPacketPool::~IasAvbIgbPacketPool()
{
  cleanup();
}

IasAvbProcessingResult IasAvbIgbPacketPool::derivedInit()
{
  IasAvbProcessingResult ret = eIasAvbProcOK;
  device_t* igbDevice = mIgbDevice;
  Page* page = NULL;

  if (NULL == igbDevice)
  {
    /*
     * @log Init failed: Returned igbDevice == nullptr
     */
    DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, " Failed to getIgbDevice!");
    ret = eIasAvbProcInitializationFailed;
  }
  else
  {
    page = new (nothrow) Page;

    if (NULL == page)
    {
      /*
       * @log Not enough memory: Couldn't allocate DMA page, Page corresponds to underlying igb_dma_alloc struct.
       */
      DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, " Not enough memory to allocate Page!");
      ret = eIasAvbProcNotEnoughMemory;
    }
  }

  if (eIasAvbProcOK == ret)
  {
    // allocate one DMA page to retrieve properties
    if (0 != igb_dma_malloc_page( igbDevice, page ))
    {
      /*
       * @log Init failed: Failed to retrieve DMA page.
       */
      DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, " Failed to igb_dma_malloc_page!");
      ret = eIasAvbProcInitializationFailed;
    }
    else
    {
      uint32_t packetCountTotal = 0u;
      const size_t packetSize = getPacketSize();
      const uint32_t poolSize = getPoolSize();

      // find out how many packets fit into a page
      const uint32_t packetsPerPage = uint32_t( size_t(page->mmap_size) / packetSize );

      if (0u == packetsPerPage)
      {
        // packetSize larger than dma page size - not supported by libigb
        /*
         * @log Unsupported format: Packet size is larger than the dma page size - not supported by libigb.
         */
        DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, " packet size > ",
            page->mmap_size, "not supported!");
        ret = eIasAvbProcUnsupportedFormat;
      }
      else
      {
        const uint32_t pagesNeeded = (poolSize + (packetsPerPage - 1u)) / packetsPerPage;

        DLT_LOG_CXX(*mLog, DLT_LOG_DEBUG, LOG_PREFIX, " DMA page overhead (bytes:",
            (uint64_t(pagesNeeded) * uint64_t(page->mmap_size)) - (uint64_t(poolSize) * uint64_t(packetSize)));

        mDmaPages.reserve(pagesNeeded);

        ret = initPage( page, packetsPerPage, packetCountTotal );

        // 1st page is already allocated
        for (uint32_t pageCount = 1u; (eIasAvbProcOK == ret) && (pageCount < pagesNeeded); pageCount++)
        {
          page = new (nothrow) Page;

          if (NULL == page)
          {
            DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, " Not enough memory to allocate Page!");
            ret = eIasAvbProcNotEnoughMemory;
          }
          else if (0 != igb_dma_malloc_page( igbDevice, page ))
          {
            DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, " igb dma memory allocation failure");
            ret = eIasAvbProcInitializationFailed;
          }
          else
          {
            ret = initPage( page, packetsPerPage, packetCountTotal );
          }
        }
      }
    }
  }

  return eIasAvbProcOK;
}

IasAvbProcessingResult IasAvbIgbPacketPool::initPage(Page * page, const uint32_t packetsPerPage, uint32_t & packetCountTotal)
{
  IasAvbProcessingResult ret = eIasAvbProcOK;

  AVB_ASSERT( NULL != getPoolBase() );
  AVB_ASSERT( NULL != page );
  AVB_ASSERT( mPacketSize > 0u );

  // add page to list
  mDmaPages.push_back(page);

  for (uint32_t packetIdx = 0u; (packetIdx < packetsPerPage) && (packetCountTotal < getPoolSize()); packetIdx++, packetCountTotal++)
  {
    /*
     *  assign unique section of dma page to each igb packet
     *  Note that this is not touched anymore during subsequent operation!
     */
    IasAvbPacket & packet = getPoolBase()[packetCountTotal];

    packet.offset = uint32_t( packetIdx * getPacketSize() );
    packet.vaddr = static_cast<uint8_t*>(page->dma_vaddr) + packet.offset;
    packet.map.mmap_size = page->mmap_size;
    packet.map.paddr = page->dma_paddr;

    packet.setHomePool( this );
    ret = returnPacket(&packet);
    if (eIasAvbProcOK != ret)
    {
      DLT_LOG_CXX(*mLog, DLT_LOG_ERROR, LOG_PREFIX, " Unable to prepare a packet");
      break;
    }
  }

  return ret;
}

void IasAvbIgbPacketPool::derivedCleanup()
{
  if (mIgbDevice)
  {
    while (!mDmaPages.empty())
    {
      Page* page = mDmaPages.back();
      mDmaPages.pop_back();

      AVB_ASSERT( NULL != page );

      igb_dma_free_page( mIgbDevice, page );
      delete page;
    }
  }
}


} // namespace IasMediaTransportAvb
