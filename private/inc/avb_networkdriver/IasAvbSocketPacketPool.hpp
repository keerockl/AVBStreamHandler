/*
 @COPYRIGHT_TAG@
 */
/**
 * @file    IasAvbPacketPool.hpp
 * @brief   The definition of the IasAvbIgbPacketPool class.
 * @details This is the pool for AVB packets designated to socket based driver
 * @date    2018
 */

#ifndef IASAVBSOCKETPACKETPOOL_HPP_
#define IASAVBSOCKETPACKETPOOL_HPP_

#include "IasAvbPacketPool.hpp"

namespace IasMediaTransportAvb
{

class IasAvbSocketPacketPool : public IasAvbPacketPool
{
  public:
    /*!
     * @brief constructor
     */
    IasAvbSocketPacketPool(DltContext &dltContext);

    /**
     * @brief destructor
     */
    virtual ~IasAvbSocketPacketPool();

  protected:
    /**
     * @brief free allocated heap memory buffers
     */
    virtual IasAvbProcessingResult derivedInit();

    /**
     * @brief bind heap memory buffers to packet objects
     */
    virtual void derivedCleanup();

  private:
    // Local Types
    typedef uint64_t Page;
    typedef std::vector<Page*> PageList;

    PageList mPages;
};


} // namespace IasMediaTransportAvb

#endif /* IASAVBSOCKETPACKETPOOL_HPP_ */
