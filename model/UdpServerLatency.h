/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   UdpServerLatency.h
 * Author: soyo
 *
 * Created on 14 June 2017, 4:38 PM
 */

#ifndef UDPSERVERLATENCY_H
#define UDPSERVERLATENCY_H

#include "ns3/application.h"
#include "ns3/event-id.h"
#include "ns3/ptr.h"
#include "ns3/address.h"
#include "packet-loss-counter.h"
namespace ns3 {
/**
 * \ingroup applications
 * \defgroup udpclientserver UdpClientServer
 */

/**
 * \ingroup udpclientserver
 *
 * \brief A UDP server, receives UDP packets from a remote host.
 *
 * UDP packets carry a 32bits sequence number followed by a 64bits time
 * stamp in their payloads. The application uses the sequence number
 * to determine if a packet is lost, and the time stamp to compute the delay.
 */
class UdpServerLatency : public Application
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  UdpServerLatency ();
  virtual ~UdpServerLatency ();
  /**
   * \brief Returns the number of lost packets
   * \return the number of lost packets
   */
  uint32_t GetLost (void) const;

  /**
   * \brief Returns the number of received packets
   * \return the number of received packets
   */
  uint64_t GetReceived (void) const;

  /**
   * \brief Returns the size of the window used for checking loss.
   * \return the size of the window used for checking loss.
   */
  uint16_t GetPacketWindowSize () const;
  double GetAverageLatency(void) ;  
    uint64_t GetReceivedTaged(void);
  /**
   * \brief Set the size of the window used for checking loss. This value should
   *  be a multiple of 8
   * \param size the size of the window used for checking loss. This value should
   *  be a multiple of 8
   */
  void SetPacketWindowSize (uint16_t size);
  void Setimsi(uint64_t imsi); 
  std::vector<std::pair<uint32_t,std::pair<double,double>>> m_data;

protected:
  virtual void DoDispose (void);

private:

  virtual void StartApplication (void);
  virtual void StopApplication (void);

  /**
   * \brief Handle a packet reception.
   *
   * This function is called by lower layers.
   *
   * \param socket the socket the packet was received to.
   */
  void HandleRead (Ptr<Socket> socket);

  uint16_t m_port; //!< Port on which we listen for incoming packets.
  Ptr<Socket> m_socket; //!< IPv4 Socket
  Ptr<Socket> m_socket6; //!< IPv6 Socket
  uint64_t m_received; //!< Number of received packets
  PacketLossCounter m_lossCounter; //!< Lost packet counter
  uint64_t m_received_taged; //!< Number of received packets
  uint64_t m_total_latency;
  uint64_t m_imsi;

};

} // namespace ns3

#endif /* UDPSERVERLATENCY_H */

