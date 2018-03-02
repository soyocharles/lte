/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   UdpClientLatency.h
 * Author: soyo
 *
 * Created on 14 June 2017, 4:34 PM
 */

#ifndef UDPCLIENTLATENCY_H
#define UDPCLIENTLATENCY_H

#include "ns3/application.h"
#include "ns3/event-id.h"
#include "ns3/ptr.h"
#include "ns3/ipv4-address.h"
#include "ns3/random-variable-stream.h"
namespace ns3 {

class Socket;
class Packet;

/**
 * \ingroup 
 *
 * \brief A Udp client. Sends UDP packet carrying sequence number and time stamp
 *  in their payloads
 *
 */
class UdpClientLatency : public Application
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  UdpClientLatency ();

  virtual ~UdpClientLatency ();

  /**
   * \brief set the remote address and port
   * \param ip remote IP address
   * \param port remote port
   */
  void SetRemote (Address ip, uint16_t port);
  /**
   * \brief set the remote address
   * \param addr remote address
   */
  void SetRemote (Address addr);
  void Setimsi(uint64_t imsi) ;
protected:
  virtual void DoDispose (void);

private:

  virtual void StartApplication (void);
  virtual void StopApplication (void);

  /**
   * \brief Send a packet
   */
  void Send (void);

  uint32_t m_count; //!< Maximum number of packets the application will send
  Time m_interval; //!< Packet inter-send time
  uint32_t m_size; //!< Size of the sent packet (including the SeqTsHeader)

  uint32_t m_sent; //!< Counter for sent packets
  Ptr<Socket> m_socket; //!< Socket
  Address m_peerAddress; //!< Remote peer address
  uint16_t m_peerPort; //!< Remote peer port
  EventId m_sendEvent; //!< Event to send the next packet
  uint64_t m_imsi;
  Ptr<ExponentialRandomVariable> m_exp;
  Ptr<UniformRandomVariable> m_uni;

};

} // namespace ns3

#endif /* UDPCLIENTLATENCY_H */

