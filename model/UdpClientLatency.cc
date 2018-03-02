/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   UdpClientLatency.cc
 * Author: soyo
 * 
 * Created on 14 June 2017, 4:34 PM
 */

#include "ns3/log.h"
#include "ns3/ipv4-address.h"
#include "ns3/nstime.h"
#include "ns3/inet-socket-address.h"
#include "ns3/inet6-socket-address.h"
#include "ns3/socket.h"
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include "UdpClientLatency.h"
#include "seq-ts-header.h"
#include <cstdlib>
#include <cstdio>
#include "timetag.h"
namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("UdpClientLatency");

NS_OBJECT_ENSURE_REGISTERED (UdpClientLatency);

TypeId
UdpClientLatency::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UdpClientLatency")
    .SetParent<Application> ()
    .SetGroupName("Applications")
    .AddConstructor<UdpClientLatency> ()
    .AddAttribute ("MaxPackets",
                   "The maximum number of packets the application will send",
                   UintegerValue (100),
                   MakeUintegerAccessor (&UdpClientLatency::m_count),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("Interval",
                   "The time to wait between packets", TimeValue (Seconds (1.0)),
                   MakeTimeAccessor (&UdpClientLatency::m_interval),
                   MakeTimeChecker ())
    .AddAttribute ("RemoteAddress",
                   "The destination Address of the outbound packets",
                   AddressValue (),
                   MakeAddressAccessor (&UdpClientLatency::m_peerAddress),
                   MakeAddressChecker ())
    .AddAttribute ("RemotePort", "The destination port of the outbound packets",
                   UintegerValue (100),
                   MakeUintegerAccessor (&UdpClientLatency::m_peerPort),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("PacketSize",
                   "Size of packets generated. The minimum packet size is 12 bytes which is the size of the header carrying the sequence number and the time stamp.",
                   UintegerValue (1024),
                   MakeUintegerAccessor (&UdpClientLatency::m_size),
                   MakeUintegerChecker<uint32_t> (12,1500))
  ;
  return tid;
}

UdpClientLatency::UdpClientLatency ()
{
  NS_LOG_FUNCTION (this);
  m_sent = 0;
  m_socket = 0;
  m_sendEvent = EventId ();
  m_exp = CreateObject<ExponentialRandomVariable> ();
  m_uni = CreateObject<UniformRandomVariable> ();
}

UdpClientLatency::~UdpClientLatency ()
{
  NS_LOG_FUNCTION (this);
}

void
UdpClientLatency::SetRemote (Address ip, uint16_t port)
{
  NS_LOG_FUNCTION (this << ip << port);
  m_peerAddress = ip;
  m_peerPort = port;
}

void
UdpClientLatency::SetRemote (Address addr)
{
  NS_LOG_FUNCTION (this << addr);
  m_peerAddress = addr;
}

void
UdpClientLatency::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  Application::DoDispose ();
}

void
UdpClientLatency::StartApplication (void)
{
  NS_LOG_FUNCTION (this);

  if (m_socket == 0)
    {
      TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
      m_socket = Socket::CreateSocket (GetNode (), tid);
      if (Ipv4Address::IsMatchingType(m_peerAddress) == true)
        {
          m_socket->Bind ();
          m_socket->Connect (InetSocketAddress (Ipv4Address::ConvertFrom(m_peerAddress), m_peerPort));
        }
      else if (Ipv6Address::IsMatchingType(m_peerAddress) == true)
        {
          m_socket->Bind6 ();
          m_socket->Connect (Inet6SocketAddress (Ipv6Address::ConvertFrom(m_peerAddress), m_peerPort));
        }
      else if (InetSocketAddress::IsMatchingType (m_peerAddress) == true)
        {
          m_socket->Bind ();
          m_socket->Connect (m_peerAddress);
        }
      else if (Inet6SocketAddress::IsMatchingType (m_peerAddress) == true)
        {
          m_socket->Bind6 ();
          m_socket->Connect (m_peerAddress);
        }
      else
        {
          NS_ASSERT_MSG (false, "Incompatible address type: " << m_peerAddress);
        }
    }

  m_socket->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ());
  m_socket->SetAllowBroadcast (true);
  m_exp->SetAttribute ("Mean", DoubleValue (m_interval.GetMicroSeconds()));

  m_sendEvent = Simulator::Schedule (MicroSeconds(m_exp->GetValue()), &UdpClientLatency::Send, this);
}

void
UdpClientLatency::StopApplication (void)
{
  NS_LOG_FUNCTION (this);
  Simulator::Cancel (m_sendEvent);
}

void
UdpClientLatency::Send (void)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT (m_sendEvent.IsExpired ());
//  if(m_sendEvent.IsExpired ()){
//      std::cout <<  "ex" << std::endl;
//  }
  SeqTsHeader seqTs;
  seqTs.SetSeq (m_sent);
  uint32_t this_size = m_uni->GetValue(m_size/2,m_size/2*3);
  Ptr<Packet> p = Create<Packet> (this_size-(8+4)); // 8+4 : the size of the seqTs header
  p->AddHeader (seqTs);
  TimestampTag timestamp;
  timestamp.SetTimestamp (Simulator::Now ());
  p->AddByteTag (timestamp);
  std::stringstream peerAddressStringStream;
  if (Ipv4Address::IsMatchingType (m_peerAddress))
    {
      peerAddressStringStream << Ipv4Address::ConvertFrom (m_peerAddress);
    }
  else if (Ipv6Address::IsMatchingType (m_peerAddress))
    {
      peerAddressStringStream << Ipv6Address::ConvertFrom (m_peerAddress);
    }

  if ((m_socket->Send (p)) >= 0)
    {
      ++m_sent;
      NS_LOG_INFO ("TraceDelay TX " << m_size << " bytes to "
                                    << peerAddressStringStream.str () << " Uid: "
                                    << p->GetUid () << " Time: "
                                    << (Simulator::Now ()).GetSeconds ());

    }
  else
    {
//      std::cout <<  "er" << std::endl;

      NS_LOG_INFO ("Error while sending " << m_size << " bytes to "
                                          << peerAddressStringStream.str ());
    }

  if (m_sent < m_count)
    {
      m_sendEvent = Simulator::Schedule (MicroSeconds(m_exp->GetValue()), &UdpClientLatency::Send, this);
    }


}
void
UdpClientLatency::Setimsi(uint64_t imsi) 
{
    m_imsi = imsi;
} 
} // Namespace ns3


