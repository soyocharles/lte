/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   UdpServerLatency.cc
 * Author: soyo
 * 
 * Created on 14 June 2017, 4:38 PM
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
#include "packet-loss-counter.h"
#include "timetag.h"
#include "seq-ts-header.h"
#include "UdpServerLatency.h"
#include "UdpClientLatency.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("UdpServerLatency");

NS_OBJECT_ENSURE_REGISTERED (UdpServerLatency);


TypeId
UdpServerLatency::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UdpServerLatency")
    .SetParent<Application> ()
    .SetGroupName("Applications")
    .AddConstructor<UdpServerLatency> ()
    .AddAttribute ("Port",
                   "Port on which we listen for incoming packets.",
                   UintegerValue (100),
                   MakeUintegerAccessor (&UdpServerLatency::m_port),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("PacketWindowSize",
                   "The size of the window used to compute the packet loss. This value should be a multiple of 8.",
                   UintegerValue (32),
                   MakeUintegerAccessor (&UdpServerLatency::GetPacketWindowSize,
                                         &UdpServerLatency::SetPacketWindowSize),
                   MakeUintegerChecker<uint16_t> (8,256))
  ;
  return tid;
}

UdpServerLatency::UdpServerLatency ()
  : m_lossCounter (0)
{
  NS_LOG_FUNCTION (this);
  m_received=0;
  m_received_taged = 0;
  m_total_latency = 0;
}

UdpServerLatency::~UdpServerLatency ()
{
  NS_LOG_FUNCTION (this);
}

uint16_t
UdpServerLatency::GetPacketWindowSize () const
{
  NS_LOG_FUNCTION (this);
  return m_lossCounter.GetBitMapSize ();
}

void
UdpServerLatency::SetPacketWindowSize (uint16_t size)
{
  NS_LOG_FUNCTION (this << size);
  m_lossCounter.SetBitMapSize (size);
}

uint32_t
UdpServerLatency::GetLost (void) const
{
  NS_LOG_FUNCTION (this);
  return m_lossCounter.GetLost ();
}

uint64_t
UdpServerLatency::GetReceived (void) const
{
  NS_LOG_FUNCTION (this);
  return m_received;
}

void
UdpServerLatency::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  Application::DoDispose ();
}

void
UdpServerLatency::StartApplication (void)
{
  NS_LOG_FUNCTION (this);

  if (m_socket == 0)
    {
      TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
      m_socket = Socket::CreateSocket (GetNode (), tid);
      InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (),
                                                   m_port);
      m_socket->Bind (local);
    }

  m_socket->SetRecvCallback (MakeCallback (&UdpServerLatency::HandleRead, this));

  if (m_socket6 == 0)
    {
      TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
      m_socket6 = Socket::CreateSocket (GetNode (), tid);
      Inet6SocketAddress local = Inet6SocketAddress (Ipv6Address::GetAny (),
                                                   m_port);
      m_socket6->Bind (local);
    }

  m_socket6->SetRecvCallback (MakeCallback (&UdpServerLatency::HandleRead, this));

}

void
UdpServerLatency::StopApplication ()
{
  NS_LOG_FUNCTION (this);

  if (m_socket != 0)
    {
      m_socket->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ());
    }
}

void
UdpServerLatency::HandleRead (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);
  Ptr<Packet> packet;
  Address from;
  while ((packet = socket->RecvFrom (from)))
    {
      if (packet->GetSize () > 0)
        {
          TimestampTag timestamp;
        if (packet->FindFirstMatchingByteTag (timestamp)) {
            m_received_taged ++;
          Time tx = timestamp.GetTimestamp ();
          Time now = Simulator::Now ();
          Time delay = (now - tx);
          m_total_latency =  m_total_latency + delay.GetMicroSeconds();
          std::pair<double,double> pair = std::pair<double,double>(delay.GetMicroSeconds(),now.GetMicroSeconds());
          m_data.push_back(std::pair<uint32_t,std::pair<double,double>>(packet->GetSize(),pair));
//          double tmp = this->GetAverageLatency();
//          std::cout << "latency " << tmp <<" id  " << m_imsi<< std::endl;
        }
          
          SeqTsHeader seqTs;
          packet->RemoveHeader (seqTs);
          

          uint32_t currentSequenceNumber = seqTs.GetSeq ();
          if (InetSocketAddress::IsMatchingType (from))
            {
              NS_LOG_INFO ("TraceDelay: RX " << packet->GetSize () <<
                           " bytes from "<< InetSocketAddress::ConvertFrom (from).GetIpv4 () <<
                           " Sequence Number: " << currentSequenceNumber <<
                           " Uid: " << packet->GetUid () <<
                           " TXtime: " << seqTs.GetTs () <<
                           " RXtime: " << Simulator::Now () <<
                           " Delay: " << Simulator::Now () - seqTs.GetTs ());
            }
          else if (Inet6SocketAddress::IsMatchingType (from))
            {
              NS_LOG_INFO ("TraceDelay: RX " << packet->GetSize () <<
                           " bytes from "<< Inet6SocketAddress::ConvertFrom (from).GetIpv6 () <<
                           " Sequence Number: " << currentSequenceNumber <<
                           " Uid: " << packet->GetUid () <<
                           " TXtime: " << seqTs.GetTs () <<
                           " RXtime: " << Simulator::Now () <<
                           " Delay: " << Simulator::Now () - seqTs.GetTs ());
            }

          m_lossCounter.NotifyReceived (currentSequenceNumber);
          m_received++;
        }
    }

}

double
UdpServerLatency::GetAverageLatency(void)
{
//    double a = (double)m_total_latency;
//    double r = a /( (double) m_received_taged);

//    std::cout << "latency " << r << " n_rec " << m_received << " n_taged " << m_received_taged <<" n_loss " << m_lossCounter.GetLost() << " id " << m_imsi << std::endl;
    return (double)m_total_latency;
}
uint64_t
UdpServerLatency::GetReceivedTaged(void){
    return m_received_taged;
}

void
UdpServerLatency::Setimsi(uint64_t imsi) 
{
    m_imsi = imsi;
}        
} // Namespace ns3
