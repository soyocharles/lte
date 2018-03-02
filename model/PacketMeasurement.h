/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   PacketMeasurement.h
 * Author: soyo
 *
 * Created on 21 June 2017, 12:28 PM
 */

#ifndef PACKETMEASUREMENT_H
#define PACKETMEASUREMENT_H
#include <iostream>
#include "ns3/log.h"
#include "ns3/mac48-address.h"
#include "ns3/ipv4.h"
#include "ns3/inet-socket-address.h"
#include "ns3/epc-gtpu-header.h"
#include "ns3/abort.h"
#include "ns3/packet.h"
#include "timetag.h"
#include "seq-ts-header.h"
#include "packet-loss-counter.h"
namespace ns3 {
    
class PacketMeasurement {
public:
    PacketMeasurement();
    std::string SetMeasurementName( std::string s);
    int MeasurePacket(const Ptr<Packet> packet);
    uint64_t GetReceived();
    uint64_t GetReceivedTaged();
    uint64_t GetTotalTagedLatency();
    int UpdateTimeTag(const Ptr<Packet> packet);
    Time GetLatency(const TimestampTag& timestamp);
    std::string GetMeasurementName();
    uint64_t GetLost();
    int RenewMeasurementHeader(const Ptr<Packet> packet);

private:
    std::string m_name;
    uint64_t m_received;
    uint64_t m_received_taged;
    uint64_t m_total_latency;
};
}
#endif /* PACKETMEASUREMENT_H */

