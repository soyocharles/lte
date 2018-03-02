/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   PacketMeasurement.cc
 * Author: soyo
 * 
 * Created on 21 June 2017, 12:28 PM
 */

#include <stdbool.h>

#include "PacketMeasurement.h"
using namespace ns3;
PacketMeasurement::PacketMeasurement() {
    m_name = "";
    m_received = 0;
    m_received_taged = 0;
    m_total_latency = 0;
}
std::string
PacketMeasurement::SetMeasurementName(std::string s){
    m_name = s;
    return m_name;
}

int 
PacketMeasurement::MeasurePacket(const Ptr<Packet> packet){
    if (packet->GetSize() > 0){
        m_received ++;
        TimestampTag timestamp;
        if (packet->FindFirstMatchingByteTag (timestamp)) {
            m_received_taged ++;
            Time delay = GetLatency(timestamp);
            m_total_latency =  m_total_latency + delay.GetMicroSeconds();
        }
        return 0;
    }
    return -1;
}
uint64_t 
PacketMeasurement::GetReceived(){
    return m_received;
}
uint64_t 
PacketMeasurement::GetReceivedTaged(){
    return m_received_taged;
}
uint64_t 
PacketMeasurement::GetTotalTagedLatency(){
    return m_total_latency;
}
int 
PacketMeasurement::UpdateTimeTag(const Ptr<Packet> packet){
    if (packet->GetSize() > 0){
        TimestampTag timestamp;
        timestamp.SetTimestamp (Simulator::Now ());
        packet->ReplacePacketTag (timestamp);
        return 0;
    }
    return -1;
}
Time 
PacketMeasurement::GetLatency(const TimestampTag& timestamp){
    Time tx = timestamp.GetTimestamp ();
    Time delay = (Simulator::Now () - tx);
//    std::cout << delay.GetMicroSeconds() << std::endl;
    return delay;
}
std::string 
PacketMeasurement::GetMeasurementName(){
    return m_name;
}
