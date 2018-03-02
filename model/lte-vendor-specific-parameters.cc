/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Marco Miozzo  <marco.miozzo@cttc.es>
 */

#include <ns3/lte-vendor-specific-parameters.h>
#include <ns3/log.h>

#include "lte-vendor-specific-parameters.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("LteVendorSpecificParameters");
  
SrsCqiRntiVsp::SrsCqiRntiVsp (uint16_t rnti)
:  m_rnti (rnti)
{
  
}

SrsCqiRntiVsp::~SrsCqiRntiVsp ()
{
  
}

uint16_t
SrsCqiRntiVsp::GetRnti ()
{
  return (m_rnti);
}

RlcPacketLengths::RlcPacketLengths(){
   
}

RlcPacketLengths::~RlcPacketLengths(){
}
uint16_t 
RlcPacketLengths::GetNumPacket (){
    return m_packetlengths.size();
}
uint32_t 
RlcPacketLengths::GetFirstPacketLength(){
    if(m_packetlengths.size()>0){
        return m_packetlengths.at(0);
    }
    else{
        return 0;
    }   
}
uint32_t 
RlcPacketLengths::GetTotalPacketLength(){
    uint32_t sumRemainingLengths = 0;
    std::vector<uint32_t>::iterator it;
    for(it = m_packetlengths.begin(); it !=m_packetlengths.end(); it++){
        sumRemainingLengths += *it;
    }
    return sumRemainingLengths;
}
uint16_t 
RlcPacketLengths::AddOnePacket(uint32_t length){
    m_packetlengths.push_back(length);
    return m_packetlengths.size();
}
uint32_t
RlcPacketLengths::RemoveEmpty(){
    uint32_t sumRemainingLengths = 0;
    while(m_packetlengths.size()>0 && m_packetlengths.at(0)<=0){
            m_packetlengths.erase(m_packetlengths.begin());
    }
    std::vector<uint32_t>::iterator it;
    for(it = m_packetlengths.begin(); it !=m_packetlengths.end(); it++){
        sumRemainingLengths += *it;
    }
    return sumRemainingLengths;
}
uint32_t
RlcPacketLengths::RemoveSomeAtFirst(uint32_t length){
    uint32_t firstRemainingLength = 0;
    while(length > 0 && m_packetlengths.size()>0 && GetTotalPacketLength() > 0){
        firstRemainingLength = m_packetlengths.at(0);
        m_packetlengths.at(0) = (m_packetlengths.at(0) >= length) ? (m_packetlengths.at(0)-length):(0);
        length -= firstRemainingLength;
        firstRemainingLength = m_packetlengths.at(0);
        RemoveEmpty();
    }
    return firstRemainingLength;
}
void 
RlcPacketLengths::CleanAll(){

    m_packetlengths.clear();
}

} // namespace ns3
