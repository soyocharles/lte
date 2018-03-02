/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   timetag.h
 * Author: soyo
 *
 * Created on 14 June 2017, 3:43 PM
 */

#ifndef TIMETAG_H
#define TIMETAG_H

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/application.h"

#include "ns3/stats-module.h"

using namespace ns3;

class TimestampTag : public Tag {
public:
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;

  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (TagBuffer i) const;
  virtual void Deserialize (TagBuffer i);

  // these are our accessors to our tag structure
  void SetTimestamp (Time time);
  Time GetTimestamp (void) const;

  void Print (std::ostream &os) const;

private:
  Time m_timestamp;

  // end class TimestampTag
};

#endif /* TIMETAG_H */

