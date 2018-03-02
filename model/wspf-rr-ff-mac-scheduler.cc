/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
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
 * Author: Marco Miozzo <marco.miozzo@cttc.es>
 */

#include <ns3/log.h>
#include <ns3/pointer.h>
#include <ns3/math.h>
#include <cfloat>
#include <set>
#include <climits>

#include <ns3/lte-amc.h>
#include <ns3/spf-rr-ff-mac-scheduler.h>
#include <ns3/simulator.h>
#include <ns3/lte-common.h>
#include <ns3/lte-vendor-specific-parameters.h>
#include <ns3/boolean.h>

#include "lte-vendor-specific-parameters.h"

namespace ns3 {

    

NS_LOG_COMPONENT_DEFINE ("SpfRrFfMacScheduler");

static const int McsToItbs[29] = {
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 10, 11, 12, 13, 14, 15, 15, 16, 17, 18,
  19, 20, 21, 22, 23, 24, 25, 26
};

/**
 * Table of number of physical resource blocks (NPRB), TBS index (ITBS), and
 * their associated transport block size. Taken from 3GPP TS 36.213 v8.8.0
 * Table 7.1.7.2.1-1: _Transport block size table (dimension 27×110)_.
 * \note For NPRB = 1 and ITBS = 6 the standard returns 328, but it is not
 *       consistent with the other values, therefore we use 88 obtained by
 *       following the sequence of NPRB = 1 values.
 */
static const int TransportBlockSizeTable [110][27] = {
  /* NPRB 001*/ { 16, 24, 32, 40, 56, 72, 88, 104, 120, 136, 144, 176, 208, 224, 256, 280, 328, 336, 376, 408, 440, 488, 520, 552, 584, 616, 712},
  /* NPRB 002*/ { 32, 56, 72, 104, 120, 144, 176, 224, 256, 296, 328, 376, 440, 488, 552, 600, 632, 696, 776, 840, 904, 1000, 1064, 1128, 1192, 1256, 1480},
  /* NPRB 003*/ { 56, 88, 144, 176, 208, 224, 256, 328, 392, 456, 504, 584, 680, 744, 840, 904, 968, 1064, 1160, 1288, 1384, 1480, 1608, 1736, 1800, 1864, 2216},
  /* NPRB 004*/ { 88, 144, 176, 208, 256, 328, 392, 472, 536, 616, 680, 776, 904, 1000, 1128, 1224, 1288, 1416, 1544, 1736, 1864, 1992, 2152, 2280, 2408, 2536, 2984},
  /* NPRB 005*/ { 120, 176, 208, 256, 328, 424, 504, 584, 680, 776, 872, 1000, 1128, 1256, 1416, 1544, 1608, 1800, 1992, 2152, 2344, 2472, 2664, 2856, 2984, 3112, 3752},
  /* NPRB 006*/ { 152, 208, 256, 328, 408, 504, 600, 712, 808, 936, 1032, 1192, 1352, 1544, 1736, 1800, 1928, 2152, 2344, 2600, 2792, 2984, 3240, 3496, 3624, 3752, 4392},
  /* NPRB 007*/ { 176, 224, 296, 392, 488, 600, 712, 840, 968, 1096, 1224, 1384, 1608, 1800, 1992, 2152, 2280, 2536, 2792, 2984, 3240, 3496, 3752, 4008, 4264, 4392, 5160},
  /* NPRB 008*/ { 208, 256, 328, 440, 552, 680, 808, 968, 1096, 1256, 1384, 1608, 1800, 2024, 2280, 2472, 2600, 2856, 3112, 3496, 3752, 4008, 4264, 4584, 4968, 5160, 5992},
  /* NPRB 009*/ { 224, 328, 376, 504, 632, 776, 936, 1096, 1256, 1416, 1544, 1800, 2024, 2280, 2600, 2728, 2984, 3240, 3624, 3880, 4136, 4584, 4776, 5160, 5544, 5736, 6712},
  /* NPRB 010*/ { 256, 344, 424, 568, 696, 872, 1032, 1224, 1384, 1544, 1736, 2024, 2280, 2536, 2856, 3112, 3240, 3624, 4008, 4264, 4584, 4968, 5352, 5736, 5992, 6200, 7480},
  /* NPRB 011*/ { 288, 376, 472, 616, 776, 968, 1128, 1320, 1544, 1736, 1928, 2216, 2472, 2856, 3112, 3368, 3624, 4008, 4392, 4776, 5160, 5544, 5992, 6200, 6712, 6968, 8248},
  /* NPRB 012*/ { 328, 424, 520, 680, 840, 1032, 1224, 1480, 1672, 1864, 2088, 2408, 2728, 3112, 3496, 3624, 3880, 4392, 4776, 5160, 5544, 5992, 6456, 6968, 7224, 7480, 8760},
  /* NPRB 013*/ { 344, 456, 568, 744, 904, 1128, 1352, 1608, 1800, 2024, 2280, 2600, 2984, 3368, 3752, 4008, 4264, 4776, 5160, 5544, 5992, 6456, 6968, 7480, 7992, 8248, 9528},
  /* NPRB 014*/ { 376, 488, 616, 808, 1000, 1224, 1480, 1672, 1928, 2216, 2472, 2792, 3240, 3624, 4008, 4264, 4584, 5160, 5544, 5992, 6456, 6968, 7480, 7992, 8504, 8760, 10296},
  /* NPRB 015*/ { 392, 520, 648, 872, 1064, 1320, 1544, 1800, 2088, 2344, 2664, 2984, 3368, 3880, 4264, 4584, 4968, 5352, 5992, 6456, 6968, 7480, 7992, 8504, 9144, 9528, 11064},
  /* NPRB 016*/ { 424, 568, 696, 904, 1128, 1384, 1672, 1928, 2216, 2536, 2792, 3240, 3624, 4136, 4584, 4968, 5160, 5736, 6200, 6968, 7480, 7992, 8504, 9144, 9912, 10296, 11832},
  /* NPRB 017*/ { 456, 600, 744, 968, 1192, 1480, 1736, 2088, 2344, 2664, 2984, 3496, 3880, 4392, 4968, 5160, 5544, 6200, 6712, 7224, 7992, 8504, 9144, 9912, 10296, 10680, 12576},
  /* NPRB 018*/ { 488, 632, 776, 1032, 1288, 1544, 1864, 2216, 2536, 2856, 3112, 3624, 4136, 4584, 5160, 5544, 5992, 6456, 7224, 7736, 8248, 9144, 9528, 10296, 11064, 11448, 13536},
  /* NPRB 019*/ { 504, 680, 840, 1096, 1352, 1672, 1992, 2344, 2664, 2984, 3368, 3880, 4392, 4968, 5544, 5736, 6200, 6712, 7480, 8248, 8760, 9528, 10296, 11064, 11448, 12216, 14112},
  /* NPRB 020*/ { 536, 712, 872, 1160, 1416, 1736, 2088, 2472, 2792, 3112, 3496, 4008, 4584, 5160, 5736, 6200, 6456, 7224, 7992, 8504, 9144, 9912, 10680, 11448, 12216, 12576, 14688},
  /* NPRB 021*/ { 568, 744, 936, 1224, 1480, 1864, 2216, 2536, 2984, 3368, 3752, 4264, 4776, 5352, 5992, 6456, 6712, 7480, 8248, 9144, 9912, 10680, 11448, 12216, 12960, 13536, 15264},
  /* NPRB 022*/ { 600, 776, 968, 1256, 1544, 1928, 2280, 2664, 3112, 3496, 3880, 4392, 4968, 5736, 6200, 6712, 7224, 7992, 8760, 9528, 10296, 11064, 11832, 12576, 13536, 14112, 16416},
  /* NPRB 023*/ { 616, 808, 1000, 1320, 1608, 2024, 2408, 2792, 3240, 3624, 4008, 4584, 5352, 5992, 6456, 6968, 7480, 8248, 9144, 9912, 10680, 11448, 12576, 12960, 14112, 14688, 16992},
  /* NPRB 024*/ { 648, 872, 1064, 1384, 1736, 2088, 2472, 2984, 3368, 3752, 4264, 4776, 5544, 6200, 6968, 7224, 7736, 8760, 9528, 10296, 11064, 12216, 12960, 13536, 14688, 15264, 17568},
  /* NPRB 025*/ { 680, 904, 1096, 1416, 1800, 2216, 2600, 3112, 3496, 4008, 4392, 4968, 5736, 6456, 7224, 7736, 7992, 9144, 9912, 10680, 11448, 12576, 13536, 14112, 15264, 15840, 18336},
  /* NPRB 026*/ { 712, 936, 1160, 1480, 1864, 2280, 2728, 3240, 3624, 4136, 4584, 5352, 5992, 6712, 7480, 7992, 8504, 9528, 10296, 11064, 12216, 12960, 14112, 14688, 15840, 16416, 19080},
  /* NPRB 027*/ { 744, 968, 1192, 1544, 1928, 2344, 2792, 3368, 3752, 4264, 4776, 5544, 6200, 6968, 7736, 8248, 8760, 9912, 10680, 11448, 12576, 13536, 14688, 15264, 16416, 16992, 19848},
  /* NPRB 028*/ { 776, 1000, 1256, 1608, 1992, 2472, 2984, 3368, 3880, 4392, 4968, 5736, 6456, 7224, 7992, 8504, 9144, 10296, 11064, 12216, 12960, 14112, 15264, 15840, 16992, 17568, 20616},
  /* NPRB 029*/ { 776, 1032, 1288, 1672, 2088, 2536, 2984, 3496, 4008, 4584, 5160, 5992, 6712, 7480, 8248, 8760, 9528, 10296, 11448, 12576, 13536, 14688, 15840, 16416, 17568, 18336, 21384},
  /* NPRB 030*/ { 808, 1064, 1320, 1736, 2152, 2664, 3112, 3624, 4264, 4776, 5352, 5992, 6712, 7736, 8504, 9144, 9912, 10680, 11832, 12960, 14112, 15264, 16416, 16992, 18336, 19080, 22152},
  /* NPRB 031*/ { 840, 1128, 1384, 1800, 2216, 2728, 3240, 3752, 4392, 4968, 5544, 6200, 6968, 7992, 8760, 9528, 9912, 11064, 12216, 13536, 14688, 15840, 16992, 17568, 19080, 19848, 22920},
  /* NPRB 032*/ { 872, 1160, 1416, 1864, 2280, 2792, 3368, 3880, 4584, 5160, 5736, 6456, 7224, 8248, 9144, 9912, 10296, 11448, 12576, 13536, 14688, 15840, 16992, 18336, 19848, 20616, 23688},
  /* NPRB 033*/ { 904, 1192, 1480, 1928, 2344, 2856, 3496, 4008, 4584, 5160, 5736, 6712, 7480, 8504, 9528, 10296, 10680, 11832, 12960, 14112, 15264, 16416, 17568, 19080, 19848, 20616, 24496},
  /* NPRB 034*/ { 936, 1224, 1544, 1992, 2408, 2984, 3496, 4136, 4776, 5352, 5992, 6968, 7736, 8760, 9912, 10296, 11064, 12216, 13536, 14688, 15840, 16992, 18336, 19848, 20616, 21384, 25456},
  /* NPRB 035*/ { 968, 1256, 1544, 2024, 2472, 3112, 3624, 4264, 4968, 5544, 6200, 6968, 7992, 9144, 9912, 10680, 11448, 12576, 14112, 15264, 16416, 17568, 19080, 19848, 21384, 22152, 25456},
  /* NPRB 036*/ { 1000, 1288, 1608, 2088, 2600, 3112, 3752, 4392, 4968, 5736, 6200, 7224, 8248, 9144, 10296, 11064, 11832, 12960, 14112, 15264, 16992, 18336, 19080, 20616, 22152, 22920, 26416},
  /* NPRB 037*/ { 1032, 1352, 1672, 2152, 2664, 3240, 3880, 4584, 5160, 5736, 6456, 7480, 8504, 9528, 10680, 11448, 12216, 13536, 14688, 15840, 16992, 18336, 19848, 21384, 22920, 23688, 27376},
  /* NPRB 038*/ { 1032, 1384, 1672, 2216, 2728, 3368, 4008, 4584, 5352, 5992, 6712, 7736, 8760, 9912, 11064, 11832, 12216, 13536, 15264, 16416, 17568, 19080, 20616, 22152, 22920, 24496, 28336},
  /* NPRB 039*/ { 1064, 1416, 1736, 2280, 2792, 3496, 4136, 4776, 5544, 6200, 6712, 7736, 8760, 9912, 11064, 11832, 12576, 14112, 15264, 16992, 18336, 19848, 21384, 22152, 23688, 24496, 29296},
  /* NPRB 040*/ { 1096, 1416, 1800, 2344, 2856, 3496, 4136, 4968, 5544, 6200, 6968, 7992, 9144, 10296, 11448, 12216, 12960, 14688, 15840, 16992, 18336, 19848, 21384, 22920, 24496, 25456, 29296},
  /* NPRB 041*/ { 1128, 1480, 1800, 2408, 2984, 3624, 4264, 4968, 5736, 6456, 7224, 8248, 9528, 10680, 11832, 12576, 13536, 14688, 16416, 17568, 19080, 20616, 22152, 23688, 25456, 26416, 30576},
  /* NPRB 042*/ { 1160, 1544, 1864, 2472, 2984, 3752, 4392, 5160, 5992, 6712, 7480, 8504, 9528, 10680, 12216, 12960, 13536, 15264, 16416, 18336, 19848, 21384, 22920, 24496, 25456, 26416, 30576},
  /* NPRB 043*/ { 1192, 1544, 1928, 2536, 3112, 3752, 4584, 5352, 5992, 6712, 7480, 8760, 9912, 11064, 12216, 12960, 14112, 15264, 16992, 18336, 19848, 21384, 22920, 24496, 26416, 27376, 31704},
  /* NPRB 044*/ { 1224, 1608, 1992, 2536, 3112, 3880, 4584, 5352, 6200, 6968, 7736, 8760, 9912, 11448, 12576, 13536, 14112, 15840, 17568, 19080, 20616, 22152, 23688, 25456, 26416, 28336, 32856},
  /* NPRB 045*/ { 1256, 1608, 2024, 2600, 3240, 4008, 4776, 5544, 6200, 6968, 7992, 9144, 10296, 11448, 12960, 13536, 14688, 16416, 17568, 19080, 20616, 22920, 24496, 25456, 27376, 28336, 32856},
  /* NPRB 046*/ { 1256, 1672, 2088, 2664, 3240, 4008, 4776, 5736, 6456, 7224, 7992, 9144, 10680, 11832, 12960, 14112, 14688, 16416, 18336, 19848, 21384, 22920, 24496, 26416, 28336, 29296, 34008},
  /* NPRB 047*/ { 1288, 1736, 2088, 2728, 3368, 4136, 4968, 5736, 6456, 7480, 8248, 9528, 10680, 12216, 13536, 14688, 15264, 16992, 18336, 20616, 22152, 23688, 25456, 27376, 28336, 29296, 35160},
  /* NPRB 048*/ { 1320, 1736, 2152, 2792, 3496, 4264, 4968, 5992, 6712, 7480, 8504, 9528, 11064, 12216, 13536, 14688, 15840, 17568, 19080, 20616, 22152, 24496, 25456, 27376, 29296, 30576, 35160},
  /* NPRB 049*/ { 1352, 1800, 2216, 2856, 3496, 4392, 5160, 5992, 6968, 7736, 8504, 9912, 11064, 12576, 14112, 15264, 15840, 17568, 19080, 21384, 22920, 24496, 26416, 28336, 29296, 31704, 36696},
  /* NPRB 050*/ { 1384, 1800, 2216, 2856, 3624, 4392, 5160, 6200, 6968, 7992, 8760, 9912, 11448, 12960, 14112, 15264, 16416, 18336, 19848, 21384, 22920, 25456, 27376, 28336, 30576, 31704, 36696},
  /* NPRB 051*/ { 1416, 1864, 2280, 2984, 3624, 4584, 5352, 6200, 7224, 7992, 9144, 10296, 11832, 12960, 14688, 15840, 16416, 18336, 19848, 22152, 23688, 25456, 27376, 29296, 31704, 32856, 37888},
  /* NPRB 052*/ { 1416, 1864, 2344, 2984, 3752, 4584, 5352, 6456, 7224, 8248, 9144, 10680, 11832, 13536, 14688, 15840, 16992, 19080, 20616, 22152, 24496, 26416, 28336, 29296, 31704, 32856, 37888},
  /* NPRB 053*/ { 1480, 1928, 2344, 3112, 3752, 4776, 5544, 6456, 7480, 8248, 9144, 10680, 12216, 13536, 15264, 16416, 16992, 19080, 21384, 22920, 24496, 26416, 28336, 30576, 32856, 34008, 39232},
  /* NPRB 054*/ { 1480, 1992, 2408, 3112, 3880, 4776, 5736, 6712, 7480, 8504, 9528, 11064, 12216, 14112, 15264, 16416, 17568, 19848, 21384, 22920, 25456, 27376, 29296, 30576, 32856, 34008, 40576},
  /* NPRB 055*/ { 1544, 1992, 2472, 3240, 4008, 4776, 5736, 6712, 7736, 8760, 9528, 11064, 12576, 14112, 15840, 16992, 17568, 19848, 22152, 23688, 25456, 27376, 29296, 31704, 34008, 35160, 40576},
  /* NPRB 056*/ { 1544, 2024, 2536, 3240, 4008, 4968, 5992, 6712, 7736, 8760, 9912, 11448, 12576, 14688, 15840, 16992, 18336, 20616, 22152, 24496, 26416, 28336, 30576, 31704, 34008, 35160, 40576},
  /* NPRB 057*/ { 1608, 2088, 2536, 3368, 4136, 4968, 5992, 6968, 7992, 9144, 9912, 11448, 12960, 14688, 16416, 17568, 18336, 20616, 22920, 24496, 26416, 28336, 30576, 32856, 35160, 36696, 42368},
  /* NPRB 058*/ { 1608, 2088, 2600, 3368, 4136, 5160, 5992, 6968, 7992, 9144, 10296, 11832, 12960, 14688, 16416, 17568, 19080, 20616, 22920, 25456, 27376, 29296, 31704, 32856, 35160, 36696, 42368},
  /* NPRB 059*/ { 1608, 2152, 2664, 3496, 4264, 5160, 6200, 7224, 8248, 9144, 10296, 11832, 13536, 15264, 16992, 18336, 19080, 21384, 23688, 25456, 27376, 29296, 31704, 34008, 36696, 37888, 43816},
  /* NPRB 060*/ { 1672, 2152, 2664, 3496, 4264, 5352, 6200, 7224, 8504, 9528, 10680, 12216, 13536, 15264, 16992, 18336, 19848, 21384, 23688, 25456, 28336, 30576, 32856, 34008, 36696, 37888, 43816},
  /* NPRB 061*/ { 1672, 2216, 2728, 3624, 4392, 5352, 6456, 7480, 8504, 9528, 10680, 12216, 14112, 15840, 17568, 18336, 19848, 22152, 24496, 26416, 28336, 30576, 32856, 35160, 36696, 39232, 45352},
  /* NPRB 062*/ { 1736, 2280, 2792, 3624, 4392, 5544, 6456, 7480, 8760, 9912, 11064, 12576, 14112, 15840, 17568, 19080, 19848, 22152, 24496, 26416, 29296, 31704, 34008, 35160, 37888, 39232, 45352},
  /* NPRB 063*/ { 1736, 2280, 2856, 3624, 4584, 5544, 6456, 7736, 8760, 9912, 11064, 12576, 14112, 16416, 18336, 19080, 20616, 22920, 24496, 27376, 29296, 31704, 34008, 36696, 37888, 40576, 46888},
  /* NPRB 064*/ { 1800, 2344, 2856, 3752, 4584, 5736, 6712, 7736, 9144, 10296, 11448, 12960, 14688, 16416, 18336, 19848, 20616, 22920, 25456, 27376, 29296, 31704, 34008, 36696, 39232, 40576, 46888},
  /* NPRB 065*/ { 1800, 2344, 2856, 3752, 4584, 5736, 6712, 7992, 9144, 10296, 11448, 12960, 14688, 16992, 18336, 19848, 21384, 23688, 25456, 28336, 30576, 32856, 35160, 37888, 39232, 40576, 48936},
  /* NPRB 066*/ { 1800, 2408, 2984, 3880, 4776, 5736, 6968, 7992, 9144, 10296, 11448, 13536, 15264, 16992, 19080, 20616, 21384, 23688, 26416, 28336, 30576, 32856, 35160, 37888, 40576, 42368, 48936},
  /* NPRB 067*/ { 1864, 2472, 2984, 3880, 4776, 5992, 6968, 8248, 9528, 10680, 11832, 13536, 15264, 16992, 19080, 20616, 22152, 24496, 26416, 29296, 31704, 34008, 36696, 37888, 40576, 42368, 48936},
  /* NPRB 068*/ { 1864, 2472, 3112, 4008, 4968, 5992, 6968, 8248, 9528, 10680, 11832, 13536, 15264, 17568, 19848, 20616, 22152, 24496, 27376, 29296, 31704, 34008, 36696, 39232, 42368, 43816, 51024},
  /* NPRB 069*/ { 1928, 2536, 3112, 4008, 4968, 5992, 7224, 8504, 9528, 11064, 12216, 14112, 15840, 17568, 19848, 21384, 22152, 24496, 27376, 29296, 31704, 35160, 36696, 39232, 42368, 43816, 51024},
  /* NPRB 070*/ { 1928, 2536, 3112, 4136, 4968, 6200, 7224, 8504, 9912, 11064, 12216, 14112, 15840, 18336, 19848, 21384, 22920, 25456, 27376, 30576, 32856, 35160, 37888, 40576, 42368, 43816, 52752},
  /* NPRB 071*/ { 1992, 2600, 3240, 4136, 5160, 6200, 7480, 8760, 9912, 11064, 12576, 14112, 16416, 18336, 20616, 22152, 22920, 25456, 28336, 30576, 32856, 35160, 37888, 40576, 43816, 45352, 52752},
  /* NPRB 072*/ { 1992, 2600, 3240, 4264, 5160, 6200, 7480, 8760, 9912, 11448, 12576, 14688, 16416, 18336, 20616, 22152, 23688, 26416, 28336, 30576, 34008, 36696, 39232, 40576, 43816, 45352, 52752},
  /* NPRB 073*/ { 2024, 2664, 3240, 4264, 5160, 6456, 7736, 8760, 10296, 11448, 12960, 14688, 16416, 19080, 20616, 22152, 23688, 26416, 29296, 31704, 34008, 36696, 39232, 42368, 45352, 46888, 55056},
  /* NPRB 074*/ { 2088, 2728, 3368, 4392, 5352, 6456, 7736, 9144, 10296, 11832, 12960, 14688, 16992, 19080, 21384, 22920, 24496, 26416, 29296, 31704, 34008, 36696, 40576, 42368, 45352, 46888, 55056},
  /* NPRB 075*/ { 2088, 2728, 3368, 4392, 5352, 6712, 7736, 9144, 10680, 11832, 12960, 15264, 16992, 19080, 21384, 22920, 24496, 27376, 29296, 32856, 35160, 37888, 40576, 43816, 45352, 46888, 55056},
  /* NPRB 076*/ { 2088, 2792, 3368, 4392, 5544, 6712, 7992, 9144, 10680, 11832, 13536, 15264, 17568, 19848, 22152, 23688, 24496, 27376, 30576, 32856, 35160, 37888, 40576, 43816, 46888, 48936, 55056},
  /* NPRB 077*/ { 2152, 2792, 3496, 4584, 5544, 6712, 7992, 9528, 10680, 12216, 13536, 15840, 17568, 19848, 22152, 23688, 25456, 27376, 30576, 32856, 35160, 39232, 42368, 43816, 46888, 48936, 57336},
  /* NPRB 078*/ { 2152, 2856, 3496, 4584, 5544, 6968, 8248, 9528, 11064, 12216, 13536, 15840, 17568, 19848, 22152, 23688, 25456, 28336, 30576, 34008, 36696, 39232, 42368, 45352, 46888, 48936, 57336},
  /* NPRB 079*/ { 2216, 2856, 3496, 4584, 5736, 6968, 8248, 9528, 11064, 12576, 14112, 15840, 18336, 20616, 22920, 24496, 25456, 28336, 31704, 34008, 36696, 39232, 42368, 45352, 48936, 51024, 57336},
  /* NPRB 080*/ { 2216, 2856, 3624, 4776, 5736, 6968, 8248, 9912, 11064, 12576, 14112, 16416, 18336, 20616, 22920, 24496, 26416, 29296, 31704, 34008, 36696, 40576, 43816, 45352, 48936, 51024, 59256},
  /* NPRB 081*/ { 2280, 2984, 3624, 4776, 5736, 7224, 8504, 9912, 11448, 12960, 14112, 16416, 18336, 20616, 22920, 24496, 26416, 29296, 31704, 35160, 37888, 40576, 43816, 46888, 48936, 51024, 59256},
  /* NPRB 082*/ { 2280, 2984, 3624, 4776, 5992, 7224, 8504, 9912, 11448, 12960, 14688, 16416, 19080, 21384, 23688, 25456, 26416, 29296, 32856, 35160, 37888, 40576, 43816, 46888, 51024, 52752, 59256},
  /* NPRB 083*/ { 2280, 2984, 3752, 4776, 5992, 7224, 8760, 10296, 11448, 12960, 14688, 16992, 19080, 21384, 23688, 25456, 27376, 30576, 32856, 35160, 39232, 42368, 45352, 46888, 51024, 52752, 61664},
  /* NPRB 084*/ { 2344, 3112, 3752, 4968, 5992, 7480, 8760, 10296, 11832, 13536, 14688, 16992, 19080, 21384, 24496, 25456, 27376, 30576, 32856, 36696, 39232, 42368, 45352, 48936, 51024, 52752, 61664},
  /* NPRB 085*/ { 2344, 3112, 3880, 4968, 5992, 7480, 8760, 10296, 11832, 13536, 14688, 16992, 19080, 22152, 24496, 26416, 27376, 30576, 34008, 36696, 39232, 42368, 45352, 48936, 52752, 55056, 61664},
  /* NPRB 086*/ { 2408, 3112, 3880, 4968, 6200, 7480, 9144, 10680, 12216, 13536, 15264, 17568, 19848, 22152, 24496, 26416, 28336, 30576, 34008, 36696, 40576, 43816, 46888, 48936, 52752, 55056, 63776},
  /* NPRB 087*/ { 2408, 3240, 3880, 5160, 6200, 7736, 9144, 10680, 12216, 13536, 15264, 17568, 19848, 22152, 25456, 26416, 28336, 31704, 34008, 37888, 40576, 43816, 46888, 51024, 52752, 55056, 63776},
  /* NPRB 088*/ { 2472, 3240, 4008, 5160, 6200, 7736, 9144, 10680, 12216, 14112, 15264, 17568, 19848, 22920, 25456, 27376, 28336, 31704, 35160, 37888, 40576, 43816, 46888, 51024, 52752, 55056, 63776},
  /* NPRB 089*/ { 2472, 3240, 4008, 5160, 6456, 7736, 9144, 11064, 12576, 14112, 15840, 18336, 20616, 22920, 25456, 27376, 29296, 31704, 35160, 37888, 42368, 45352, 48936, 51024, 55056, 57336, 66592},
  /* NPRB 090*/ { 2536, 3240, 4008, 5352, 6456, 7992, 9528, 11064, 12576, 14112, 15840, 18336, 20616, 22920, 25456, 27376, 29296, 32856, 35160, 39232, 42368, 45352, 48936, 51024, 55056, 57336, 66592},
  /* NPRB 091*/ { 2536, 3368, 4136, 5352, 6456, 7992, 9528, 11064, 12576, 14112, 15840, 18336, 20616, 23688, 26416, 28336, 29296, 32856, 36696, 39232, 42368, 45352, 48936, 52752, 55056, 57336, 66592},
  /* NPRB 092*/ { 2536, 3368, 4136, 5352, 6456, 7992, 9528, 11448, 12960, 14688, 16416, 18336, 21384, 23688, 26416, 28336, 30576, 32856, 36696, 39232, 42368, 46888, 48936, 52752, 57336, 59256, 68808},
  /* NPRB 093*/ { 2600, 3368, 4136, 5352, 6712, 8248, 9528, 11448, 12960, 14688, 16416, 19080, 21384, 23688, 26416, 28336, 30576, 34008, 36696, 40576, 43816, 46888, 51024, 52752, 57336, 59256, 68808},
  /* NPRB 094*/ { 2600, 3496, 4264, 5544, 6712, 8248, 9912, 11448, 12960, 14688, 16416, 19080, 21384, 24496, 27376, 29296, 30576, 34008, 37888, 40576, 43816, 46888, 51024, 55056, 57336, 59256, 68808},
  /* NPRB 095*/ { 2664, 3496, 4264, 5544, 6712, 8248, 9912, 11448, 13536, 15264, 16992, 19080, 21384, 24496, 27376, 29296, 30576, 34008, 37888, 40576, 43816, 46888, 51024, 55056, 57336, 61664, 71112},
  /* NPRB 096*/ { 2664, 3496, 4264, 5544, 6968, 8504, 9912, 11832, 13536, 15264, 16992, 19080, 22152, 24496, 27376, 29296, 31704, 35160, 37888, 40576, 45352, 48936, 51024, 55056, 59256, 61664, 71112},
  /* NPRB 097*/ { 2728, 3496, 4392, 5736, 6968, 8504, 10296, 11832, 13536, 15264, 16992, 19848, 22152, 25456, 28336, 29296, 31704, 35160, 37888, 42368, 45352, 48936, 52752, 55056, 59256, 61664, 71112},
  /* NPRB 098*/ { 2728, 3624, 4392, 5736, 6968, 8760, 10296, 11832, 13536, 15264, 16992, 19848, 22152, 25456, 28336, 30576, 31704, 35160, 39232, 42368, 45352, 48936, 52752, 57336, 59256, 61664, 73712},
  /* NPRB 099*/ { 2728, 3624, 4392, 5736, 6968, 8760, 10296, 12216, 14112, 15840, 17568, 19848, 22920, 25456, 28336, 30576, 31704, 35160, 39232, 42368, 46888, 48936, 52752, 57336, 61664, 63776, 73712},
  /* NPRB 100*/ { 2792, 3624, 4584, 5736, 7224, 8760, 10296, 12216, 14112, 15840, 17568, 19848, 22920, 25456, 28336, 30576, 32856, 36696, 39232, 43816, 46888, 51024, 55056, 57336, 61664, 63776, 75376},
  /* NPRB 101*/ { 2792, 3752, 4584, 5992, 7224, 8760, 10680, 12216, 14112, 15840, 17568, 20616, 22920, 26416, 29296, 30576, 32856, 36696, 40576, 43816, 46888, 51024, 55056, 57336, 61664, 63776, 75376},
  /* NPRB 102*/ { 2856, 3752, 4584, 5992, 7224, 9144, 10680, 12576, 14112, 16416, 18336, 20616, 23688, 26416, 29296, 31704, 32856, 36696, 40576, 43816, 46888, 51024, 55056, 59256, 61664, 63776, 75376},
  /* NPRB 103*/ { 2856, 3752, 4584, 5992, 7480, 9144, 10680, 12576, 14688, 16416, 18336, 20616, 23688, 26416, 29296, 31704, 34008, 36696, 40576, 43816, 48936, 51024, 55056, 59256, 63776, 66592, 75376},
  /* NPRB 104*/ { 2856, 3752, 4584, 5992, 7480, 9144, 10680, 12576, 14688, 16416, 18336, 21384, 23688, 26416, 29296, 31704, 34008, 37888, 40576, 45352, 48936, 52752, 57336, 59256, 63776, 66592, 75376},
  /* NPRB 105*/ { 2984, 3880, 4776, 6200, 7480, 9144, 11064, 12960, 14688, 16416, 18336, 21384, 23688, 27376, 30576, 31704, 34008, 37888, 42368, 45352, 48936, 52752, 57336, 59256, 63776, 66592, 75376},
  /* NPRB 106*/ { 2984, 3880, 4776, 6200, 7480, 9528, 11064, 12960, 14688, 16992, 18336, 21384, 24496, 27376, 30576, 32856, 34008, 37888, 42368, 45352, 48936, 52752, 57336, 61664, 63776, 66592, 75376},
  /* NPRB 107*/ { 2984, 3880, 4776, 6200, 7736, 9528, 11064, 12960, 15264, 16992, 19080, 21384, 24496, 27376, 30576, 32856, 35160, 39232, 42368, 46888, 48936, 52752, 57336, 61664, 66592, 68808, 75376},
  /* NPRB 108*/ { 2984, 4008, 4776, 6200, 7736, 9528, 11448, 12960, 15264, 16992, 19080, 22152, 24496, 27376, 30576, 32856, 35160, 39232, 42368, 46888, 51024, 55056, 59256, 61664, 66592, 68808, 75376},
  /* NPRB 109*/ { 2984, 4008, 4968, 6456, 7736, 9528, 11448, 13536, 15264, 16992, 19080, 22152, 24496, 28336, 31704, 34008, 35160, 39232, 43816, 46888, 51024, 55056, 59256, 61664, 66592, 68808, 75376},
  /* NPRB 110*/ { 3112, 4008, 4968, 6456, 7992, 9528, 11448, 13536, 15264, 17568, 19080, 22152, 25456, 28336, 31704, 34008, 35160, 39232, 43816, 46888, 51024, 55056, 59256, 63776, 66592, 71112, 75376}

};  

static const double SpectralEfficiencyForCqi[16] = {
  0.0, // out of range
  0.15, 0.23, 0.38, 0.6, 0.88, 1.18,
  1.48, 1.91, 2.41,
  2.73, 3.32, 3.9, 4.52, 5.12, 5.55
};
/**
 * Table of MCS index and its spectral efficiency. Taken from 3GPP TSG-RAN WG1
 * [R1-081483 Conveying MCS and TB size via PDCCH]
 * (http://www.3gpp.org/ftp/tsg_ran/WG1_RL1/TSGR1_52b/Docs/R1-081483.zip)
 * file `TBS_support.xls` tab "MCS Table" (rounded to 2 decimal digits).
 * The index of the vector (range 0-31) corresponds to the MCS index according
 * to the convention in TS 36.213 (i.e., the MCS index reported in R1-081483
 * minus one)
 */
static const double SpectralEfficiencyForMcs[32] = {
  0.15, 0.19, 0.23, 0.31, 0.38, 0.49, 0.6, 0.74, 0.88, 1.03, 1.18,
  1.33, 1.48, 1.7, 1.91, 2.16, 2.41, 2.57,
  2.73, 3.03, 3.32, 3.61, 3.9, 4.21, 4.52, 4.82, 5.12, 5.33, 5.55,
  0, 0, 0
};

static int
StaticGetMcsFromCqi (int cqi)
{
//  NS_LOG_FUNCTION (cqi);
//  NS_ASSERT_MSG (cqi >= 0 && cqi <= 15, "CQI must be in [0..15] = " << cqi);
  double spectralEfficiency = SpectralEfficiencyForCqi[cqi];
  int mcs = 0;
  while ((mcs < 28) && (SpectralEfficiencyForMcs[mcs + 1] <= spectralEfficiency))
    {
      ++mcs;
    }
//  NS_LOG_LOGIC ("mcs = " << mcs);
  return mcs;
}

static int
StaticGetNPRBSizeFromTbSize (int mcs, int tbsize)
{
//  NS_LOG_FUNCTION (mcs);
//  NS_ASSERT_MSG (mcs < 29, "MCS=" << mcs);
  int itbs = McsToItbs[mcs];
  if(tbsize <= 0){
      return 0;
  }
  int i = 0;
  for(; i < 110; i++){

      if(TransportBlockSizeTable[i][itbs] >= tbsize){
          break;
      }
  }
  return i+1;
}


NS_OBJECT_ENSURE_REGISTERED (SpfRrFfMacScheduler);


class SpfRrSchedulerMemberCschedSapProvider : public FfMacCschedSapProvider
{
public:
  SpfRrSchedulerMemberCschedSapProvider (SpfRrFfMacScheduler* scheduler);

  // inherited from FfMacCschedSapProvider
  virtual void CschedCellConfigReq (const struct CschedCellConfigReqParameters& params);
  virtual void CschedUeConfigReq (const struct CschedUeConfigReqParameters& params);
  virtual void CschedLcConfigReq (const struct CschedLcConfigReqParameters& params);
  virtual void CschedLcReleaseReq (const struct CschedLcReleaseReqParameters& params);
  virtual void CschedUeReleaseReq (const struct CschedUeReleaseReqParameters& params);

private:
  SpfRrSchedulerMemberCschedSapProvider ();
  SpfRrFfMacScheduler* m_scheduler;
};

SpfRrSchedulerMemberCschedSapProvider::SpfRrSchedulerMemberCschedSapProvider ()
{
}

SpfRrSchedulerMemberCschedSapProvider::SpfRrSchedulerMemberCschedSapProvider (SpfRrFfMacScheduler* scheduler) : m_scheduler (scheduler)
{
}


void
SpfRrSchedulerMemberCschedSapProvider::CschedCellConfigReq (const struct CschedCellConfigReqParameters& params)
{
  m_scheduler->DoCschedCellConfigReq (params);
}

void
SpfRrSchedulerMemberCschedSapProvider::CschedUeConfigReq (const struct CschedUeConfigReqParameters& params)
{
  m_scheduler->DoCschedUeConfigReq (params);
}


void
SpfRrSchedulerMemberCschedSapProvider::CschedLcConfigReq (const struct CschedLcConfigReqParameters& params)
{
  m_scheduler->DoCschedLcConfigReq (params);
}

void
SpfRrSchedulerMemberCschedSapProvider::CschedLcReleaseReq (const struct CschedLcReleaseReqParameters& params)
{
  m_scheduler->DoCschedLcReleaseReq (params);
}

void
SpfRrSchedulerMemberCschedSapProvider::CschedUeReleaseReq (const struct CschedUeReleaseReqParameters& params)
{
  m_scheduler->DoCschedUeReleaseReq (params);
}




class SpfRrSchedulerMemberSchedSapProvider : public FfMacSchedSapProvider
{
public:
  SpfRrSchedulerMemberSchedSapProvider (SpfRrFfMacScheduler* scheduler);

  // inherited from FfMacSchedSapProvider
  virtual void SchedDlRlcBufferReq (const struct SchedDlRlcBufferReqParameters& params);
  virtual void SchedDlPagingBufferReq (const struct SchedDlPagingBufferReqParameters& params);
  virtual void SchedDlMacBufferReq (const struct SchedDlMacBufferReqParameters& params);
  virtual void SchedDlTriggerReq (const struct SchedDlTriggerReqParameters& params);
  virtual void SchedDlRachInfoReq (const struct SchedDlRachInfoReqParameters& params);
  virtual void SchedDlCqiInfoReq (const struct SchedDlCqiInfoReqParameters& params);
  virtual void SchedUlTriggerReq (const struct SchedUlTriggerReqParameters& params);
  virtual void SchedUlNoiseInterferenceReq (const struct SchedUlNoiseInterferenceReqParameters& params);
  virtual void SchedUlSrInfoReq (const struct SchedUlSrInfoReqParameters& params);
  virtual void SchedUlMacCtrlInfoReq (const struct SchedUlMacCtrlInfoReqParameters& params);
  virtual void SchedUlCqiInfoReq (const struct SchedUlCqiInfoReqParameters& params);


private:
  SpfRrSchedulerMemberSchedSapProvider ();
  SpfRrFfMacScheduler* m_scheduler;
};



SpfRrSchedulerMemberSchedSapProvider::SpfRrSchedulerMemberSchedSapProvider ()
{
}


SpfRrSchedulerMemberSchedSapProvider::SpfRrSchedulerMemberSchedSapProvider (SpfRrFfMacScheduler* scheduler)
  : m_scheduler (scheduler)
{
}

void
SpfRrSchedulerMemberSchedSapProvider::SchedDlRlcBufferReq (const struct SchedDlRlcBufferReqParameters& params)
{
  m_scheduler->DoSchedDlRlcBufferReq (params);
}

void
SpfRrSchedulerMemberSchedSapProvider::SchedDlPagingBufferReq (const struct SchedDlPagingBufferReqParameters& params)
{
  m_scheduler->DoSchedDlPagingBufferReq (params);
}

void
SpfRrSchedulerMemberSchedSapProvider::SchedDlMacBufferReq (const struct SchedDlMacBufferReqParameters& params)
{
  m_scheduler->DoSchedDlMacBufferReq (params);
}

void
SpfRrSchedulerMemberSchedSapProvider::SchedDlTriggerReq (const struct SchedDlTriggerReqParameters& params)
{
  m_scheduler->DoSchedDlTriggerReq (params);
}

void
SpfRrSchedulerMemberSchedSapProvider::SchedDlRachInfoReq (const struct SchedDlRachInfoReqParameters& params)
{
  m_scheduler->DoSchedDlRachInfoReq (params);
}

void
SpfRrSchedulerMemberSchedSapProvider::SchedDlCqiInfoReq (const struct SchedDlCqiInfoReqParameters& params)
{
  m_scheduler->DoSchedDlCqiInfoReq (params);
}

void
SpfRrSchedulerMemberSchedSapProvider::SchedUlTriggerReq (const struct SchedUlTriggerReqParameters& params)
{
  m_scheduler->DoSchedUlTriggerReq (params);
}

void
SpfRrSchedulerMemberSchedSapProvider::SchedUlNoiseInterferenceReq (const struct SchedUlNoiseInterferenceReqParameters& params)
{
  m_scheduler->DoSchedUlNoiseInterferenceReq (params);
}

void
SpfRrSchedulerMemberSchedSapProvider::SchedUlSrInfoReq (const struct SchedUlSrInfoReqParameters& params)
{
  m_scheduler->DoSchedUlSrInfoReq (params);
}

void
SpfRrSchedulerMemberSchedSapProvider::SchedUlMacCtrlInfoReq (const struct SchedUlMacCtrlInfoReqParameters& params)
{
  m_scheduler->DoSchedUlMacCtrlInfoReq (params);
}

void
SpfRrSchedulerMemberSchedSapProvider::SchedUlCqiInfoReq (const struct SchedUlCqiInfoReqParameters& params)
{
  m_scheduler->DoSchedUlCqiInfoReq (params);
}





SpfRrFfMacScheduler::SpfRrFfMacScheduler ()
  :   m_cschedSapUser (0),
    m_schedSapUser (0),
    m_nextRntiDl (0),
    m_nextRntiUl (0)
{
  m_amc = CreateObject <LteAmc> ();
  m_cschedSapProvider = new SpfRrSchedulerMemberCschedSapProvider (this);
  m_schedSapProvider = new SpfRrSchedulerMemberSchedSapProvider (this);
}

SpfRrFfMacScheduler::~SpfRrFfMacScheduler ()
{
  NS_LOG_FUNCTION (this);
}

void
SpfRrFfMacScheduler::DoDispose ()
{
  NS_LOG_FUNCTION (this);
  m_dlHarqProcessesDciBuffer.clear ();
  m_dlHarqProcessesTimer.clear ();
  m_dlHarqProcessesRlcPduListBuffer.clear ();
  m_dlInfoListBuffered.clear ();
  m_ulHarqCurrentProcessId.clear ();
  m_ulHarqProcessesStatus.clear ();
  m_ulHarqProcessesDciBuffer.clear ();
  delete m_cschedSapProvider;
  delete m_schedSapProvider;
}

TypeId
SpfRrFfMacScheduler::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SpfRrFfMacScheduler")
    .SetParent<FfMacScheduler> ()
    .SetGroupName("Lte")
    .AddConstructor<SpfRrFfMacScheduler> ()
    .AddAttribute ("CqiTimerThreshold",
                   "The number of TTIs a CQI is valid (default 1000 - 1 sec.)",
                   UintegerValue (1000),
                   MakeUintegerAccessor (&SpfRrFfMacScheduler::m_cqiTimersThreshold),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("HarqEnabled",
                   "Activate/Deactivate the HARQ [by default is active].",
                   BooleanValue (true),
                   MakeBooleanAccessor (&SpfRrFfMacScheduler::m_harqOn),
                   MakeBooleanChecker ())
    .AddAttribute ("UlGrantMcs",
                   "The MCS of the UL grant, must be [0..15] (default 0)",
                   UintegerValue (0),
                   MakeUintegerAccessor (&SpfRrFfMacScheduler::m_ulGrantMcs),
                   MakeUintegerChecker<uint8_t> ())
  ;
  return tid;
}



void
SpfRrFfMacScheduler::SetFfMacCschedSapUser (FfMacCschedSapUser* s)
{
  m_cschedSapUser = s;
}

void
SpfRrFfMacScheduler::SetFfMacSchedSapUser (FfMacSchedSapUser* s)
{
  m_schedSapUser = s;
}

FfMacCschedSapProvider*
SpfRrFfMacScheduler::GetFfMacCschedSapProvider ()
{
  return m_cschedSapProvider;
}

FfMacSchedSapProvider*
SpfRrFfMacScheduler::GetFfMacSchedSapProvider ()
{
  return m_schedSapProvider;
}

void
SpfRrFfMacScheduler::SetLteFfrSapProvider (LteFfrSapProvider* s)
{
  m_ffrSapProvider = s;
}

LteFfrSapUser*
SpfRrFfMacScheduler::GetLteFfrSapUser ()
{
  return m_ffrSapUser;
}

void
SpfRrFfMacScheduler::DoCschedCellConfigReq (const struct FfMacCschedSapProvider::CschedCellConfigReqParameters& params)
{
  NS_LOG_FUNCTION (this);
  // Read the subset of parameters used
  m_cschedCellConfig = params;
  m_rachAllocationMap.resize (m_cschedCellConfig.m_ulBandwidth, 0);
  FfMacCschedSapUser::CschedUeConfigCnfParameters cnf;
  cnf.m_result = SUCCESS;
  m_cschedSapUser->CschedUeConfigCnf (cnf);
  return;
}

void
SpfRrFfMacScheduler::DoCschedUeConfigReq (const struct FfMacCschedSapProvider::CschedUeConfigReqParameters& params)
{
  NS_LOG_FUNCTION (this << " RNTI " << params.m_rnti << " txMode " << (uint16_t)params.m_transmissionMode);
  std::map <uint16_t,uint8_t>::iterator it = m_uesTxMode.find (params.m_rnti);
  if (it == m_uesTxMode.end ())
    {
      m_uesTxMode.insert (std::pair <uint16_t, double> (params.m_rnti, params.m_transmissionMode));
      // generate HARQ buffers
      m_dlHarqCurrentProcessId.insert (std::pair <uint16_t,uint8_t > (params.m_rnti, 0));
      DlHarqProcessesStatus_t dlHarqPrcStatus;
      dlHarqPrcStatus.resize (8,0);
      m_dlHarqProcessesStatus.insert (std::pair <uint16_t, DlHarqProcessesStatus_t> (params.m_rnti, dlHarqPrcStatus));
      DlHarqProcessesTimer_t dlHarqProcessesTimer;
      dlHarqProcessesTimer.resize (8,0);
      m_dlHarqProcessesTimer.insert (std::pair <uint16_t, DlHarqProcessesTimer_t> (params.m_rnti, dlHarqProcessesTimer));
      DlHarqProcessesDciBuffer_t dlHarqdci;
      dlHarqdci.resize (8);
      m_dlHarqProcessesDciBuffer.insert (std::pair <uint16_t, DlHarqProcessesDciBuffer_t> (params.m_rnti, dlHarqdci));
      DlHarqRlcPduListBuffer_t dlHarqRlcPdu;
      dlHarqRlcPdu.resize (2);
      dlHarqRlcPdu.at (0).resize (8);
      dlHarqRlcPdu.at (1).resize (8);
      m_dlHarqProcessesRlcPduListBuffer.insert (std::pair <uint16_t, DlHarqRlcPduListBuffer_t> (params.m_rnti, dlHarqRlcPdu));
      m_ulHarqCurrentProcessId.insert (std::pair <uint16_t,uint8_t > (params.m_rnti, 0));
      UlHarqProcessesStatus_t ulHarqPrcStatus;
      ulHarqPrcStatus.resize (8,0);
      m_ulHarqProcessesStatus.insert (std::pair <uint16_t, UlHarqProcessesStatus_t> (params.m_rnti, ulHarqPrcStatus));
      UlHarqProcessesDciBuffer_t ulHarqdci;
      ulHarqdci.resize (8);
      m_ulHarqProcessesDciBuffer.insert (std::pair <uint16_t, UlHarqProcessesDciBuffer_t> (params.m_rnti, ulHarqdci));
    }
  else
    {
      (*it).second = params.m_transmissionMode;
    }
  return;
}

void
SpfRrFfMacScheduler::DoCschedLcConfigReq (const struct FfMacCschedSapProvider::CschedLcConfigReqParameters& params)
{
  NS_LOG_FUNCTION (this);
  // Not used at this stage (LCs updated by DoSchedDlRlcBufferReq)
  return;
}

void
SpfRrFfMacScheduler::DoCschedLcReleaseReq (const struct FfMacCschedSapProvider::CschedLcReleaseReqParameters& params)
{
  NS_LOG_FUNCTION (this);
    for (uint16_t i = 0; i < params.m_logicalChannelIdentity.size (); i++)
    {
     std::list<FfMacSchedSapProvider::SchedDlRlcBufferReqParameters>::iterator it = m_rlcBufferReq.begin ();
      while (it!=m_rlcBufferReq.end ())
        {
          if (((*it).m_rnti == params.m_rnti)&&((*it).m_logicalChannelIdentity == params.m_logicalChannelIdentity.at (i)))
            {
              it = m_rlcBufferReq.erase (it);
            }
          else
            {
              it++;
            }
        }
    }
  return;
}

void
SpfRrFfMacScheduler::DoCschedUeReleaseReq (const struct FfMacCschedSapProvider::CschedUeReleaseReqParameters& params)
{
  NS_LOG_FUNCTION (this << " Release RNTI " << params.m_rnti);
  
  m_uesTxMode.erase (params.m_rnti);
  m_dlHarqCurrentProcessId.erase (params.m_rnti);
  m_dlHarqProcessesStatus.erase  (params.m_rnti);
  m_dlHarqProcessesTimer.erase (params.m_rnti);
  m_dlHarqProcessesDciBuffer.erase  (params.m_rnti);
  m_dlHarqProcessesRlcPduListBuffer.erase  (params.m_rnti);
  m_ulHarqCurrentProcessId.erase  (params.m_rnti);
  m_ulHarqProcessesStatus.erase  (params.m_rnti);
  m_ulHarqProcessesDciBuffer.erase  (params.m_rnti);
  m_ceBsrRxed.erase (params.m_rnti);
  std::list<FfMacSchedSapProvider::SchedDlRlcBufferReqParameters>::iterator it = m_rlcBufferReq.begin ();
  while (it != m_rlcBufferReq.end ())
    {
      if ((*it).m_rnti == params.m_rnti)
        {
          NS_LOG_INFO (this << " Erase RNTI " << (*it).m_rnti << " LC " << (uint16_t)(*it).m_logicalChannelIdentity);
          it = m_rlcBufferReq.erase (it);
        }
      else
        {
          it++;
        }
    }
  if (m_nextRntiUl == params.m_rnti)
    {
      m_nextRntiUl = 0;
    }

  if (m_nextRntiDl == params.m_rnti)
    {
      m_nextRntiDl = 0;
    }
    
  return;
}

/*
 *This function update buffer status of logical channel data in RLC.
 *erase *it of m_rlcBufferReq
 *and add the new bufferReq at the end
 */
void
SpfRrFfMacScheduler::DoSchedDlRlcBufferReq (const struct FfMacSchedSapProvider::SchedDlRlcBufferReqParameters& params)
{
  NS_LOG_FUNCTION (this << params.m_rnti << (uint32_t) params.m_logicalChannelIdentity);
    NS_LOG_FUNCTION (this << "+++++SOYO+++++SchedDlRlcBufferReqParameters+++++++++" << params.m_vendorSpecificList.size());

  // API generated by RLC for updating RLC parameters on a LC (tx and retx queues)
  std::list<FfMacSchedSapProvider::SchedDlRlcBufferReqParameters>::iterator it = m_rlcBufferReq.begin ();
  bool newLc = true;
  while (it != m_rlcBufferReq.end ())
    {
      // remove old entries of this UE-LC
      if (((*it).m_rnti == params.m_rnti)&&((*it).m_logicalChannelIdentity == params.m_logicalChannelIdentity))
        {
          it = m_rlcBufferReq.erase (it);
          newLc = false;
        }
      else
        {
          ++it;
        }
    }
  // add the new parameters
  m_rlcBufferReq.insert (it, params);
  NS_LOG_INFO (this << " RNTI " << params.m_rnti << " LC " << (uint16_t)params.m_logicalChannelIdentity << " RLC tx size " << params.m_rlcTransmissionQueueHolDelay << " RLC retx size " << params.m_rlcRetransmissionQueueSize << " RLC stat size " <<  params.m_rlcStatusPduSize);
  // initialize statistics of the flow in case of new flows
  if (newLc == true)
    {
      m_p10CqiRxed.insert ( std::pair<uint16_t, uint8_t > (params.m_rnti, 1)); // only codeword 0 at this stage (SISO)
      // initialized to 1 (i.e., the lowest value for transmitting a signal)
      m_p10CqiTimers.insert ( std::pair<uint16_t, uint32_t > (params.m_rnti, m_cqiTimersThreshold));
    }
    NS_LOG_FUNCTION (this << "+++++SOYO+++++SchedDlRlcBufferReqParameters+++++return" << params.m_vendorSpecificList.size());

  return;
}

void
SpfRrFfMacScheduler::DoSchedDlPagingBufferReq (const struct FfMacSchedSapProvider::SchedDlPagingBufferReqParameters& params)
{
  NS_LOG_FUNCTION (this);
  NS_FATAL_ERROR ("method not implemented");
  return;
}

void
SpfRrFfMacScheduler::DoSchedDlMacBufferReq (const struct FfMacSchedSapProvider::SchedDlMacBufferReqParameters& params)
{
  NS_LOG_FUNCTION (this);
  NS_FATAL_ERROR ("method not implemented");
  return;
}

int
SpfRrFfMacScheduler::GetRbgSize (int dlbandwidth)
{
  for (int i = 0; i < 4; i++)
    {
      if (dlbandwidth < Type0AllocationRbg[i])
        {
          return (i + 1);
        }
    }

  return (-1);
}

bool
SpfRrFfMacScheduler::SortRlcBufferReq (FfMacSchedSapProvider::SchedDlRlcBufferReqParameters i,FfMacSchedSapProvider::SchedDlRlcBufferReqParameters j)
{
    return (i.m_rnti < j.m_rnti);
}
bool 
SpfRrFfMacScheduler::SortRlcBufferReqCqiPair (std::pair<FfMacSchedSapProvider::SchedDlRlcBufferReqParameters,uint8_t> i,std::pair<FfMacSchedSapProvider::SchedDlRlcBufferReqParameters,uint8_t> j)
{
    std::vector <struct VendorSpecificListElement_s>  list_i = i.first.m_vendorSpecificList;
    std::vector <struct VendorSpecificListElement_s>  list_j = j.first.m_vendorSpecificList;
    if( list_i.size()>0 && list_j.size()>0 ){
        Ptr<RlcPacketLengths> p_i = DynamicCast<RlcPacketLengths> (list_i.at(0).m_value);
        Ptr<RlcPacketLengths> p_j = DynamicCast<RlcPacketLengths> (list_j.at(0).m_value);
        int size_i = StaticGetNPRBSizeFromTbSize(StaticGetMcsFromCqi(i.second),p_i->GetFirstPacketLength()*8);
        int size_j = StaticGetNPRBSizeFromTbSize(StaticGetMcsFromCqi(j.second),p_j->GetFirstPacketLength()*8);
        if(size_i == size_j){
            return p_i->GetFirstPacketLength() < p_j->GetFirstPacketLength();
        }
        else{
            return size_i < size_j;
        }
    }
    return true;
}
uint32_t 
SpfRrFfMacScheduler::updatePacketBuffer (const std::list <std::pair<FfMacSchedSapProvider::SchedDlRlcBufferReqParameters,uint8_t>>& canBeTransferred)
{
    uint32_t sum = 0;
    std::list <std::pair<FfMacSchedSapProvider::SchedDlRlcBufferReqParameters,uint8_t>>::const_iterator it =  canBeTransferred.begin();
    for(; it != canBeTransferred.end(); it++){
        uint32_t temp = 0;
        if((*it).first.m_vendorSpecificList.size()>0){    
            Ptr<RlcPacketLengths> p = DynamicCast<RlcPacketLengths>( (*it).first.m_vendorSpecificList.at(0).m_value);
            temp = p->GetTotalPacketLength();
        }
        sum += temp;
    }
    return sum;
}
bool
SortTrans(std::tuple<uint16_t,uint8_t,int,int,uint16_t> i, std::tuple<uint16_t,uint8_t,int,int,uint16_t> j){
    uint16_t rnit_i = std::get<0>(i);
    uint16_t rnit_j = std::get<0>(j);
    uint8_t lci_i = std::get<1>(i);    
    uint8_t lci_j = std::get<1>(j);
    
    if(rnit_i < rnit_j) {
        return true;
    }
    else if(rnit_i > rnit_j) {
        return false;
    }
    else if(rnit_i == rnit_j) {
        return lci_i < lci_j;
    }
    return true;

}



uint8_t
SpfRrFfMacScheduler::HarqProcessAvailability (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << rnti);

  std::map <uint16_t, uint8_t>::iterator it = m_dlHarqCurrentProcessId.find (rnti);
  if (it == m_dlHarqCurrentProcessId.end ())
    {
      NS_FATAL_ERROR ("No Process Id found for this RNTI " << rnti);
    }
  std::map <uint16_t, DlHarqProcessesStatus_t>::iterator itStat = m_dlHarqProcessesStatus.find (rnti);
  if (itStat == m_dlHarqProcessesStatus.end ())
    {
      NS_FATAL_ERROR ("No Process Id Statusfound for this RNTI " << rnti);
    }
  uint8_t i = (*it).second;
  do
    {
      i = (i + 1) % HARQ_PROC_NUM;
    }
  while ( ((*itStat).second.at (i) != 0)&&(i != (*it).second));
  if ((*itStat).second.at (i) == 0)
    {
      return (true);
    }
  else
    {
      return (false); // return a not valid harq proc id
    }
}



uint8_t
SpfRrFfMacScheduler::UpdateHarqProcessId (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << rnti);


  if (m_harqOn == false)
    {
      return (0);
    }

  std::map <uint16_t, uint8_t>::iterator it = m_dlHarqCurrentProcessId.find (rnti);
  if (it == m_dlHarqCurrentProcessId.end ())
    {
      NS_FATAL_ERROR ("No Process Id found for this RNTI " << rnti);
    }
  std::map <uint16_t, DlHarqProcessesStatus_t>::iterator itStat = m_dlHarqProcessesStatus.find (rnti);
  if (itStat == m_dlHarqProcessesStatus.end ())
    {
      NS_FATAL_ERROR ("No Process Id Statusfound for this RNTI " << rnti);
    }
  uint8_t i = (*it).second;
  do
    {
      i = (i + 1) % HARQ_PROC_NUM;
    }
  while ( ((*itStat).second.at (i) != 0)&&(i != (*it).second));
  if ((*itStat).second.at (i) == 0)
    {
      (*it).second = i;
      (*itStat).second.at (i) = 1;
    }
  else
    {
      return (9); // return a not valid harq proc id
    }

  return ((*it).second);
}


void
SpfRrFfMacScheduler::RefreshHarqProcesses ()
{
  NS_LOG_FUNCTION (this);

  std::map <uint16_t, DlHarqProcessesTimer_t>::iterator itTimers;
  for (itTimers = m_dlHarqProcessesTimer.begin (); itTimers != m_dlHarqProcessesTimer.end (); itTimers ++)
    {
      for (uint16_t i = 0; i < HARQ_PROC_NUM; i++)
        {
          if ((*itTimers).second.at (i) == HARQ_DL_TIMEOUT)
            {
              // reset HARQ process

              NS_LOG_INFO (this << " Reset HARQ proc " << i << " for RNTI " << (*itTimers).first);
              std::map <uint16_t, DlHarqProcessesStatus_t>::iterator itStat = m_dlHarqProcessesStatus.find ((*itTimers).first);
              if (itStat == m_dlHarqProcessesStatus.end ())
                {
                  NS_FATAL_ERROR ("No Process Id Status found for this RNTI " << (*itTimers).first);
                }
              (*itStat).second.at (i) = 0;
              (*itTimers).second.at (i) = 0;
            }
          else
            {
              (*itTimers).second.at (i)++;
            }
        }
    }

}

/*Start the DLMAC scheduler for this subframe
 * 
 */
void
SpfRrFfMacScheduler::DoSchedDlTriggerReq (const struct FfMacSchedSapProvider::SchedDlTriggerReqParameters& params)
{
  NS_LOG_FUNCTION (this << " DL Frame no. " << (params.m_sfnSf >> 4) << " subframe no. " << (0xF & params.m_sfnSf));
  // API generated by RLC for triggering the scheduling of a DL subframe

  RefreshDlCqiMaps ();
  int rbgSize = GetRbgSize (m_cschedCellConfig.m_dlBandwidth);
  int rbgNum = m_cschedCellConfig.m_dlBandwidth / rbgSize;
    NS_LOG_INFO (this << " DL RB INFO: BandWidth" << (int)(m_cschedCellConfig.m_dlBandwidth) << " ResourceBlockGroupSize " << rbgSize << "NUmber Of ResourceBlockGroupSize " << rbgNum);

  FfMacSchedSapUser::SchedDlConfigIndParameters ret;

  // Generate RBGs map
  std::vector <bool> rbgMap;
  uint16_t rbgAllocatedNum = 0;
  std::set <uint16_t> rntiAllocated;
  rbgMap.resize (m_cschedCellConfig.m_dlBandwidth / rbgSize, false);

  //   update UL HARQ proc id
  std::map <uint16_t, uint8_t>::iterator itProcId;
  for (itProcId = m_ulHarqCurrentProcessId.begin (); itProcId != m_ulHarqCurrentProcessId.end (); itProcId++)
    {
      (*itProcId).second = ((*itProcId).second + 1) % HARQ_PROC_NUM;
    }

  // RACH Allocation
  m_rachAllocationMap.resize (m_cschedCellConfig.m_ulBandwidth, 0);
  uint16_t rbStart = 0;
  std::vector <struct RachListElement_s>::iterator itRach;
  for (itRach = m_rachList.begin (); itRach != m_rachList.end (); itRach++)
    {
      NS_ASSERT_MSG (m_amc->GetTbSizeFromMcs (m_ulGrantMcs, m_cschedCellConfig.m_ulBandwidth) > (*itRach).m_estimatedSize, " Default UL Grant MCS does not allow to send RACH messages");
      BuildRarListElement_s newRar;
      newRar.m_rnti = (*itRach).m_rnti;
      // DL-RACH Allocation
      // Ideal: no needs of configuring m_dci
      // UL-RACH Allocation
      newRar.m_grant.m_rnti = newRar.m_rnti;
      newRar.m_grant.m_mcs = m_ulGrantMcs;
      uint16_t rbLen = 1;
      uint16_t tbSizeBits = 0;
      // find lowest TB size that fits UL grant estimated size
      while ((tbSizeBits < (*itRach).m_estimatedSize) && (rbStart + rbLen < m_cschedCellConfig.m_ulBandwidth))
        {
          rbLen++;
          tbSizeBits = m_amc->GetTbSizeFromMcs (m_ulGrantMcs, rbLen);
        }
      if (tbSizeBits < (*itRach).m_estimatedSize)
        {
          // no more allocation space: finish allocation
          break;
        }
      newRar.m_grant.m_rbStart = rbStart;
      newRar.m_grant.m_rbLen = rbLen;
      newRar.m_grant.m_tbSize = tbSizeBits / 8;
      newRar.m_grant.m_hopping = false;
      newRar.m_grant.m_tpc = 0;
      newRar.m_grant.m_cqiRequest = false;
      newRar.m_grant.m_ulDelay = false;
      NS_LOG_INFO (this << " UL grant allocated to RNTI " << (*itRach).m_rnti << " rbStart " << rbStart << " rbLen " << rbLen << " MCS " << (uint16_t) m_ulGrantMcs << " tbSize " << newRar.m_grant.m_tbSize);
      for (uint16_t i = rbStart; i < rbStart + rbLen; i++)
        {
          m_rachAllocationMap.at (i) = (*itRach).m_rnti;
        }

      if (m_harqOn == true)
        {
          // generate UL-DCI for HARQ retransmissions
          UlDciListElement_s uldci;
          uldci.m_rnti = newRar.m_rnti;
          uldci.m_rbLen = rbLen;
          uldci.m_rbStart = rbStart;
          uldci.m_mcs = m_ulGrantMcs;
          uldci.m_tbSize = tbSizeBits / 8;
          uldci.m_ndi = 1;
          uldci.m_cceIndex = 0;
          uldci.m_aggrLevel = 1;
          uldci.m_ueTxAntennaSelection = 3; // antenna selection OFF
          uldci.m_hopping = false;
          uldci.m_n2Dmrs = 0;
          uldci.m_tpc = 0; // no power control
          uldci.m_cqiRequest = false; // only period CQI at this stage
          uldci.m_ulIndex = 0; // TDD parameter
          uldci.m_dai = 1; // TDD parameter
          uldci.m_freqHopping = 0;
          uldci.m_pdcchPowerOffset = 0; // not used

          uint8_t harqId = 0;
          std::map <uint16_t, uint8_t>::iterator itProcId;
          itProcId = m_ulHarqCurrentProcessId.find (uldci.m_rnti);
          if (itProcId == m_ulHarqCurrentProcessId.end ())
            {
              NS_FATAL_ERROR ("No info find in HARQ buffer for UE " << uldci.m_rnti);
            }
          harqId = (*itProcId).second;
          std::map <uint16_t, UlHarqProcessesDciBuffer_t>::iterator itDci = m_ulHarqProcessesDciBuffer.find (uldci.m_rnti);
          if (itDci == m_ulHarqProcessesDciBuffer.end ())
            {
              NS_FATAL_ERROR ("Unable to find RNTI entry in UL DCI HARQ buffer for RNTI " << uldci.m_rnti);
            }
          (*itDci).second.at (harqId) = uldci;
        }

      rbStart = rbStart + rbLen;
      ret.m_buildRarList.push_back (newRar);
    }
  m_rachList.clear ();

  // Process DL HARQ feedback
  RefreshHarqProcesses ();
  // retrieve past HARQ retx buffered
  if (m_dlInfoListBuffered.size () > 0)
    {
      if (params.m_dlInfoList.size () > 0)
        {
          NS_LOG_INFO (this << " Received DL-HARQ feedback");
          m_dlInfoListBuffered.insert (m_dlInfoListBuffered.end (), params.m_dlInfoList.begin (), params.m_dlInfoList.end ());
        }
    }
  else
    {
      if (params.m_dlInfoList.size () > 0)
        {
          m_dlInfoListBuffered = params.m_dlInfoList;
        }
    }
  if (m_harqOn == false)
    {
      // Ignore HARQ feedback
      m_dlInfoListBuffered.clear ();
    }
  std::vector <struct DlInfoListElement_s> dlInfoListUntxed;
  for (uint16_t i = 0; i < m_dlInfoListBuffered.size (); i++)
    {
      std::set <uint16_t>::iterator itRnti = rntiAllocated.find (m_dlInfoListBuffered.at (i).m_rnti);
      if (itRnti != rntiAllocated.end ())
        {
          // RNTI already allocated for retx
          continue;
        }
      uint8_t nLayers = m_dlInfoListBuffered.at (i).m_harqStatus.size ();
      std::vector <bool> retx;
      NS_LOG_INFO (this << " Processing DLHARQ feedback");
      if (nLayers == 1)
        {
          retx.push_back (m_dlInfoListBuffered.at (i).m_harqStatus.at (0) == DlInfoListElement_s::NACK);
          retx.push_back (false);
        }
      else
        {
          retx.push_back (m_dlInfoListBuffered.at (i).m_harqStatus.at (0) == DlInfoListElement_s::NACK);
          retx.push_back (m_dlInfoListBuffered.at (i).m_harqStatus.at (1) == DlInfoListElement_s::NACK);
        }
      if (retx.at (0) || retx.at (1))
        {
          // retrieve HARQ process information
          uint16_t rnti = m_dlInfoListBuffered.at (i).m_rnti;
          uint8_t harqId = m_dlInfoListBuffered.at (i).m_harqProcessId;
          NS_LOG_INFO (this << " HARQ retx RNTI " << rnti << " harqId " << (uint16_t)harqId);
          std::map <uint16_t, DlHarqProcessesDciBuffer_t>::iterator itHarq = m_dlHarqProcessesDciBuffer.find (rnti);
          if (itHarq == m_dlHarqProcessesDciBuffer.end ())
            {
              NS_FATAL_ERROR ("No info find in HARQ buffer for UE " << rnti);
            }

          DlDciListElement_s dci = (*itHarq).second.at (harqId);
          int rv = 0;
          if (dci.m_rv.size () == 1)
            {
              rv = dci.m_rv.at (0);
            }
          else
            {
              rv = (dci.m_rv.at (0) > dci.m_rv.at (1) ? dci.m_rv.at (0) : dci.m_rv.at (1));
            }

          if (rv == 3)
            {
              // maximum number of retx reached -> drop process
              NS_LOG_INFO ("Max number of retransmissions reached -> drop process");
              std::map <uint16_t, DlHarqProcessesStatus_t>::iterator it = m_dlHarqProcessesStatus.find (rnti);
              if (it == m_dlHarqProcessesStatus.end ())
                {
                  NS_LOG_ERROR ("No info find in HARQ buffer for UE (might change eNB) " << m_dlInfoListBuffered.at (i).m_rnti);
                }
              (*it).second.at (harqId) = 0;
              std::map <uint16_t, DlHarqRlcPduListBuffer_t>::iterator itRlcPdu =  m_dlHarqProcessesRlcPduListBuffer.find (rnti);
              if (itRlcPdu == m_dlHarqProcessesRlcPduListBuffer.end ())
                {
                  NS_FATAL_ERROR ("Unable to find RlcPdcList in HARQ buffer for RNTI " << m_dlInfoListBuffered.at (i).m_rnti);
                }
              for (uint16_t k = 0; k < (*itRlcPdu).second.size (); k++)
                {
                  (*itRlcPdu).second.at (k).at (harqId).clear ();
                }
              continue;
            }
          // check the feasibility of retransmitting on the same RBGs
          // translate the DCI to Spectrum framework
          std::vector <int> dciRbg;
          uint32_t mask = 0x1;
          NS_LOG_INFO ("Original RBGs " << dci.m_rbBitmap << " rnti " << dci.m_rnti);
          for (int j = 0; j < 32; j++)
            {
              if (((dci.m_rbBitmap & mask) >> j) == 1)
                {
                  dciRbg.push_back (j);
                  NS_LOG_INFO ("\t" << j);
                }
              mask = (mask << 1);
            }
          bool free = true;
          for (uint8_t j = 0; j < dciRbg.size (); j++)
            {
              if (rbgMap.at (dciRbg.at (j)) == true)
                {
                  free = false;
                  break;
                }
            }
          if (free)
            {
              // use the same RBGs for the retx
              // reserve RBGs
              for (uint8_t j = 0; j < dciRbg.size (); j++)
                {
                  rbgMap.at (dciRbg.at (j)) = true;
                  NS_LOG_INFO ("RBG " << dciRbg.at (j) << " assigned");
                  rbgAllocatedNum++;
                }

              NS_LOG_INFO (this << " Send retx in the same RBGs");
            }
          else
            {
              // find RBGs for sending HARQ retx
              uint8_t j = 0;
              uint8_t rbgId = (dciRbg.at (dciRbg.size () - 1) + 1) % rbgNum;
              uint8_t startRbg = dciRbg.at (dciRbg.size () - 1);
              std::vector <bool> rbgMapCopy = rbgMap;
              while ((j < dciRbg.size ())&&(startRbg != rbgId))
                {
                  if (rbgMapCopy.at (rbgId) == false)
                    {
                      rbgMapCopy.at (rbgId) = true;
                      dciRbg.at (j) = rbgId;
                      j++;
                    }
                  rbgId = (rbgId + 1) % rbgNum;
                }
              if (j == dciRbg.size ())
                {
                  // find new RBGs -> update DCI map
                  uint32_t rbgMask = 0;
                  for (uint16_t k = 0; k < dciRbg.size (); k++)
                    {
                      rbgMask = rbgMask + (0x1 << dciRbg.at (k));
                      NS_LOG_INFO (this << " New allocated RBG " << dciRbg.at (k));
                      rbgAllocatedNum++;
                    }
                  dci.m_rbBitmap = rbgMask;
                  rbgMap = rbgMapCopy;
                }
              else
                {
                  // HARQ retx cannot be performed on this TTI -> store it
                  dlInfoListUntxed.push_back (m_dlInfoListBuffered.at (i));
                  NS_LOG_INFO (this << " No resource for this retx -> buffer it");
                }
            }
          // retrieve RLC PDU list for retx TBsize and update DCI
          BuildDataListElement_s newEl;
          std::map <uint16_t, DlHarqRlcPduListBuffer_t>::iterator itRlcPdu =  m_dlHarqProcessesRlcPduListBuffer.find (rnti);
          if (itRlcPdu == m_dlHarqProcessesRlcPduListBuffer.end ())
            {
              NS_FATAL_ERROR ("Unable to find RlcPdcList in HARQ buffer for RNTI " << rnti);
            }
          for (uint8_t j = 0; j < nLayers; j++)
            {
              if (retx.at (j))
                {
                  if (j >= dci.m_ndi.size ())
                    {
                      // for avoiding errors in MIMO transient phases
                      dci.m_ndi.push_back (0);
                      dci.m_rv.push_back (0);
                      dci.m_mcs.push_back (0);
                      dci.m_tbsSize.push_back (0);
                      NS_LOG_INFO (this << " layer " << (uint16_t)j << " no txed (MIMO transition)");

                    }
                  else
                    {
                      dci.m_ndi.at (j) = 0;
                      dci.m_rv.at (j)++;
                      (*itHarq).second.at (harqId).m_rv.at (j)++;
                      NS_LOG_INFO (this << " layer " << (uint16_t)j << " RV " << (uint16_t)dci.m_rv.at (j));
                    }
                }
              else
                {
                  // empty TB of layer j
                  dci.m_ndi.at (j) = 0;
                  dci.m_rv.at (j) = 0;
                  dci.m_mcs.at (j) = 0;
                  dci.m_tbsSize.at (j) = 0;
                  NS_LOG_INFO (this << " layer " << (uint16_t)j << " no retx");
                }
            }

          for (uint16_t k = 0; k < (*itRlcPdu).second.at (0).at (dci.m_harqProcess).size (); k++)
            {
              std::vector <struct RlcPduListElement_s> rlcPduListPerLc;
              for (uint8_t j = 0; j < nLayers; j++)
                {
                  if (retx.at (j))
                    {
                      if (j < dci.m_ndi.size ())
                        {
                          rlcPduListPerLc.push_back ((*itRlcPdu).second.at (j).at (dci.m_harqProcess).at (k));
                        }
                    }
                }

              if (rlcPduListPerLc.size () > 0)
                {
                  newEl.m_rlcPduList.push_back (rlcPduListPerLc);
                }
            }
          newEl.m_rnti = rnti;
          newEl.m_dci = dci;
          (*itHarq).second.at (harqId).m_rv = dci.m_rv;
          // refresh timer
          std::map <uint16_t, DlHarqProcessesTimer_t>::iterator itHarqTimer = m_dlHarqProcessesTimer.find (rnti);
          if (itHarqTimer== m_dlHarqProcessesTimer.end ())
            {
              NS_FATAL_ERROR ("Unable to find HARQ timer for RNTI " << (uint16_t)rnti);
            }
          (*itHarqTimer).second.at (harqId) = 0;
          ret.m_buildDataList.push_back (newEl);
          rntiAllocated.insert (rnti);
        }
      else
        {
          // update HARQ process status
          NS_LOG_INFO (this << " HARQ ACK UE " << m_dlInfoListBuffered.at (i).m_rnti);
          std::map <uint16_t, DlHarqProcessesStatus_t>::iterator it = m_dlHarqProcessesStatus.find (m_dlInfoListBuffered.at (i).m_rnti);
          if (it == m_dlHarqProcessesStatus.end ())
            {
              NS_FATAL_ERROR ("No info find in HARQ buffer for UE " << m_dlInfoListBuffered.at (i).m_rnti);
            }
          (*it).second.at (m_dlInfoListBuffered.at (i).m_harqProcessId) = 0;
          std::map <uint16_t, DlHarqRlcPduListBuffer_t>::iterator itRlcPdu =  m_dlHarqProcessesRlcPduListBuffer.find (m_dlInfoListBuffered.at (i).m_rnti);
          if (itRlcPdu == m_dlHarqProcessesRlcPduListBuffer.end ())
            {
              NS_FATAL_ERROR ("Unable to find RlcPdcList in HARQ buffer for RNTI " << m_dlInfoListBuffered.at (i).m_rnti);
            }
          for (uint16_t k = 0; k < (*itRlcPdu).second.size (); k++)
            {
              (*itRlcPdu).second.at (k).at (m_dlInfoListBuffered.at (i).m_harqProcessId).clear ();
            }
        }
    }
  m_dlInfoListBuffered.clear ();
  m_dlInfoListBuffered = dlInfoListUntxed;

  if (rbgAllocatedNum == rbgNum)
    {
      // all the RBGs are already allocated -> exit
      if ((ret.m_buildDataList.size () > 0) || (ret.m_buildRarList.size () > 0))
        {
          m_schedSapUser->SchedDlConfigInd (ret);
        }
      return;
    }

  // Get the actual active flows (queue!=0)
  std::list<FfMacSchedSapProvider::SchedDlRlcBufferReqParameters>::iterator it;
  m_rlcBufferReq.sort (SortRlcBufferReq);
  int nflows = 0;
  int nTbs = 0;
  std::map <uint16_t,uint8_t> lcActivesPerRnti; // tracks how many active LCs per RNTI there are
  std::map <uint16_t,uint8_t>::iterator itLcRnti;
    std::list <std::pair<FfMacSchedSapProvider::SchedDlRlcBufferReqParameters,uint8_t>> canBeTransferred;

  for (it = m_rlcBufferReq.begin (); it != m_rlcBufferReq.end (); it++)
    {
      // remove old entries of this UE-LC
      std::set <uint16_t>::iterator itRnti = rntiAllocated.find ((*it).m_rnti); //!!!
      if ( (((*it).m_rlcTransmissionQueueSize > 0)
            || ((*it).m_rlcRetransmissionQueueSize > 0)
            || ((*it).m_rlcStatusPduSize > 0))
           && (itRnti == rntiAllocated.end ())  // UE must not be allocated for HARQ retx !!!
           && (HarqProcessAvailability ((*it).m_rnti))  ) // UE needs HARQ proc free

        {
          NS_LOG_LOGIC (this << " User " << (*it).m_rnti << " LC " << (uint16_t)(*it).m_logicalChannelIdentity << " is active, status  " << (*it).m_rlcStatusPduSize << " retx " << (*it).m_rlcRetransmissionQueueSize << " tx " << (*it).m_rlcTransmissionQueueSize);
          std::map <uint16_t,uint8_t>::iterator itCqi = m_p10CqiRxed.find ((*it).m_rnti);
          uint8_t cqi = 0;
          if (itCqi != m_p10CqiRxed.end ())
            {
              cqi = (*itCqi).second;
            }
          else
            {
              cqi = 1; // lowest value fro trying a transmission
            }
          if (cqi != 0)
            {
              // CQI == 0 means "out of range" (see table 7.2.3-1 of 36.213)
              nflows++;
              canBeTransferred.push_back(std::pair<FfMacSchedSapProvider::SchedDlRlcBufferReqParameters,uint8_t>((*it),cqi));
              itLcRnti = lcActivesPerRnti.find ((*it).m_rnti);
              if (itLcRnti != lcActivesPerRnti.end ())
                {
                  (*itLcRnti).second++;
                }
              else
                {
                  lcActivesPerRnti.insert (std::pair<uint16_t, uint8_t > ((*it).m_rnti, 1));
                  nTbs++;
                }

            }
        }
    }

  if (nflows == 0)
    {
      if ((ret.m_buildDataList.size () > 0) || (ret.m_buildRarList.size () > 0))
        {
          m_schedSapUser->SchedDlConfigInd (ret);
        }
      return;
    }
  // Divide the resource equally among the active users according to
  // Resource allocation type 0 (see sec 7.1.6.1 of 36.213)

  int rbgPerTb = (nTbs > 0) ? ((rbgNum - rbgAllocatedNum) / nTbs) : INT_MAX;
  NS_LOG_INFO (this << " Flows to be transmitted " << nflows << " rbgPerTb " << rbgPerTb);
  if (rbgPerTb == 0)
    {
      rbgPerTb = 1;                // at least 1 rbg per TB (till available resource)
    }
    int rbgAllocated = 0;
    
    int rbgRemaining = rbgNum - rbgAllocatedNum;
//    int rbgMaskTemp = 0;

    std::list <std::pair<FfMacSchedSapProvider::SchedDlRlcBufferReqParameters,uint8_t>>::iterator its;
    std::map <uint16_t,uint8_t>::iterator itTxMode;
//    bool hasTransmittedReset;
//    hasTransmittedReset = false;
    NS_LOG_INFO (this << "++++SOYO+++++start Spf+++++++");
    
    std::list <std::tuple<uint16_t,uint8_t,int,int,uint16_t>> trans_rnti_lci_size_mcs_tbsize;
    
    uint32_t SumAllBufferRemaining = updatePacketBuffer(canBeTransferred);
    while(    rbgRemaining > 0
            &&SumAllBufferRemaining> 0
            ){
        std::pair<uint16_t, uint8_t > nextRntiLc;
        std::map <uint16_t,uint8_t>::iterator temp;
       //find the shorteset packet of user that havent be transmitted
        canBeTransferred.sort(SortRlcBufferReqCqiPair);
        its = canBeTransferred.begin();
        for(; its != canBeTransferred.end(); its++ ){
            if(rbgRemaining <= 0){
                break;
            }
            temp = m_transmittedRnti.begin();
            bool hasTransmitted = false;
            for(; temp != m_transmittedRnti.end(); temp++ ){
                if((*temp).first == (*its).first.m_rnti && (*temp).second == (*its).first.m_logicalChannelIdentity){
                    hasTransmitted = true;
                    break;
                }
            }
            if(!hasTransmitted){
                //transmit
                if(rbgRemaining <= 0){
                    break;
                }
                else{
                    //transmit update rbg
                    if((*its).first.m_vendorSpecificList.size()>0 && (*its).first.m_vendorSpecificList.at(0).m_type == 0){
                        Ptr<RlcPacketLengths> rpl = DynamicCast<RlcPacketLengths> ((*its).first.m_vendorSpecificList.at(0).m_value);
                        if(rpl->GetFirstPacketLength() >= 0){
                            //transmit
                            uint16_t rnti = (*its).first.m_rnti;
                            NS_LOG_INFO (this << "++++SOYO+++++RNIT+++++++" << rnti);
                            uint8_t lci = (*its).first.m_logicalChannelIdentity;
                            uint32_t fps = rpl->GetFirstPacketLength() + 4; //add Rlc header size    
                            itTxMode = m_uesTxMode.find (rnti);
                            if (itTxMode == m_uesTxMode.end ())
                            {
                              NS_FATAL_ERROR ("No Transmission Mode info on user " << rnti);
                            }
                            int nLayer = TransmissionModesLayers::TxMode2LayerNum ((*itTxMode).second);                            
//                            BuildDataListElement_s newEl; //!!!
//                            newEl.m_rnti = rnti;
//                            DlDciListElement_s newDci; //!!!!
//                            newDci.m_rnti = rnti;
//                            newDci.m_harqProcess = UpdateHarqProcessId (rnti);
//                            NS_LOG_INFO (this << "++++SOYO+++++HARQ ID+++++++ " << (int)newDci.m_harqProcess);
//                            newDci.m_resAlloc = 0;
//                            newDci.m_rbBitmap = 0;
                            int mcs_of_this_rnti = 0;
                            std::map <uint16_t,uint8_t>::iterator itCqi = m_p10CqiRxed.find (rnti);
                            for (uint8_t i = 0; i < nLayer; i++)
                            {
                                if (itCqi == m_p10CqiRxed.end ())
                                {
//                                    newDci.m_mcs.push_back (0); // no info on this user -> lowest MCS
                                    mcs_of_this_rnti = 0;
                                }
                                else
                                {
                                    mcs_of_this_rnti = m_amc->GetMcsFromCqi ((*itCqi).second);
                                }
                            }
                            
                            
                            uint16_t tbSize = 0;    //transmitted byte size
                            uint16_t rbSize = 0;    //used rb size for thie transmission
                            NS_LOG_LOGIC (this << "++++SOYO++++ Remaining Block NUmber " << rbgRemaining );

                            if(((m_amc->GetTbSizeFromMcs(mcs_of_this_rnti, rbgRemaining * rbgSize) / 8 )* nLayer) < (int)fps ){
                                NS_LOG_LOGIC (this << "++++SOYO++++ Remaining Block is Not enough for " << fps <<" bytes rnti " << rnti << " remaining rbg " << rbgRemaining );

                                tbSize = ((m_amc->GetTbSizeFromMcs(mcs_of_this_rnti, rbgRemaining * rbgSize) / 8 )* nLayer);
                                rbSize = rbgRemaining;
                            }
                            else{
                                NS_LOG_LOGIC (this << "++++SOYO++++ Remaining Block is enough for " << fps <<" bytes rnti " << rnti << " remaining rbg " << rbgRemaining );
                                rbSize = m_amc->GetNPRBSizeFromTbSize(mcs_of_this_rnti,(int)(fps*8/nLayer));
                                rbSize = rbSize % rbgSize == 0 ? rbSize / rbgSize : rbSize / rbgSize + 1;
                                tbSize = ((m_amc->GetTbSizeFromMcs(mcs_of_this_rnti, rbSize * rbgSize) / 8 )* nLayer);
//                                m_transmittedRnti.insert(std::pair<uint16_t,uint8_t> (rnti,lci));
                            }
                            NS_LOG_INFO (this << "++++SOYO+++++rbsize+++++++ " << rbSize <<" rnti " << rnti);

//                            std::vector <struct RlcPduListElement_s> newRlcPduLe;
                            for (uint8_t j = 0; j < nLayer; j++)
                            {
//                                RlcPduListElement_s newRlcEl;
//                                newRlcEl.m_logicalChannelIdentity = lci;
//                                NS_LOG_INFO (this << " LCID " << (int) newRlcEl.m_logicalChannelIdentity << " size " << tbSize/nLayer << " ID " << rnti << " layer " << (uint16_t)j);
//                                newRlcEl.m_size = tbSize/nLayer;
                                UpdateDlRlcBufferInfo (rnti, lci, tbSize/nLayer);
//                                newRlcPduLe.push_back (newRlcEl);
//                                if (m_harqOn == true)
//                                {
//                                    // store RLC PDU list for HARQ
//                                    NS_LOG_INFO (this << " store RLC PDU list for HARQ " );
//
//                                    std::map <uint16_t, DlHarqRlcPduListBuffer_t>::iterator itRlcPdu =  m_dlHarqProcessesRlcPduListBuffer.find (rnti);
//                                    if (itRlcPdu == m_dlHarqProcessesRlcPduListBuffer.end ())
//                                      {
//                                        NS_FATAL_ERROR ("Unable to find RlcPdcList in HARQ buffer for RNTI " << rnti);
//                                      }
//                                    (*itRlcPdu).second.at (j).at (newDci.m_harqProcess).push_back (newRlcEl);
//                                }
//                                newEl.m_rlcPduList.push_back (newRlcPduLe);
                            }
//                            uint32_t rbgMask = 0;
//                            NS_LOG_INFO (this << " DL - Allocate user " << newEl.m_rnti << " Logic Channel Id " <<  (int)lci << " bytes " << tbSize << " mcs " << (uint16_t) newDci.m_mcs.at (0) << " harqId " << (int)newDci.m_harqProcess <<  " layers " << nLayer);
//                            NS_LOG_INFO ("RBG:");
//                            uint16_t i = 0;
//                            while (i < rbSize)
//                            {
//                                if (rbgMap.at (rbgMaskTemp) == false)
//                                {
//                                  rbgMask = rbgMask + (0x1 << rbgMaskTemp);
//                                  NS_LOG_INFO ("\t " << rbgMaskTemp);
//                                  i++;
//                                  rbgMap.at (rbgMaskTemp) = true;
//                                  rbgAllocatedNum++;
//                                }
//                                rbgMaskTemp++;
//                            }
//                            newDci.m_rbBitmap = rbgMask;
//                            for (int i = 0; i < nLayer; i++)
//                            {
//                                newDci.m_tbsSize.push_back (tbSize);
//                                newDci.m_ndi.push_back (1);
//                                newDci.m_rv.push_back (0);
//                            }
//                            newDci.m_tpc = 1; //1 is mapped to 0 in Accumulated Mode and to -1 in Absolute Mode
//                            newEl.m_dci = newDci;
//                            if (m_harqOn == true)
//                            {
//                                // store DCI for HARQ
//                                std::map <uint16_t, DlHarqProcessesDciBuffer_t>::iterator itDci = m_dlHarqProcessesDciBuffer.find (newEl.m_rnti);
//                                if (itDci == m_dlHarqProcessesDciBuffer.end ())
//                                    {
//                                      NS_FATAL_ERROR ("Unable to find RNTI entry in DCI HARQ buffer for RNTI " << newEl.m_rnti);
//                                    }
//                                (*itDci).second.at (newDci.m_harqProcess) = newDci;
//                                // refresh timer
//                                std::map <uint16_t, DlHarqProcessesTimer_t>::iterator itHarqTimer =  m_dlHarqProcessesTimer.find (newEl.m_rnti);
//                                if (itHarqTimer== m_dlHarqProcessesTimer.end ())
//                                    {
//                                      NS_FATAL_ERROR ("Unable to find HARQ timer for RNTI " << (uint16_t)newEl.m_rnti);
//                                    }
//                                (*itHarqTimer).second.at (newDci.m_harqProcess) = 0;
//                            }
//                            ret.m_buildDataList.push_back (newEl);
                            rpl->RemoveSomeAtFirst(tbSize-4); //remove header
//                            updatePacketBuffer(canBeTransferred);
                            rbgRemaining -= rbSize;
                            trans_rnti_lci_size_mcs_tbsize.push_back(std::tuple<uint16_t,uint8_t,int,int,uint16_t>(rnti,lci,rbSize,mcs_of_this_rnti,tbSize/nLayer));
                        }
                    } 
                }
            }    
        }
        SumAllBufferRemaining = updatePacketBuffer(canBeTransferred);
        if(its == canBeTransferred.end()){
            m_transmittedRnti.clear();
        }
    }
    
    trans_rnti_lci_size_mcs_tbsize.sort(SortTrans);
    
    std::list <std::tuple<uint16_t,uint8_t,int,int,uint16_t>> combined;
    std::list <std::tuple<uint16_t,uint8_t,int,int,uint16_t>>::iterator ittrans;
    std::list <std::tuple<uint16_t,uint8_t,int,int,uint16_t>>::iterator itcom;

    for(ittrans = trans_rnti_lci_size_mcs_tbsize.begin(); ittrans != trans_rnti_lci_size_mcs_tbsize.end(); ittrans ++){
        itcom = combined.begin();
        uint16_t trans_rnti  = std::get<0>(*ittrans);
        uint8_t trans_lci   = std::get<1>(*ittrans); 
        bool has_com = false;
        while (itcom != combined.end()){
            uint16_t com_rnti = std::get<0>(*itcom);            
            uint8_t com_lci = std::get<1>(*itcom); 
            if(     com_rnti    == trans_rnti
               &&   com_lci     == trans_lci ){
                std::get<2>(*itcom) += std::get<2>(*ittrans);
                std::get<4>(*itcom) += std::get<4>(*ittrans);
                NS_LOG_INFO ("++++++SOYO+++++Combining++++++RNTI " << (int) com_rnti << "  LCID " << (int) com_lci);

                has_com = true;
                break;
            }
            itcom ++;
        }
        if(has_com == false){
            combined.push_back((*ittrans));
        }
    }
    combined.sort(SortTrans);
    int rbgMaskTemp = 0;

    for(itcom = combined.begin();itcom != combined.end();){
        uint16_t rnti = std::get<0>(*itcom); 
        itTxMode = m_uesTxMode.find (rnti);
        int nLayer = TransmissionModesLayers::TxMode2LayerNum ((*itTxMode).second);
        BuildDataListElement_s newEl;
        newEl.m_rnti = rnti;
        DlDciListElement_s newDci;
        newDci.m_rnti = rnti;
        newDci.m_harqProcess = UpdateHarqProcessId (rnti);
        newDci.m_resAlloc = 0;
        newDci.m_rbBitmap = 0;
        std::map <uint16_t,uint8_t>::iterator itCqi = m_p10CqiRxed.find (rnti);
        for (uint8_t i = 0; i < nLayer; i++)
        {
            if (itCqi == m_p10CqiRxed.end ())
            {
                newDci.m_mcs.push_back (0); // no info on this user -> lowest MCS
            }
            else
            {
                newDci.m_mcs.push_back ( m_amc->GetMcsFromCqi ((*itCqi).second) );
            }
        }
        uint16_t tbsize_perlayer_total = 0;
        int rbg_all_lc = 0;
        while (std::get<0>(*itcom) == newEl.m_rnti){
            std::vector <struct RlcPduListElement_s> newRlcPduLe;
            tbsize_perlayer_total += std::get<4>(*itcom);
            rbg_all_lc += std::get<2>(*itcom);
            for (uint8_t j = 0; j < nLayer; j++)
                {
                    uint16_t tbsize_perlayer = std::get<4>(*itcom);  
                    uint8_t lci = std::get<1>(*itcom);
                    RlcPduListElement_s newRlcEl;
                    newRlcEl.m_logicalChannelIdentity = lci;
                    NS_LOG_INFO (this << "LCID " << (int) newRlcEl.m_logicalChannelIdentity << " size " << (int)tbsize_perlayer << " ID " << rnti << " layer " << (uint16_t)j);
                    newRlcEl.m_size = tbsize_perlayer;
                    newRlcPduLe.push_back (newRlcEl);
                    if (m_harqOn == true)
                    {
                        // store RLC PDU list for HARQ
                        std::map <uint16_t, DlHarqRlcPduListBuffer_t>::iterator itRlcPdu =  m_dlHarqProcessesRlcPduListBuffer.find (rnti);
                        if (itRlcPdu == m_dlHarqProcessesRlcPduListBuffer.end ())
                          {
                            NS_FATAL_ERROR ("Unable to find RlcPdcList in HARQ buffer for RNTI " << (*it).m_rnti);
                          }
                        (*itRlcPdu).second.at (j).at (newDci.m_harqProcess).push_back (newRlcEl);
                    }

                }
            newEl.m_rlcPduList.push_back (newRlcPduLe);  
            itcom++;
            if(itcom == combined.end()){
                break;
            }
        }

        uint32_t rbgMask = 0;
        uint16_t i = 0;
        NS_LOG_INFO (this << " DL - Allocate user " << newEl.m_rnti << " LCs " << (uint16_t)(*itLcRnti).second << " bytes " << (int)(tbsize_perlayer_total*nLayer )<< " mcs " << (uint16_t) newDci.m_mcs.at (0) << " harqId " << (uint16_t)newDci.m_harqProcess <<  " layers " << nLayer);
        NS_LOG_INFO ("RBG:" << rbg_all_lc);
        while (i < rbg_all_lc)
        {
          if (rbgMap.at (rbgMaskTemp) == false)
            {
              rbgMask = rbgMask + (0x1 << rbgMaskTemp);
              NS_LOG_INFO ("\t " << rbgMaskTemp);
              i++;
              rbgMap.at (rbgMaskTemp) = true;
              rbgAllocatedNum++;
            }
          rbgMaskTemp++;
        }
        newDci.m_rbBitmap = rbgMask; // (32 bit bitmap see 7.1.6 of 36.213)

        for (int i = 0; i < nLayer; i++)
        {
            newDci.m_tbsSize.push_back (tbsize_perlayer_total);
            newDci.m_ndi.push_back (1);
            newDci.m_rv.push_back (0);
        }

        newDci.m_tpc = 1; //1 is mapped to 0 in Accumulated Mode and to -1 in Absolute Mode

        newEl.m_dci = newDci;
        if (m_harqOn == true)
        {
          // store DCI for HARQ
          std::map <uint16_t, DlHarqProcessesDciBuffer_t>::iterator itDci = m_dlHarqProcessesDciBuffer.find (newEl.m_rnti);
          if (itDci == m_dlHarqProcessesDciBuffer.end ())
            {
              NS_FATAL_ERROR ("Unable to find RNTI entry in DCI HARQ buffer for RNTI " << newEl.m_rnti);
            }
          (*itDci).second.at (newDci.m_harqProcess) = newDci;
          // refresh timer
          std::map <uint16_t, DlHarqProcessesTimer_t>::iterator itHarqTimer =  m_dlHarqProcessesTimer.find (newEl.m_rnti);
          if (itHarqTimer== m_dlHarqProcessesTimer.end ())
            {
              NS_FATAL_ERROR ("Unable to find HARQ timer for RNTI " << (uint16_t)newEl.m_rnti);
            }
          (*itHarqTimer).second.at (newDci.m_harqProcess) = 0;
        }
        // ...more parameters -> ignored in this version

        ret.m_buildDataList.push_back (newEl);
    
    }
    

      
            
            
          
            
    ret.m_nrOfPdcchOfdmSymbols = 1;
    m_schedSapUser->SchedDlConfigInd (ret);
    
    NS_LOG_INFO (this << "++++SOYO+++++End Spf+++++++");

    return;
    
    if (m_nextRntiDl != 0)
    {
      NS_LOG_DEBUG ("Start from the successive of " << (uint16_t) m_nextRntiDl);
      for (it = m_rlcBufferReq.begin (); it != m_rlcBufferReq.end (); it++)
        {
          if ((*it).m_rnti == m_nextRntiDl)
            {
              // select the next RNTI to starting
              it++;
              if (it == m_rlcBufferReq.end ())
              {
                it = m_rlcBufferReq.begin ();
              }
              m_nextRntiDl = (*it).m_rnti;
              break;
            }
        }

      if (it == m_rlcBufferReq.end ())
        {
          NS_LOG_ERROR (this << " no user found");
        }
    }
    else
    {
      it = m_rlcBufferReq.begin ();
      m_nextRntiDl = (*it).m_rnti;
    }
//    std::map <uint16_t,uint8_t>::iterator itTxMode;
    do
    {
      itLcRnti = lcActivesPerRnti.find ((*it).m_rnti);
      std::set <uint16_t>::iterator itRnti = rntiAllocated.find ((*it).m_rnti);
      if ((itLcRnti == lcActivesPerRnti.end ())||(itRnti != rntiAllocated.end ()))
        {
          // skip this RNTI (no active queue or yet allocated for HARQ)
          uint16_t rntiDiscared = (*it).m_rnti;
          while (it != m_rlcBufferReq.end ())
            {
              if ((*it).m_rnti != rntiDiscared)
                {
                  break;
                }
              it++;
            }
          if (it == m_rlcBufferReq.end ())
            {
              // restart from the first
              it = m_rlcBufferReq.begin ();
            }
          continue;
        }
      itTxMode = m_uesTxMode.find ((*it).m_rnti);
      if (itTxMode == m_uesTxMode.end ())
        {
          NS_FATAL_ERROR ("No Transmission Mode info on user " << (*it).m_rnti);
        }
      int nLayer = TransmissionModesLayers::TxMode2LayerNum ((*itTxMode).second);
      int lcNum = (*itLcRnti).second;
      // create new BuildDataListElement_s for this RNTI
      BuildDataListElement_s newEl; //!!!
      newEl.m_rnti = (*it).m_rnti;
      // create the DlDciListElement_s
      DlDciListElement_s newDci; //!!!!
      newDci.m_rnti = (*it).m_rnti;
      newDci.m_harqProcess = UpdateHarqProcessId ((*it).m_rnti);
      newDci.m_resAlloc = 0;
      newDci.m_rbBitmap = 0;
      std::map <uint16_t,uint8_t>::iterator itCqi = m_p10CqiRxed.find (newEl.m_rnti);
      for (uint8_t i = 0; i < nLayer; i++)
        {
            if (itCqi == m_p10CqiRxed.end ())
            {
              newDci.m_mcs.push_back (0); // no info on this user -> lowest MCS
            }
            else
            {
              newDci.m_mcs.push_back ( m_amc->GetMcsFromCqi ((*itCqi).second) );
            }
        }
        int tbSize = (m_amc->GetTbSizeFromMcs (newDci.m_mcs.at (0), rbgPerTb * rbgSize) / 8);
        uint16_t rlcPduSize = tbSize / lcNum;
        while ((*it).m_rnti == newEl.m_rnti)
        {
          if ( ((*it).m_rlcTransmissionQueueSize > 0)
               || ((*it).m_rlcRetransmissionQueueSize > 0)
               || ((*it).m_rlcStatusPduSize > 0) )
            {
              std::vector <struct RlcPduListElement_s> newRlcPduLe;
              for (uint8_t j = 0; j < nLayer; j++)
                {
                    RlcPduListElement_s newRlcEl;
                    newRlcEl.m_logicalChannelIdentity = (*it).m_logicalChannelIdentity;
                    NS_LOG_INFO (this << "LCID " << (uint32_t) newRlcEl.m_logicalChannelIdentity << " size " << rlcPduSize << " ID " << (*it).m_rnti << " layer " << (uint16_t)j);
                    newRlcEl.m_size = rlcPduSize;
                    UpdateDlRlcBufferInfo ((*it).m_rnti, newRlcEl.m_logicalChannelIdentity, rlcPduSize);
                    newRlcPduLe.push_back (newRlcEl);

                    if (m_harqOn == true)
                    {
                        // store RLC PDU list for HARQ
                        std::map <uint16_t, DlHarqRlcPduListBuffer_t>::iterator itRlcPdu =  m_dlHarqProcessesRlcPduListBuffer.find ((*it).m_rnti);
                        if (itRlcPdu == m_dlHarqProcessesRlcPduListBuffer.end ())
                          {
                            NS_FATAL_ERROR ("Unable to find RlcPdcList in HARQ buffer for RNTI " << (*it).m_rnti);
                          }
                        (*itRlcPdu).second.at (j).at (newDci.m_harqProcess).push_back (newRlcEl);
                    }
                }
              newEl.m_rlcPduList.push_back (newRlcPduLe);
              lcNum--;
            }
          it++;
          if (it == m_rlcBufferReq.end ())
            {
              // restart from the first
              it = m_rlcBufferReq.begin ();
              break;
            }
        }
        uint32_t rbgMask = 0;
        uint16_t i = 0;
        NS_LOG_INFO (this << " DL - Allocate user " << newEl.m_rnti << " LCs " << (uint16_t)(*itLcRnti).second << " bytes " << tbSize << " mcs " << (uint16_t) newDci.m_mcs.at (0) << " harqId " << (uint16_t)newDci.m_harqProcess <<  " layers " << nLayer);
        NS_LOG_INFO ("RBG:");
        while (i < rbgPerTb)
        {
          if (rbgMap.at (rbgAllocated) == false)
            {
              rbgMask = rbgMask + (0x1 << rbgAllocated);
              NS_LOG_INFO ("\t " << rbgAllocated);
              i++;
              rbgMap.at (rbgAllocated) = true;
              rbgAllocatedNum++;
            }
          rbgAllocated++;
        }
      newDci.m_rbBitmap = rbgMask; // (32 bit bitmap see 7.1.6 of 36.213)

      for (int i = 0; i < nLayer; i++)
        {
          newDci.m_tbsSize.push_back (tbSize);
          newDci.m_ndi.push_back (1);
          newDci.m_rv.push_back (0);
        }

      newDci.m_tpc = 1; //1 is mapped to 0 in Accumulated Mode and to -1 in Absolute Mode

      newEl.m_dci = newDci;
      if (m_harqOn == true)
        {
          // store DCI for HARQ
          std::map <uint16_t, DlHarqProcessesDciBuffer_t>::iterator itDci = m_dlHarqProcessesDciBuffer.find (newEl.m_rnti);
          if (itDci == m_dlHarqProcessesDciBuffer.end ())
            {
              NS_FATAL_ERROR ("Unable to find RNTI entry in DCI HARQ buffer for RNTI " << newEl.m_rnti);
            }
          (*itDci).second.at (newDci.m_harqProcess) = newDci;
          // refresh timer
          std::map <uint16_t, DlHarqProcessesTimer_t>::iterator itHarqTimer =  m_dlHarqProcessesTimer.find (newEl.m_rnti);
          if (itHarqTimer== m_dlHarqProcessesTimer.end ())
            {
              NS_FATAL_ERROR ("Unable to find HARQ timer for RNTI " << (uint16_t)newEl.m_rnti);
            }
          (*itHarqTimer).second.at (newDci.m_harqProcess) = 0;
        }
      // ...more parameters -> ignored in this version

      ret.m_buildDataList.push_back (newEl);
      if (rbgAllocatedNum == rbgNum)
        {
          m_nextRntiDl = newEl.m_rnti; // store last RNTI served
          break;                       // no more RGB to be allocated
        }
    }
  while ((*it).m_rnti != m_nextRntiDl);

  ret.m_nrOfPdcchOfdmSymbols = 1;   /// \todo check correct value according the DCIs txed  

  m_schedSapUser->SchedDlConfigInd (ret);
  return;
}

void
SpfRrFfMacScheduler::DoSchedDlRachInfoReq (const struct FfMacSchedSapProvider::SchedDlRachInfoReqParameters& params)
{
  NS_LOG_FUNCTION (this);
  
  m_rachList = params.m_rachList;

  return;
}

void
SpfRrFfMacScheduler::DoSchedDlCqiInfoReq (const struct FfMacSchedSapProvider::SchedDlCqiInfoReqParameters& params)
{
  NS_LOG_FUNCTION (this);

  std::map <uint16_t,uint8_t>::iterator it;
  for (unsigned int i = 0; i < params.m_cqiList.size (); i++)
    {
      if ( params.m_cqiList.at (i).m_cqiType == CqiListElement_s::P10 )
        {
          NS_LOG_LOGIC ("wideband CQI " <<  (uint32_t) params.m_cqiList.at (i).m_wbCqi.at (0) << " reported");
          std::map <uint16_t,uint8_t>::iterator it;
          uint16_t rnti = params.m_cqiList.at (i).m_rnti;
          it = m_p10CqiRxed.find (rnti);
          if (it == m_p10CqiRxed.end ())
            {
              // create the new entry
              m_p10CqiRxed.insert ( std::pair<uint16_t, uint8_t > (rnti, params.m_cqiList.at (i).m_wbCqi.at (0)) ); // only codeword 0 at this stage (SISO)
              // generate correspondent timer
              m_p10CqiTimers.insert ( std::pair<uint16_t, uint32_t > (rnti, m_cqiTimersThreshold));
            }
          else
            {
              // update the CQI value
              (*it).second = params.m_cqiList.at (i).m_wbCqi.at (0);
              // update correspondent timer
              std::map <uint16_t,uint32_t>::iterator itTimers;
              itTimers = m_p10CqiTimers.find (rnti);
              (*itTimers).second = m_cqiTimersThreshold;
            }
        }
      else if ( params.m_cqiList.at (i).m_cqiType == CqiListElement_s::A30 )
        {
          // subband CQI reporting high layer configured
          // Not used by RR Scheduler
        }
      else
        {
          NS_LOG_ERROR (this << " CQI type unknown");
        }
    }

  return;
}

void
SpfRrFfMacScheduler::DoSchedUlTriggerReq (const struct FfMacSchedSapProvider::SchedUlTriggerReqParameters& params)
{
  NS_LOG_FUNCTION (this << " UL - Frame no. " << (params.m_sfnSf >> 4) << " subframe no. " << (0xF & params.m_sfnSf) << " size " << params.m_ulInfoList.size ());

  RefreshUlCqiMaps ();

  // Generate RBs map
  FfMacSchedSapUser::SchedUlConfigIndParameters ret;
  std::vector <bool> rbMap;
  uint16_t rbAllocatedNum = 0;
  std::set <uint16_t> rntiAllocated;
  std::vector <uint16_t> rbgAllocationMap;
  // update with RACH allocation map
  rbgAllocationMap = m_rachAllocationMap;
  //rbgAllocationMap.resize (m_cschedCellConfig.m_ulBandwidth, 0);
  m_rachAllocationMap.clear ();
  m_rachAllocationMap.resize (m_cschedCellConfig.m_ulBandwidth, 0);

  rbMap.resize (m_cschedCellConfig.m_ulBandwidth, false);
  // remove RACH allocation
  for (uint16_t i = 0; i < m_cschedCellConfig.m_ulBandwidth; i++)
    {
      if (rbgAllocationMap.at (i) != 0)
        {
          rbMap.at (i) = true;
          NS_LOG_DEBUG (this << " Allocated for RACH " << i);
        }
    }

  if (m_harqOn == true)
    {
      //   Process UL HARQ feedback
      for (uint16_t i = 0; i < params.m_ulInfoList.size (); i++)
        {
          if (params.m_ulInfoList.at (i).m_receptionStatus == UlInfoListElement_s::NotOk)
            {
              // retx correspondent block: retrieve the UL-DCI
              uint16_t rnti = params.m_ulInfoList.at (i).m_rnti;
              std::map <uint16_t, uint8_t>::iterator itProcId = m_ulHarqCurrentProcessId.find (rnti);
              if (itProcId == m_ulHarqCurrentProcessId.end ())
                {
                  NS_LOG_ERROR ("No info find in HARQ buffer for UE (might change eNB) " << rnti);
                }
              uint8_t harqId = (uint8_t)((*itProcId).second - HARQ_PERIOD) % HARQ_PROC_NUM;
              NS_LOG_INFO (this << " UL-HARQ retx RNTI " << rnti << " harqId " << (uint16_t)harqId);
              std::map <uint16_t, UlHarqProcessesDciBuffer_t>::iterator itHarq = m_ulHarqProcessesDciBuffer.find (rnti);
              if (itHarq == m_ulHarqProcessesDciBuffer.end ())
                {
                  NS_LOG_ERROR ("No info find in UL-HARQ buffer for UE (might change eNB) " << rnti);
                }
              UlDciListElement_s dci = (*itHarq).second.at (harqId);
              std::map <uint16_t, UlHarqProcessesStatus_t>::iterator itStat = m_ulHarqProcessesStatus.find (rnti);
              if (itStat == m_ulHarqProcessesStatus.end ())
                {
                  NS_LOG_ERROR ("No info find in HARQ buffer for UE (might change eNB) " << rnti);
                }
              if ((*itStat).second.at (harqId) > 3)
                {
                  NS_LOG_INFO ("Max number of retransmissions reached (UL)-> drop process");
                  continue;
                }
              bool free = true;
              for (int j = dci.m_rbStart; j < dci.m_rbStart + dci.m_rbLen; j++)
                {
                  if (rbMap.at (j) == true)
                    {
                      free = false;
                      NS_LOG_INFO (this << " BUSY " << j);
                    }
                }
              if (free)
                {
                  // retx on the same RBs
                  for (int j = dci.m_rbStart; j < dci.m_rbStart + dci.m_rbLen; j++)
                    {
                      rbMap.at (j) = true;
                      rbgAllocationMap.at (j) = dci.m_rnti;
                      NS_LOG_INFO ("\tRB " << j);
                      rbAllocatedNum++;
                    }
                  NS_LOG_INFO (this << " Send retx in the same RBGs " << (uint16_t)dci.m_rbStart << " to " << dci.m_rbStart + dci.m_rbLen << " RV " << (*itStat).second.at (harqId) + 1);
                }
              else
                {
                  NS_LOG_INFO ("Cannot allocate retx due to RACH allocations for UE " << rnti);
                  continue;
                }
              dci.m_ndi = 0;
              // Update HARQ buffers with new HarqId
              (*itStat).second.at ((*itProcId).second) = (*itStat).second.at (harqId) + 1;
              (*itStat).second.at (harqId) = 0;
              (*itHarq).second.at ((*itProcId).second) = dci;
              ret.m_dciList.push_back (dci);
              rntiAllocated.insert (dci.m_rnti);
            }
        }
    }

  std::map <uint16_t,uint32_t>::iterator it;
  int nflows = 0;

  for (it = m_ceBsrRxed.begin (); it != m_ceBsrRxed.end (); it++)
    {
      std::set <uint16_t>::iterator itRnti = rntiAllocated.find ((*it).first);
      // select UEs with queues not empty and not yet allocated for HARQ
      NS_LOG_INFO (this << " UE " << (*it).first << " queue " << (*it).second);
      if (((*it).second > 0)&&(itRnti == rntiAllocated.end ()))
        {
          nflows++;
        }
    }

  if (nflows == 0)
    {
      if (ret.m_dciList.size () > 0)
        {
          m_allocationMaps.insert (std::pair <uint16_t, std::vector <uint16_t> > (params.m_sfnSf, rbgAllocationMap));
          m_schedSapUser->SchedUlConfigInd (ret);
        }
      return;  // no flows to be scheduled
    }


  // Divide the remaining resources equally among the active users starting from the subsequent one served last scheduling trigger
  uint16_t rbPerFlow = (m_cschedCellConfig.m_ulBandwidth) / (nflows + rntiAllocated.size ());
  if (rbPerFlow < 3)
    {
      rbPerFlow = 3;  // at least 3 rbg per flow (till available resource) to ensure TxOpportunity >= 7 bytes
    }
  uint16_t rbAllocated = 0;

  if (m_nextRntiUl != 0)
    {
      for (it = m_ceBsrRxed.begin (); it != m_ceBsrRxed.end (); it++)
        {
          if ((*it).first == m_nextRntiUl)
            {
              break;
            }
        }
      if (it == m_ceBsrRxed.end ())
        {
          NS_LOG_ERROR (this << " no user found");
        }
    }
  else
    {
      it = m_ceBsrRxed.begin ();
      m_nextRntiUl = (*it).first;
    }
  NS_LOG_INFO (this << " NFlows " << nflows << " RB per Flow " << rbPerFlow);
  do
    {
      std::set <uint16_t>::iterator itRnti = rntiAllocated.find ((*it).first);
      if ((itRnti != rntiAllocated.end ())||((*it).second == 0))
        {
          // UE already allocated for UL-HARQ -> skip it
          it++;
          if (it == m_ceBsrRxed.end ())
            {
              // restart from the first
              it = m_ceBsrRxed.begin ();
            }
          continue;
        }
      if (rbAllocated + rbPerFlow - 1 > m_cschedCellConfig.m_ulBandwidth)
        {
          // limit to physical resources last resource assignment
          rbPerFlow = m_cschedCellConfig.m_ulBandwidth - rbAllocated;
          // at least 3 rbg per flow to ensure TxOpportunity >= 7 bytes
          if (rbPerFlow < 3)
            {
              // terminate allocation
              rbPerFlow = 0;      
            }
        }
      NS_LOG_INFO (this << " try to allocate " << (*it).first);
      UlDciListElement_s uldci;
      uldci.m_rnti = (*it).first;
      uldci.m_rbLen = rbPerFlow;
      bool allocated = false;
      NS_LOG_INFO (this << " RB Allocated " << rbAllocated << " rbPerFlow " << rbPerFlow << " flows " << nflows);
      while ((!allocated)&&((rbAllocated + rbPerFlow - m_cschedCellConfig.m_ulBandwidth) < 1) && (rbPerFlow != 0))
        {
          // check availability
          bool free = true;
          for (uint16_t j = rbAllocated; j < rbAllocated + rbPerFlow; j++)
            {
              if (rbMap.at (j) == true)
                {
                  free = false;
                  break;
                }
            }
          if (free)
            {
              uldci.m_rbStart = rbAllocated;

              for (uint16_t j = rbAllocated; j < rbAllocated + rbPerFlow; j++)
                {
                  rbMap.at (j) = true;
                  // store info on allocation for managing ul-cqi interpretation
                  rbgAllocationMap.at (j) = (*it).first;
                  NS_LOG_INFO ("\t " << j);
                }
              rbAllocated += rbPerFlow;
              allocated = true;
              break;
            }
          rbAllocated++;
          if (rbAllocated + rbPerFlow - 1 > m_cschedCellConfig.m_ulBandwidth)
            {
              // limit to physical resources last resource assignment
              rbPerFlow = m_cschedCellConfig.m_ulBandwidth - rbAllocated;
              // at least 3 rbg per flow to ensure TxOpportunity >= 7 bytes
              if (rbPerFlow < 3)
                {
                  // terminate allocation
                  rbPerFlow = 0;                 
                }
            }
        }
      if (!allocated)
        {
          // unable to allocate new resource: finish scheduling
          m_nextRntiUl = (*it).first;
          if (ret.m_dciList.size () > 0)
            {
              m_schedSapUser->SchedUlConfigInd (ret);
            }
          m_allocationMaps.insert (std::pair <uint16_t, std::vector <uint16_t> > (params.m_sfnSf, rbgAllocationMap));
          return;
        }
      std::map <uint16_t, std::vector <double> >::iterator itCqi = m_ueCqi.find ((*it).first);
      int cqi = 0;
      if (itCqi == m_ueCqi.end ())
        {
          // no cqi info about this UE
          uldci.m_mcs = 0; // MCS 0 -> UL-AMC TBD
          NS_LOG_INFO (this << " UE does not have ULCQI " << (*it).first );
        }
      else
        {
          // take the lowest CQI value (worst RB)
          double minSinr = (*itCqi).second.at (uldci.m_rbStart);
          for (uint16_t i = uldci.m_rbStart; i < uldci.m_rbStart + uldci.m_rbLen; i++)
            {
              if ((*itCqi).second.at (i) < minSinr)
                {
                  minSinr = (*itCqi).second.at (i);
                }
            }
          // translate SINR -> cqi: WILD ACK: same as DL
          double s = log2 ( 1 + (
                                 std::pow (10, minSinr / 10 )  /
                                 ( (-std::log (5.0 * 0.00005 )) / 1.5) ));


          cqi = m_amc->GetCqiFromSpectralEfficiency (s);
          if (cqi == 0)
            {
              it++;
              if (it == m_ceBsrRxed.end ())
                {
                  // restart from the first
                  it = m_ceBsrRxed.begin ();
                }
              NS_LOG_DEBUG (this << " UE discared for CQI=0, RNTI " << uldci.m_rnti);
              // remove UE from allocation map
              for (uint16_t i = uldci.m_rbStart; i < uldci.m_rbStart + uldci.m_rbLen; i++)
                {
                  rbgAllocationMap.at (i) = 0;
                }
              continue; // CQI == 0 means "out of range" (see table 7.2.3-1 of 36.213)
            }
          uldci.m_mcs = m_amc->GetMcsFromCqi (cqi);
        }
      uldci.m_tbSize = (m_amc->GetTbSizeFromMcs (uldci.m_mcs, rbPerFlow) / 8); // MCS 0 -> UL-AMC TBD

      UpdateUlRlcBufferInfo (uldci.m_rnti, uldci.m_tbSize);
      uldci.m_ndi = 1;
      uldci.m_cceIndex = 0;
      uldci.m_aggrLevel = 1;
      uldci.m_ueTxAntennaSelection = 3; // antenna selection OFF
      uldci.m_hopping = false;
      uldci.m_n2Dmrs = 0;
      uldci.m_tpc = 0; // no power control
      uldci.m_cqiRequest = false; // only period CQI at this stage
      uldci.m_ulIndex = 0; // TDD parameter
      uldci.m_dai = 1; // TDD parameter
      uldci.m_freqHopping = 0;
      uldci.m_pdcchPowerOffset = 0; // not used
      ret.m_dciList.push_back (uldci);
      // store DCI for HARQ_PERIOD
      uint8_t harqId = 0;
      if (m_harqOn == true)
        {
          std::map <uint16_t, uint8_t>::iterator itProcId;
          itProcId = m_ulHarqCurrentProcessId.find (uldci.m_rnti);
          if (itProcId == m_ulHarqCurrentProcessId.end ())
            {
              NS_FATAL_ERROR ("No info find in HARQ buffer for UE " << uldci.m_rnti);
            }
          harqId = (*itProcId).second;
          std::map <uint16_t, UlHarqProcessesDciBuffer_t>::iterator itDci = m_ulHarqProcessesDciBuffer.find (uldci.m_rnti);
          if (itDci == m_ulHarqProcessesDciBuffer.end ())
            {
              NS_FATAL_ERROR ("Unable to find RNTI entry in UL DCI HARQ buffer for RNTI " << uldci.m_rnti);
            }
          (*itDci).second.at (harqId) = uldci;
          // Update HARQ process status (RV 0)
          std::map <uint16_t, UlHarqProcessesStatus_t>::iterator itStat = m_ulHarqProcessesStatus.find (uldci.m_rnti);
          if (itStat == m_ulHarqProcessesStatus.end ())
            {
              NS_LOG_ERROR ("No info find in HARQ buffer for UE (might change eNB) " << uldci.m_rnti);
            }
          (*itStat).second.at (harqId) = 0;
        }
        
      NS_LOG_INFO (this << " UL Allocation - UE " << (*it).first << " startPRB " << (uint32_t)uldci.m_rbStart << " nPRB " << (uint32_t)uldci.m_rbLen << " CQI " << cqi << " MCS " << (uint32_t)uldci.m_mcs << " TBsize " << uldci.m_tbSize << " harqId " << (uint16_t)harqId);

      it++;
      if (it == m_ceBsrRxed.end ())
        {
          // restart from the first
          it = m_ceBsrRxed.begin ();
        }
      if ((rbAllocated == m_cschedCellConfig.m_ulBandwidth) || (rbPerFlow == 0))
        {
          // Stop allocation: no more PRBs
          m_nextRntiUl = (*it).first;
          break;
        }
    }
  while (((*it).first != m_nextRntiUl)&&(rbPerFlow!=0));

  m_allocationMaps.insert (std::pair <uint16_t, std::vector <uint16_t> > (params.m_sfnSf, rbgAllocationMap));

  m_schedSapUser->SchedUlConfigInd (ret);
  return;
}

void
SpfRrFfMacScheduler::DoSchedUlNoiseInterferenceReq (const struct FfMacSchedSapProvider::SchedUlNoiseInterferenceReqParameters& params)
{
  NS_LOG_FUNCTION (this);
  return;
}

void
SpfRrFfMacScheduler::DoSchedUlSrInfoReq (const struct FfMacSchedSapProvider::SchedUlSrInfoReqParameters& params)
{
  NS_LOG_FUNCTION (this);
  return;
}

void
SpfRrFfMacScheduler::DoSchedUlMacCtrlInfoReq (const struct FfMacSchedSapProvider::SchedUlMacCtrlInfoReqParameters& params)
{
  NS_LOG_FUNCTION (this);

  std::map <uint16_t,uint32_t>::iterator it;

  for (unsigned int i = 0; i < params.m_macCeList.size (); i++)
    {
      if ( params.m_macCeList.at (i).m_macCeType == MacCeListElement_s::BSR )
        {
          // buffer status report
          // note that this scheduler does not differentiate the
          // allocation according to which LCGs have more/less bytes
          // to send.
          // Hence the BSR of different LCGs are just summed up to get
          // a total queue size that is used for allocation purposes.

          uint32_t buffer = 0;
          for (uint8_t lcg = 0; lcg < 4; ++lcg)
            {
              uint8_t bsrId = params.m_macCeList.at (i).m_macCeValue.m_bufferStatus.at (lcg);
              buffer += BufferSizeLevelBsr::BsrId2BufferSize (bsrId);
            }

          uint16_t rnti = params.m_macCeList.at (i).m_rnti;
          it = m_ceBsrRxed.find (rnti);
          if (it == m_ceBsrRxed.end ())
            {
              // create the new entry
              m_ceBsrRxed.insert ( std::pair<uint16_t, uint32_t > (rnti, buffer));
              NS_LOG_INFO (this << " Insert RNTI " << rnti << " queue " << buffer);
            }
          else
            {
              // update the buffer size value
              (*it).second = buffer;
              NS_LOG_INFO (this << " Update RNTI " << rnti << " queue " << buffer);
            }
        }
    }

  return;
}

void
SpfRrFfMacScheduler::DoSchedUlCqiInfoReq (const struct FfMacSchedSapProvider::SchedUlCqiInfoReqParameters& params)
{
  NS_LOG_FUNCTION (this);

  switch (m_ulCqiFilter)
    {
    case FfMacScheduler::SRS_UL_CQI:
      {
        // filter all the CQIs that are not SRS based
        if (params.m_ulCqi.m_type != UlCqi_s::SRS)
          {
            return;
          }
      }
      break;
    case FfMacScheduler::PUSCH_UL_CQI:
      {
        // filter all the CQIs that are not SRS based
        if (params.m_ulCqi.m_type != UlCqi_s::PUSCH)
          {
            return;
          }
      }
    case FfMacScheduler::ALL_UL_CQI:
      break;

    default:
      NS_FATAL_ERROR ("Unknown UL CQI type");
    }
  switch (params.m_ulCqi.m_type)
    {
    case UlCqi_s::PUSCH:
      {
        std::map <uint16_t, std::vector <uint16_t> >::iterator itMap;
        std::map <uint16_t, std::vector <double> >::iterator itCqi;
        itMap = m_allocationMaps.find (params.m_sfnSf);
        if (itMap == m_allocationMaps.end ())
          {
            NS_LOG_INFO (this << " Does not find info on allocation, size : " << m_allocationMaps.size ());
            return;
          }
        for (uint32_t i = 0; i < (*itMap).second.size (); i++)
          {
            // convert from fixed point notation Sxxxxxxxxxxx.xxx to double
            double sinr = LteFfConverter::fpS11dot3toDouble (params.m_ulCqi.m_sinr.at (i));
            itCqi = m_ueCqi.find ((*itMap).second.at (i));
            if (itCqi == m_ueCqi.end ())
              {
                // create a new entry
                std::vector <double> newCqi;
                for (uint32_t j = 0; j < m_cschedCellConfig.m_ulBandwidth; j++)
                  {
                    if (i == j)
                      {
                        newCqi.push_back (sinr);
                      }
                    else
                      {
                        // initialize with NO_SINR value.
                        newCqi.push_back (30.0);
                      }

                  }
                m_ueCqi.insert (std::pair <uint16_t, std::vector <double> > ((*itMap).second.at (i), newCqi));
                // generate correspondent timer
                m_ueCqiTimers.insert (std::pair <uint16_t, uint32_t > ((*itMap).second.at (i), m_cqiTimersThreshold));
              }
            else
              {
                // update the value
                (*itCqi).second.at (i) = sinr;
                // update correspondent timer
                std::map <uint16_t, uint32_t>::iterator itTimers;
                itTimers = m_ueCqiTimers.find ((*itMap).second.at (i));
                (*itTimers).second = m_cqiTimersThreshold;

              }

          }
        // remove obsolete info on allocation
        m_allocationMaps.erase (itMap);
      }
      break;
    case UlCqi_s::SRS:
      {
        // get the RNTI from vendor specific parameters
        uint16_t rnti = 0;
        NS_ASSERT (params.m_vendorSpecificList.size () > 0);
        for (uint16_t i = 0; i < params.m_vendorSpecificList.size (); i++)
          {
            if (params.m_vendorSpecificList.at (i).m_type == SRS_CQI_RNTI_VSP)
              {
                Ptr<SrsCqiRntiVsp> vsp = DynamicCast<SrsCqiRntiVsp> (params.m_vendorSpecificList.at (i).m_value);
                rnti = vsp->GetRnti ();
              }
          }
        std::map <uint16_t, std::vector <double> >::iterator itCqi;
        itCqi = m_ueCqi.find (rnti);
        if (itCqi == m_ueCqi.end ())
          {
            // create a new entry
            std::vector <double> newCqi;
            for (uint32_t j = 0; j < m_cschedCellConfig.m_ulBandwidth; j++)
              {
                double sinr = LteFfConverter::fpS11dot3toDouble (params.m_ulCqi.m_sinr.at (j));
                newCqi.push_back (sinr);
                NS_LOG_INFO (this << " RNTI " << rnti << " new SRS-CQI for RB  " << j << " value " << sinr);

              }
            m_ueCqi.insert (std::pair <uint16_t, std::vector <double> > (rnti, newCqi));
            // generate correspondent timer
            m_ueCqiTimers.insert (std::pair <uint16_t, uint32_t > (rnti, m_cqiTimersThreshold));
          }
        else
          {
            // update the values
            for (uint32_t j = 0; j < m_cschedCellConfig.m_ulBandwidth; j++)
              {
                double sinr = LteFfConverter::fpS11dot3toDouble (params.m_ulCqi.m_sinr.at (j));
                (*itCqi).second.at (j) = sinr;
                NS_LOG_INFO (this << " RNTI " << rnti << " update SRS-CQI for RB  " << j << " value " << sinr);
              }
            // update correspondent timer
            std::map <uint16_t, uint32_t>::iterator itTimers;
            itTimers = m_ueCqiTimers.find (rnti);
            (*itTimers).second = m_cqiTimersThreshold;

          }


      }
      break;
    case UlCqi_s::PUCCH_1:
    case UlCqi_s::PUCCH_2:
    case UlCqi_s::PRACH:
      {
        NS_FATAL_ERROR ("PfFfMacScheduler supports only PUSCH and SRS UL-CQIs");
      }
      break;
    default:
      NS_FATAL_ERROR ("Unknown type of UL-CQI");
    }
  return;
}


void
SpfRrFfMacScheduler::RefreshDlCqiMaps (void)
{
  NS_LOG_FUNCTION (this << m_p10CqiTimers.size ());
  // refresh DL CQI P01 Map
  std::map <uint16_t,uint32_t>::iterator itP10 = m_p10CqiTimers.begin ();
  while (itP10 != m_p10CqiTimers.end ())
    {
      NS_LOG_INFO (this << " P10-CQI for user " << (*itP10).first << " is " << (uint32_t)(*itP10).second << " thr " << (uint32_t)m_cqiTimersThreshold);
      if ((*itP10).second == 0)
        {
          // delete correspondent entries
          std::map <uint16_t,uint8_t>::iterator itMap = m_p10CqiRxed.find ((*itP10).first);
          NS_ASSERT_MSG (itMap != m_p10CqiRxed.end (), " Does not find CQI report for user " << (*itP10).first);
          NS_LOG_INFO (this << " P10-CQI exired for user " << (*itP10).first);
          m_p10CqiRxed.erase (itMap);
          std::map <uint16_t,uint32_t>::iterator temp = itP10;
          itP10++;
          m_p10CqiTimers.erase (temp);
        }
      else
        {
          (*itP10).second--;
          itP10++;
        }
    }

  return;
}


void
SpfRrFfMacScheduler::RefreshUlCqiMaps (void)
{
  // refresh UL CQI  Map
  std::map <uint16_t,uint32_t>::iterator itUl = m_ueCqiTimers.begin ();
  while (itUl != m_ueCqiTimers.end ())
    {
      NS_LOG_INFO (this << " UL-CQI for user " << (*itUl).first << " is " << (uint32_t)(*itUl).second << " thr " << (uint32_t)m_cqiTimersThreshold);
      if ((*itUl).second == 0)
        {
          // delete correspondent entries
          std::map <uint16_t, std::vector <double> >::iterator itMap = m_ueCqi.find ((*itUl).first);
          NS_ASSERT_MSG (itMap != m_ueCqi.end (), " Does not find CQI report for user " << (*itUl).first);
          NS_LOG_INFO (this << " UL-CQI exired for user " << (*itUl).first);
          (*itMap).second.clear ();
          m_ueCqi.erase (itMap);
          std::map <uint16_t,uint32_t>::iterator temp = itUl;
          itUl++;
          m_ueCqiTimers.erase (temp);
        }
      else
        {
          (*itUl).second--;
          itUl++;
        }
    }

  return;
}

void
SpfRrFfMacScheduler::UpdateDlRlcBufferInfo (uint16_t rnti, uint8_t lcid, uint16_t size)
{
  NS_LOG_FUNCTION (this);
  std::list<FfMacSchedSapProvider::SchedDlRlcBufferReqParameters>::iterator it;
  for (it = m_rlcBufferReq.begin (); it != m_rlcBufferReq.end (); it++)
    {
      if (((*it).m_rnti == rnti) && ((*it).m_logicalChannelIdentity == lcid))
        {
          NS_LOG_INFO (this << " UE " << rnti << " LC " << (uint16_t)lcid << " txqueue " << (*it).m_rlcTransmissionQueueSize << " retxqueue " << (*it).m_rlcRetransmissionQueueSize << " status " << (*it).m_rlcStatusPduSize << " decrease " << size);
          // Update queues: RLC tx order Status, ReTx, Tx
          // Update status queue
           if (((*it).m_rlcStatusPduSize > 0) && (size >= (*it).m_rlcStatusPduSize))
              {
                (*it).m_rlcStatusPduSize = 0;
              }
            else if (((*it).m_rlcRetransmissionQueueSize > 0) && (size >= (*it).m_rlcRetransmissionQueueSize))
              {
                (*it).m_rlcRetransmissionQueueSize = 0;
              }
            else if ((*it).m_rlcTransmissionQueueSize > 0)
              {
                uint32_t rlcOverhead;
                if (lcid == 1)
                  {
                    // for SRB1 (using RLC AM) it's better to
                    // overestimate RLC overhead rather than
                    // underestimate it and risk unneeded
                    // segmentation which increases delay 
                    rlcOverhead = 4;                                  
                  }
                else
                  {
                    // minimum RLC overhead due to header
                    rlcOverhead = 2;
                  }
                // update transmission queue
                if ((*it).m_rlcTransmissionQueueSize <= size - rlcOverhead)
                  {
                    (*it).m_rlcTransmissionQueueSize = 0;
                  }
                else
                  {                    
                    (*it).m_rlcTransmissionQueueSize -= size - rlcOverhead;
                  }
              }
          return;
        }
    }
}

void
SpfRrFfMacScheduler::UpdateUlRlcBufferInfo (uint16_t rnti, uint16_t size)
{

  size = size - 2; // remove the minimum RLC overhead
  std::map <uint16_t,uint32_t>::iterator it = m_ceBsrRxed.find (rnti);
  if (it != m_ceBsrRxed.end ())
    {
      NS_LOG_INFO (this << " Update RLC BSR UE " << rnti << " size " << size << " BSR " << (*it).second);
      if ((*it).second >= size)
        {
          (*it).second -= size;
        }
      else
        {
          (*it).second = 0;
        }
    }
  else
    {
      NS_LOG_ERROR (this << " Does not find BSR report info of UE " << rnti);
    }

}


void
SpfRrFfMacScheduler::TransmissionModeConfigurationUpdate (uint16_t rnti, uint8_t txMode)
{
  NS_LOG_FUNCTION (this << " RNTI " << rnti << " txMode " << (uint16_t)txMode);
  FfMacCschedSapUser::CschedUeConfigUpdateIndParameters params;
  params.m_rnti = rnti;
  params.m_transmissionMode = txMode;
  m_cschedSapUser->CschedUeConfigUpdateInd (params);
}



}
