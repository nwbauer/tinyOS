#ifndef COMMON_H
#define COMMON_H

enum{
  BACKOFF_TIME   = 25,
  JAMMER_TX_TIME = 20,
  AM_NODEOPTENC  = 6,
  AM_JAMMER      = 5,
  NUM_SAMPLES    = 1000
}; 

typedef nx_struct NodeOptEncMsg{
  nx_uint8_t  nodeid;
  nx_uint16_t pktNr;
  nx_uint16_t edgeCount;
}NodeOptEncMsg;


#endif
