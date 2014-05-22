#include "../Common.h"
#include "StorageVolumes.h"

configuration NodeDcMotorAppC { }
implementation
{
  components MainC, LedsC, NodeDcMotorC, ActiveMessageC;
  components HplMsp430GeneralIOC;
  components new AMSenderC(AM_NODEOPTENC);
  components new AMSnooperC(AM_NODEOPTENC);
  components new TimerMilliC() as Timer1;
  components new LogStorageC(VOLUME_LOGTEST,FALSE);
  
  NodeDcMotorC.Boot -> MainC;
  NodeDcMotorC.Leds -> LedsC;
  NodeDcMotorC.LogTimer -> Timer1;

  //radio chip wiring
  NodeDcMotorC.Receive -> AMSnooperC;
  NodeDcMotorC.AMControl -> ActiveMessageC;
  NodeDcMotorC.AMSend -> AMSenderC;
  NodeDcMotorC.Ack       -> AMSenderC; 
  NodeDcMotorC.Packet -> AMSenderC;
  NodeDcMotorC.AMPacket -> ActiveMessageC;

  //wire data logging
  NodeDcMotorC.LogWrite -> LogStorageC;
  NodeDcMotorC.LogRead -> LogStorageC;
}
