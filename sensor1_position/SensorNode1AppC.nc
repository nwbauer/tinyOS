
#include "LS7366R.h"
#include "printf.h"
#include "NodeOptEnc.h"

configuration SensorNodeAppC {
}
implementation {
  
  components MainC,ActiveMessageC;

  components new TimerMilliC() as Timer;
  components new AMSenderC(AM_NODEOPTENC);
  components SensorNodeC as Enc;

  Enc.TimerSamples -> Timer;
  Enc.Boot -> MainC;
  
  components LS7366RControlC;
  
  Enc.EncConfig -> LS7366RControlC;
  Enc.EncReceive -> LS7366RControlC;
  Enc.EncResource -> LS7366RControlC.Resource;
  
  components LedsC;
  Enc.Leds -> LedsC;
  
  //radio chip wiring
  
  //HIL
  Enc.Packet -> AMSenderC;
  //Enc.AMPacket -> AMSenderC;
  Enc.AMSend -> AMSenderC;
  Enc.AMControl -> ActiveMessageC;
  Enc.Ack       -> AMSenderC;
  //Enc.CcaOverride -> RadioBackoffC;
  
  //HAL
  //Enc.Packet -> CC2420ActiveMessageC;
  //Enc.AMControl -> CC2420ActiveMessageC;
  //Enc.AMSend -> CC2420ActiveMessageC.AMSend[AM_NODEOPTENC];
  
}
