
#include "LS7366R.h"
#include "printf.h"
#include "Timer.h"
#include "../Common.h"

module SensorNodeC{
  
  uses {
    interface Boot;
    
    interface LS7366RConfig as EncConfig;
    interface LS7366RReceive as EncReceive;
    interface Resource as EncResource;
    interface Leds;
    
    interface Timer<TMilli> as TimerSamples;
    
    
    interface Packet;
    interface AMPacket;
    interface AMSend;
    interface Receive;
    interface SplitControl as AMControl;
    interface PacketAcknowledgements as Ack;
    //interface RadioBackoff as CcaOverride;
  }
}

implementation {
	
  uint8_t count[4];
  uint16_t edges;
  uint16_t msgNr=0;
  bool busy = FALSE;
  message_t pkt;



  /************************************************************/
  /************************************************************/
  /*Boot Code*/

  event void Boot.booted() {
    //turn on radio
    call AMControl.start();
  }

  /************************************************************/
  /************************************************************/
  /*Encoder Code*/
  
  event void TimerSamples.fired(){
	//when timer fires, sample optical encoder value
	call EncResource.request();
  }
  
  event void EncReceive.receiveDone( uint8_t* data ) {
    //value received from optical encoder, now send it to the controller
	
	if(!busy){
		//create message to be sent to controller
		NodeOptEncMsg* nocpkt =(NodeOptEncMsg*)(call Packet.getPayload(&pkt, sizeof(NodeOptEncMsg)));
		
		//add message number to message to detect if packet loss occurs
		nocpkt -> pktNr = msgNr++;
		
		//add number of edges by optical encoder to message for control
		edges = 256*data[0]+data[1];
		nocpkt -> edgeCount = edges;
		
		call Leds.set(edges);
		call Ack.requestAck(&pkt);
		
		//send message to controller
		if(call AMSend.send(1,&pkt,sizeof(NodeOptEncMsg))==SUCCESS){
			busy=TRUE;
			call Leds.led1On();
		}
	}
  }  
  
  event void EncResource.granted() {
    call EncReceive.receive(count);
  }
  
  event void EncConfig.syncDone( error_t error ) {}
  
  
  /************************************************************/
  /************************************************************/
  /*Radio Code*/

  event void AMControl.startDone(error_t err){
    if(err == SUCCESS){
      call Leds.set(TOS_NODE_ID);
    }else
      call AMControl.start();
  }

  event void AMControl.stopDone(error_t err){}
  
  event void AMSend.sendDone(message_t* msg, error_t error){
    
	if(&pkt == msg){
		busy = FALSE;
	}
	if(call Ack.wasAcked(msg)){
		// do something here if acked
	}else{
		//no ACK, so re-sample and re-send
		call Leds.led0Toggle();
		call TimerSamples.startOneShot(BACKOFF_TIME);
	}
  }

  event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len){

	if(call AMPacket.isForMe(msg)){
		// this node's turn to sample and transmit data
		call TimerSamples.startOneShot(BACKOFF_TIME);
	}
		call Leds.led1Off();
		return msg;
	}

}
