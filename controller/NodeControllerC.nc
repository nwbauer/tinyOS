//#include "timer.h"
#include "printf.h"
#include "down25.h"

module NodeControllerC {
	uses {
		interface Boot;
		interface Leds;
		interface Timer<TMilli> as LogTimer;

		interface Packet;
		interface AMPacket;
		interface AMSend;
		interface Receive;
		interface SplitControl as AMControl;
		interface PacketAcknowledgements as Ack;

		//data logging
		interface LogRead;
		interface LogWrite;
	}
}

implementation {
  
  enum{
    Dac0Common = 2047,
    Dac1Common = 2040,
    NUM_LOGBINS = 125,
    SIZE_LOGBINS = 2 
  };

  float xD[8]={0,0,0,0,0,0,0,0},y[2]={0,0},u=0;
  uint16_t msgNr=0, compDelay, uImplementedTime, curMsgTime;
  uint16_t ctr=0, uOut=0,ctr2=0;
  int16_t edges;
  uint8_t node,delay,currBin=0,currPos=0,nextLogged=0;
  storage_cookie_t START_DATA;
  message_t pkt;
  error_t e;
  bool m_busy;

  typedef struct logentry_t{
	uint16_t recNum[SIZE_LOGBINS];
	uint8_t  sigma[SIZE_LOGBINS];
	uint16_t msgNr[SIZE_LOGBINS];
	uint16_t time[SIZE_LOGBINS];    
	uint16_t compDelay[SIZE_LOGBINS];
	uint8_t transDelay[SIZE_LOGBINS];
	float angle[SIZE_LOGBINS];
	float pos[SIZE_LOGBINS];
	float x1[SIZE_LOGBINS];
	float x2[SIZE_LOGBINS];
	float u[SIZE_LOGBINS];
  }logentry_t;

  logentry_t m_store[NUM_LOGBINS];

  /************************************************************/
  /************************************************************/
  /*Boot code*/


  event void Boot.booted() {
    // configure DAC once radio is turned on
    atomic{
      ADC12CTL0 = REF2_5V+REFON;
      DAC12_0CTL = DAC12IR+DAC12AMP_5+DAC12ENC+DAC12GRP+DAC12LSEL_1;
      DAC12_1CTL = DAC12IR+DAC12AMP_5+DAC12ENC+DAC12LSEL_1;
      DAC12_0DAT = Dac0Common;
      DAC12_1DAT = Dac1Common;
    }

    //erase the last flash memory log
    call LogWrite.erase();
  }


  event void LogWrite.eraseDone(error_t error){
	START_DATA = call LogWrite.currentOffset();

	//turn the radio on once the last log is erased
	call AMControl.start();     
  }

  /************************************************************/
  /************************************************************/
  /*Radio code*/

  event void AMControl.startDone(error_t err){
	if(err != SUCCESS){
		call AMControl.start();
	}	
	call Ack.requestAck(&pkt);
	//Send a package to node 1 to initilize protocol
	call AMSend.send(1,&pkt,sizeof(NodeOptEncMsg));
  }


  event void AMControl.stopDone(error_t err){}

  event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len){   
	//main code - executed every time a packet is received

    if(len==sizeof(NodeOptEncMsg)){
		
		curMsgTime = call LogTimer.getNow();
     
	 //--------------------------------------------
      // break apart the payload of the message
	  
	    NodeOptEncMsg* nocpkt =(NodeOptEncMsg*)payload;
		node = call AMPacket.source(msg);
		delay  = nocpkt -> delay;
		msgNr = nocpkt -> pktNr;
		edges = nocpkt -> edgeCount;
      
      //--------------------------------------------
      // calculate reception interval

		transIntvl = curMsgTime-lastMsgTime;
		lastMsgTime = curMsgTime;
            
      //--------------------------------------------
      //check for dropped packages
      
		if(lastMsg == 0){       //initialize lastMsg
			lastMsg = msgNr-1;
		}

		if(++lastMsg != msgNr){
			call Leds.led0Toggle();
			drops=drops + msgNr-lastMsg;
		}

		//lastMsg = msgNr;

      //----------------------------------------------
      //control law goes here!
      
		if(node == 1){
			call Leds.led0Toggle();
			y[0] = -edges*0.000022750;   // ticks to meters, taken from Quanser data sheet
		}else if(node ==2){
			call Leds.led1Toggle();
			y[1] = -edges*0.0015;      // ticks to radians, taken from Quanser data sheet
		}

		// observer-based controller
      
		u=-(xD[0]*K[0]+xD[1]*K[1]+xD[2]*K[2]+xD[3]*K[3]);      

		xD[4]=A[0]*xD[0]+A[4]*xD[1]+A[8]*xD[2]+A[12]*xD[3]  +B[0]*u  +L[0]*(y[0]-xD[0])+L[4]*(y[1]-xD[1]); 
		xD[5]=A[1]*xD[0]+A[5]*xD[1]+A[9]*xD[2]+A[13]*xD[3]  +B[1]*u  +L[1]*(y[0]-xD[0])+L[5]*(y[1]-xD[1]); 
		xD[6]=A[2]*xD[0]+A[6]*xD[1]+A[10]*xD[2]+A[14]*xD[3] +B[2]*u  +L[2]*(y[0]-xD[0])+L[6]*(y[1]-xD[1]); 
		xD[7]=A[3]*xD[0]+A[7]*xD[1]+A[11]*xD[2]+A[15]*xD[3] +B[3]*u  +L[3]*(y[0]-xD[0])+L[7]*(y[1]-xD[1]);

		xD[0]=xD[4];
		xD[1]=xD[5];
		xD[2]=xD[6];
		xD[3]=xD[7];

      //---------------------------------------------------
      // Convert u from volts to DAC integer value 
      // Facor 3 due to amplifier gain

		if(u>3*2.450){
			uOut=2038;
		}else if(u< -3*2.450){
			uOut=-2038;
		}else{
			uOut=(2038/(3*2.450))*u;
		}

		// set differential input (u=0 -> DAC0 = Dac0Common, DAC1 = Dac1Common)
		// common voltage should be around 1.25V
		DAC12_0DAT = Dac0Common+uOut;
		DAC12_1DAT = Dac1Common-uOut;

      //---------------------------------------------------
      // Stamp computation time for analysis 
	  
	    uImplementedTime  = call LogTimer.getNow();     
		compDelay = uImplementedTime-curMsgTime;      


      //----------------------------
      //Log Data into Flash
      //Store analysis data in an array in memory and load into flash when possible
	  
		if(ctr++<NUM_SAMPLES){
			// Experiment running
			
			m_store[currBin].recNum[currPos] 		= ctr;
			m_store[currBin].sigma[currPos] 		= node;
			m_store[currBin].msgNr[currPos] 		= msgNr;
			m_store[currBin].transDelay[currPos] 	= delay;
			m_store[currBin].angle[currPos] 		= y[1];
			m_store[currBin].pos[currPos] 			= y[0];
			m_store[currBin].x1[currPos] 				= xD[0];
			m_store[currBin].x2[currPos] 				= xD[1];
			m_store[currBin].u[currPos] 				= u;
			m_store[currBin].time[currPos] 			= curMsgTime;
			m_store[currBin].compDelay[currPos] = compDelay;    
				
			if(currPos==SIZE_LOGBINS-1){     //if at bottom of bin
				currPos = 0;              
				if(currBin==NUM_LOGBINS-1){
					currBin = 0;	//wrap around
				}else{
					currBin++;
				}
			}else{
			currPos++;
			}

			// Save the bins into flash memory
			if(nextLogged!=currBin){
				if(!m_busy){
					m_busy=TRUE;
					if(call LogWrite.append(&(m_store[nextLogged]),sizeof(m_store[nextLogged]))!=SUCCESS){
						m_busy=FALSE;
						//call Leds.set(1);  
					}else{
						//call Leds.set(2);
						if(nextLogged==NUM_LOGBINS-1)
							nextLogged=0;	//wrap around
						else
							nextLogged++;
					}
				}else{
					call Leds.set(4);
				}
			}    
			
		}else{
			// Experiment has ended
			
			call Leds.set(3);
			call AMControl.stop();
			DAC12_0DAT = Dac0Common;
			DAC12_1DAT = Dac1Common;
			call LogTimer.startOneShot(15);
		}
	}
	return msg;
  }



  event void AMSend.sendDone(message_t* msg, error_t error){
	//check to maybe resend a packet to one of the sensor devices to start the control system
	
	if(call Ack.wasAcked(msg)){
		//Protocol should be started
		call Leds.led2On();
	}else{
		//Resend a package to node 1 to initialize protocol
		call AMSend.send(1,&pkt,sizeof(NodeOptEncMsg));
	}
  }
  
  

  /************************************************************/
  /************************************************************/
  /*Data Logging code*/
  /*Code is ran after experiment is completed to extract data from flash */

  // no native way to print type float in tinyOS printf library
  void printfFloat(float toBePrinted) {
	uint32_t fi,f0,f1,f2;
	char c;
	float f = toBePrinted;

	if(f<0){
		c = '-'; 
		f = -f;
	}else{
		c= ' ';
	}
	//integer portion
	fi=(uint32_t) f;

	// decimal portion ... 3 decimal places
	f=f-((float) fi);
	f0= f*10;
	f0 %= 10;
	f1 = f*100;
	f1 %= 10;
	f2 = f*1000;
	f2 %= 10;
	printf("%c%ld.%d%d%d",c,fi,(uint8_t) f0, (uint8_t) f1, (uint8_t) f2);
  } 
  
 
  event void LogTimer.fired(){
	if(ctr++<NUM_SAMPLES/SIZE_LOGBINS-1){
		call LogRead.read(&m_entry,sizeof(m_entry));
	}else{
		printf("END DATA\n");
		printfflush();
		call LogRead.seek(START_DATA);
	}

  }

  event void LogRead.readDone(void* buf, storage_len_t len, error_t error){
	
	//print out the data saved in the flash memory, comma delimited
	for(ctr2=0;ctr2<SIZE_LOGBINS;ctr2++){
		printf("%d,%d,%d,%d,%d,%d,",m_entry.recNum[ctr2],m_entry.time[ctr2],m_entry.sigma[ctr2],m_entry.msgNr[ctr2],m_entry.compDelay[ctr2],m_entry.transDelay[ctr2]);
		printfFloat(m_entry.angle[ctr2]);
		printf(",");
		printfFloat(m_entry.pos[ctr2]);
		printf(",");
		printfFloat(m_entry.x1[ctr2]);
		printf(",");
		printfFloat(m_entry.x2[ctr2]);
		printf(",");
		printfFloat(m_entry.u[ctr2]);
		printf("\n");
		printfflush();
		call LogTimer.startOneShot(15);
	}
  }

  event void LogRead.seekDone(error_t error){
	ctr=0;
	printf("BEGIN DATA:\n");
	printfflush();
	call LogRead.read(&m_entry,sizeof(m_entry));
  }

  event void LogWrite.appendDone(void* buf, storage_len_t len, bool recordsLost, error_t error){ 
	m_busy=FALSE;
  }

  event void LogWrite.syncDone(error_t error){}

} 


