/*
 Copyright (C) 2011 James Coliz, Jr. <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */
#include "RF24Network_config.h"
#include "RF24Network.h"
#include "mcu_config.hpp"


#if defined (ENABLE_SLEEP_MODE) && !defined (RF24_LINUX) && !defined (__ARDUINO_X86__)
	volatile byte sleep_cycles_remaining;
	volatile bool wasInterrupted;
#endif

uint16_t RF24NetworkHeader::next_id = 1;
#if defined ENABLE_NETWORK_STATS
uint32_t RF24Network::nFails = 0;
uint32_t RF24Network::nOK = 0;
#endif
void pipe_address( uint16_t node, uint8_t pipe, uint8_t* return_address );
#if defined (RF24NetworkMulticast)
uint16_t levelToAddress( uint8_t level );
#endif


constexpr uint8_t rf24_min(uint8_t x,uint8_t y) { return (x < y) ?  x :  y; }
uint8_t RF24Network::frame_buffer[32] = {0};
uint8_t RF24Network::frame_queue[MAIN_BUFFER_SIZE] = {0};
uint16_t RF24Network::node_address = 0;
uint8_t* RF24Network::next_frame = frame_queue;
uint8_t RF24Network::frame_size;

uint16_t RF24Network::parent_node; /**< Our parent's node address */
uint8_t RF24Network::parent_pipe; /**< The pipe our parent uses to listen to us */
uint16_t RF24Network::node_mask;

uint8_t RF24Network::networkFlags = 0;

uint32_t RF24Network::txTime=0;
bool RF24Network::returnSysMsgs = false;
bool RF24Network::multicastRelay = false;
#if defined (RF24NetworkMulticast)
uint8_t RF24Network::multicast_level;
#endif


/******************************************************************/
void RF24Network::begin(uint16_t _node_address )
{

  if (! is_valid_address(_node_address) )
    return;


  node_address = _node_address;
  next_frame = &frame_queue[0];


  // Set up the radio the way we want it to look


  radio::disableAutoAck(Nrf24Pipe::RX_PIPE0);

 //dnamic payloads enabled through template
  
  // Use different retry periods to reduce data collisions
  uint8_t retryVar = (((node_address % 6)+1) *2) + 3;
  radio::setAutoRetransmit(retryVar, 5);

  //txTimeout = 25;
  //routeTimeout = txTimeout*3; // Adjust for max delay per node within a single chain

  setup_address();

  uint8_t pipe[5];
  // Open up all listening pipes
  for( uint8_t i = static_cast<uint8_t>(Nrf24Pipe::RX_PIPE5); i >= static_cast<uint8_t>(Nrf24Pipe::RX_PIPE0); --i){

	pipe_address(_node_address,i-static_cast<uint8_t>(Nrf24Pipe::RX_PIPE0),pipe);
    (void)radio::openPipe(static_cast<Nrf24Pipe>(i),pipe);
  }
  radio::enableRx();

  // Setup our address helper cache


}

/******************************************************************/

#if defined ENABLE_NETWORK_STATS
void RF24Network::failures(uint32_t *_fails, uint32_t *_ok){
	*_fails = nFails;
	*_ok = nOK;
}
#endif

/******************************************************************/

uint8_t RF24Network::update(void)
{
  // if there is data ready
  Nrf24Pipe pipe_num;
  uint8_t returnVal = 0;
  // If bypass is enabled, continue although incoming user data may be dropped
  // Allows system payloads to be read while user cache is full
  // Incoming Hold prevents data from being read from the radio, preventing incoming payloads from being acked
  
  #if !defined (RF24_LINUX)
  if(!(networkFlags & FLAG_BYPASS_HOLDS)){
    if( (networkFlags & FLAG_HOLD_INCOMING) || (next_frame-frame_queue) + 34 > MAIN_BUFFER_SIZE ){
      if(!available()){
        networkFlags &= ~FLAG_HOLD_INCOMING;
      }else{
        return 0;
      }
    }
  }
  #endif
  
  while ( radio::rxPayloadAvaliable(pipe_num) ){


    if (radio::dynamicPayloadsEnabled()) {
      if( (frame_size = radio::getRxPayloadSize() ) < sizeof(RF24NetworkHeader)){
    	sys::delayInMs(10);
		continue;
	  }
    }
    else {
      frame_size=32;
    }

    PRINT("Frame Size ", frame_size, ENDL);

	// Dump the payloads until we've gotten everything
	// Fetch the payload, and see if this was the last one.
	radio::readPayload( frame_buffer, frame_size );

      // Read the beginning of the frame as the header
	  RF24NetworkHeader *header = (RF24NetworkHeader*)(&frame_buffer);
	  
	  PRINT("From node: ",header->from_node, ENDL);
	  PRINT("To node: ",header->to_node, ENDL);

      //IF_SERIAL_DEBUG(printf_P(PSTR("%lu: MAC Received on %u %s\n\r"),sys::millis(),pipe_num,header->toString()));
      //IF_SERIAL_DEBUG(const uint16_t* i = reinterpret_cast<const uint16_t*>(frame_buffer + sizeof(RF24NetworkHeader));printf_P(PSTR("%lu: NET message %04x\n\r"),sys::millis(),*i));
	  
      // Throw it away if it's not a valid address
      if ( !is_valid_address(header->to_node) ){
		continue;
	  }
	  
	  uint8_t returnVal = header->type;

	  // Is this for us?
      if ( header->to_node == node_address   ){
			
			if(header->type == NETWORK_PING){
				PRINT("Ping", ENDL);
			   continue;
			}
		    if(header->type == NETWORK_ADDR_RESPONSE ){	
		    	PRINT("Resp", ENDL);
			    uint16_t requester = 04444;
				if(requester != node_address){
					header->to_node = requester;
					write(header->to_node,USER_TX_TO_PHYSICAL_ADDRESS);
					sys::delayInMs(10);
                    write(header->to_node,USER_TX_TO_PHYSICAL_ADDRESS);
					//printf("Fwd add response to 0%o\n",requester);
					continue;
				}
			}
			if(header->type == NETWORK_REQ_ADDRESS && node_address){
				PRINT("Req", ENDL);
				//printf("Fwd add req to 0\n");
				header->from_node = node_address;
				header->to_node = 0;
				write(header->to_node,TX_NORMAL);
				continue;
			}
			
			if( (returnSysMsgs && header->type > 127) || header->type == NETWORK_ACK ){	
				PRINT("Type Received", header->type, ENDL);
				//IF_SERIAL_DEBUG_ROUTING( printf_P(PSTR("%lu MAC: System payload rcvd %d\n"),sys::millis(),returnVal); );
				//if( (header->type < 148 || header->type > 150) && header->type != NETWORK_MORE_FRAGMENTS_NACK && header->type != EXTERNAL_DATA_TYPE && header->type!= NETWORK_LAST_FRAGMENT){
				if( header->type != NETWORK_FIRST_FRAGMENT && header->type != NETWORK_MORE_FRAGMENTS && header->type != NETWORK_MORE_FRAGMENTS_NACK && header->type != EXTERNAL_DATA_TYPE && header->type!= NETWORK_LAST_FRAGMENT){
					return returnVal;
				}
			}
			if( enqueue(header) == 2 ){ //External data received
				PRINT("ext",ENDL);
				return EXTERNAL_DATA_TYPE;				
			}
	  }else{	  
	  #if defined	(RF24NetworkMulticast)	

			if( header->to_node == 0100){
			

				if(header->type == NETWORK_POLL  ){
                    if( !(networkFlags & FLAG_NO_POLL) && node_address != 04444 ){
					  header->to_node = header->from_node;
					  header->from_node = node_address;			
					  sys::delayInMs(parent_pipe);
                      write(header->to_node,USER_TX_TO_PHYSICAL_ADDRESS);                      
                    }
					continue;
				}
				uint8_t val = enqueue(header);
				
				if(multicastRelay){					
					//IF_SERIAL_DEBUG_ROUTING( printf_P(PSTR("%u MAC: FWD multicast frame from 0%o to level %u\n"),sys::millis(),header->from_node,multicast_level+1); );
					write(levelToAddress(multicast_level)<<3,4);
				}
				if( val == 2 ){ //External data received			
				  //Serial.println("ret ext multicast");
					return EXTERNAL_DATA_TYPE;
				}

			}else{
				write(header->to_node,1);	//Send it on, indicate it is a routed payload
			}
		#else
		write(header->to_node,1);	//Send it on, indicate it is a routed payload
		#endif
	  }
	  
  }
  return returnVal;
}

/******************************************************************/
/******************************************************************/

uint8_t RF24Network::enqueue(RF24NetworkHeader* header)
{
  bool result = false;
  uint16_t message_size = frame_size - sizeof(RF24NetworkHeader);
  
  //IF_SERIAL_DEBUG(printf_P(PSTR("%lu: NET Enqueue @%x "),sys::millis(),next_frame-frame_queue));
  
#if !defined ( DISABLE_FRAGMENTATION ) 

  bool isFragment = header->type == NETWORK_FIRST_FRAGMENT || header->type == NETWORK_MORE_FRAGMENTS || header->type == NETWORK_LAST_FRAGMENT || header->type == NETWORK_MORE_FRAGMENTS_NACK ;

  if(isFragment){

	if(header->type == NETWORK_FIRST_FRAGMENT){
	    // Drop frames exceeding max size and duplicates (MAX_PAYLOAD_SIZE needs to be divisible by 24)
	    if(header->reserved > (MAX_PAYLOAD_SIZE / max_frame_payload_size) ){
			//printf_P(PSTR("Frag frame with %d frags exceeds MAX_PAYLOAD_SIZE or out of sequence\n"),header->reserved);

			frag_queue.header.reserved = 0;
			return false;
		}else
        if(frag_queue.header.id == header->id && frag_queue.header.from_node == header->from_node){
            return true;
        }
        
        if( (header->reserved * 24) > (MAX_PAYLOAD_SIZE - (next_frame-frame_queue)) ){
          networkFlags |= FLAG_HOLD_INCOMING;
          radio::disableRx();
        }
  		  
		memcpy(&frag_queue,&frame_buffer,8);
		memcpy(frag_queue.message_buffer,frame_buffer+sizeof(RF24NetworkHeader),message_size);
		
//IF_SERIAL_DEBUG_FRAGMENTATION( Serial.print(F("queue first, total frags ")); Serial.println(header->reserved); );
		//Store the total size of the stored frame in message_size
	    frag_queue.message_size = message_size;
		--frag_queue.header.reserved;
		  
//IF_SERIAL_DEBUG_FRAGMENTATION_L2(  for(int i=0; i<frag_queue.message_size;i++){  Serial.println(frag_queue.message_buffer[i],HEX);  } );
		
		return true;		

	}else // NETWORK_MORE_FRAGMENTS	
	if(header->type == NETWORK_LAST_FRAGMENT || header->type == NETWORK_MORE_FRAGMENTS || header->type == NETWORK_MORE_FRAGMENTS_NACK){
		
        if(frag_queue.message_size + message_size > MAX_PAYLOAD_SIZE){
          //Serial.print(F("Drop frag ")); Serial.print(header->reserved);
          //Serial.println(F(" Size exceeds max"));
          frag_queue.header.reserved=0;
          return false;
        }
		if(  frag_queue.header.reserved == 0 || (header->type != NETWORK_LAST_FRAGMENT && header->reserved != frag_queue.header.reserved ) || frag_queue.header.id != header->id ){
			//Serial.print(F("Drop frag ")); Serial.print(header->reserved);
			//Serial.print(F(" header id ")); Serial.print(header->id);
			//Serial.println(F(" Out of order "));
			return false;
		}
		
		memcpy(frag_queue.message_buffer+frag_queue.message_size,frame_buffer+sizeof(RF24NetworkHeader),message_size);
	    frag_queue.message_size += message_size;
		
		if(header->type != NETWORK_LAST_FRAGMENT){
		  --frag_queue.header.reserved;
		  return true;
		}
		frag_queue.header.reserved = 0;
        frag_queue.header.type = header->reserved;
		
//IF_SERIAL_DEBUG_FRAGMENTATION( printf_P(PSTR("fq 3: %d\n"),frag_queue.message_size); );
//IF_SERIAL_DEBUG_FRAGMENTATION_L2(for(int i=0; i< frag_queue.message_size;i++){ Serial.println(frag_queue.message_buffer[i],HEX); }  );
	
		//Frame assembly complete, copy to main buffer if OK		
        if(frag_queue.header.type == EXTERNAL_DATA_TYPE){
           return 2;
        }
        #if defined (DISABLE_USER_PAYLOADS)
		  return 0;
		#endif
            
        if(MAX_PAYLOAD_SIZE - (next_frame-frame_queue) >= frag_queue.message_size){
          memcpy(next_frame,&frag_queue,10);
          memcpy(next_frame+10,frag_queue.message_buffer,frag_queue.message_size);
          next_frame += (10+frag_queue.message_size);
          #if !defined(ARDUINO_ARCH_AVR)
          if(uint8_t padding = (frag_queue.message_size+10)%4){
            next_frame += 4 - padding;
          }
          #endif
          //IF_SERIAL_DEBUG_FRAGMENTATION( printf_P(PSTR("enq size %d\n"),frag_queue.message_size); );
		  return true;
		}else{
          radio::disableRx();
          networkFlags |= FLAG_HOLD_INCOMING;          
        }
       // IF_SERIAL_DEBUG_FRAGMENTATION( printf_P(PSTR("Drop frag payload, queue full\n")); );
        return false;
	}//If more or last fragments

  }else //else is not a fragment
 #endif // End fragmentation enabled

  // Copy the current frame into the frame queue

#if !defined( DISABLE_FRAGMENTATION )

	if(header->type == EXTERNAL_DATA_TYPE){
		memcpy(&frag_queue,&frame_buffer,8);
		frag_queue.message_buffer = frame_buffer+sizeof(RF24NetworkHeader);
		frag_queue.message_size = message_size;
		return 2;
	}
#endif		
#if defined (DISABLE_USER_PAYLOADS)
	return 0;
 }
#else
  if(message_size + (next_frame-frame_queue) <= MAIN_BUFFER_SIZE){
	  uart::sendLine("User Msg Pass");
	memcpy(next_frame,&frame_buffer,8);
    memcpy(next_frame+8,&message_size,2);
	memcpy(next_frame+10,frame_buffer+8,message_size);
    
	//IF_SERIAL_DEBUG_FRAGMENTATION( for(int i=0; i<message_size;i++){ Serial.print(next_frame[i],HEX); Serial.print(" : "); } Serial.println(""); );
    
	next_frame += (message_size + 10);
    #if !defined(ARDUINO_ARCH_AVR)
    if(uint8_t padding = (message_size+10)%4){
      next_frame += 4 - padding;
    }
    #endif
  //IF_SERIAL_DEBUG_FRAGMENTATION( Serial.print("Enq "); Serial.println(next_frame-frame_queue); );//printf_P(PSTR("enq %d\n"),next_frame-frame_queue); );
  
    result = true;
  }else{
	  uart::sendLine("User Msg Fail: ");
	  uart::send(message_size);
	  uart::send(" ");
	  uart::send((int)next_frame);
	  uart::send(" ");
	  uart::send((int)frame_queue);
	  uart::sendLine();
    result = false;
    //IF_SERIAL_DEBUG(printf_P(PSTR("NET **Drop Payload** Buffer Full")));
  }
  return result;
}
#endif //USER_PAYLOADS_ENABLED

/******************************************************************/

bool RF24Network::available(void)
{
  // Are there frames on the queue for us?
  return (next_frame > frame_queue);
}

/******************************************************************/

uint16_t RF24Network::parent()
{
  if ( node_address == 0 )
    return -1;
  else
    return parent_node;
}

/******************************************************************/
/*uint8_t RF24Network::peekData(){
		
		return frame_queue[0];
}*/

uint16_t RF24Network::peek(RF24NetworkHeader& header)
{
  if ( available() )
  {
	RF24NetworkFrame *frame = (RF24NetworkFrame*)(frame_queue);
	memcpy(&header,&frame->header,sizeof(RF24NetworkHeader));
    uint16_t msg_size;
    memcpy(&msg_size,frame+8,2);
    return msg_size;
  }
  return 0;
}

/******************************************************************/

uint16_t RF24Network::read(RF24NetworkHeader& header,void* message, uint16_t maxlen)
{
  uint16_t bufsize = 0;

  if ( available() )
  {
    
	memcpy(&header,frame_queue,8);
    memcpy(&bufsize,frame_queue+8,2);

    if (maxlen > 0)
    {		
		maxlen = rf24_min(maxlen,bufsize);
		memcpy(message,frame_queue+10,maxlen);
	    //IF_SERIAL_DEBUG(printf("%lu: NET message size %d\n",sys::millis(),bufsize););

	
	//IF_SERIAL_DEBUG( uint16_t len = maxlen; printf_P(PSTR("%lu: NET r message "),sys::millis());const uint8_t* charPtr = reinterpret_cast<const uint8_t*>(message);while(len--){ printf("%02x ",charPtr[len]);} printf_P(PSTR("\n\r") ) );
	  
    }
	memmove(frame_queue,frame_queue+bufsize+10,sizeof(frame_queue)- bufsize);
	next_frame-=bufsize+10;
    #if !defined(ARDUINO_ARCH_AVR)
    if(uint8_t padding = (bufsize+10)%4){
      next_frame -= 4 - padding;
    }
    #endif
	//IF_SERIAL_DEBUG(printf_P(PSTR("%lu: NET Received %s\n\r"),sys::millis(),header.toString()));
  }

  return bufsize;
}


#if defined RF24NetworkMulticast
/******************************************************************/
bool RF24Network::multicast(RF24NetworkHeader& header,const void* message, uint16_t len, uint8_t level){
	// Fill out the header
  header.to_node = 0100;
  header.from_node = node_address;
  return write(header, message, len, levelToAddress(level));
}
#endif

/******************************************************************/
bool RF24Network::write(RF24NetworkHeader& header,const void* message, uint16_t len){
	return write(header,message,len,070);
}
/******************************************************************/
bool RF24Network::write(RF24NetworkHeader& header,const void* message, uint16_t len, uint16_t writeDirect){
    
	uart::sendLine("write");
    //Allows time for requests (RF24Mesh) to get through between failed writes on busy nodes
    while(sys::millis()-txTime < 25){ if(update() > 127){break;} }
	sys::delayInMs(200);

#if defined (DISABLE_FRAGMENTATION)
    frame_size = rf24_min(len+sizeof(RF24NetworkHeader),MAX_FRAME_SIZE);
	return _write(header,message,rf24_min(len,max_frame_payload_size),writeDirect);
#else  
  if(len <= max_frame_payload_size){
    //Normal Write (Un-Fragmented)
	frame_size = len + sizeof(RF24NetworkHeader);
    if(_write(header,message,len,writeDirect)){
      return 1;
    }
    txTime = sys::millis();
    return 0;
  }
  //Check payload size
  if (len > MAX_PAYLOAD_SIZE) {
    //IF_SERIAL_DEBUG(printf("%u: NET write message failed. Given 'len' %d is bigger than the MAX Payload size %i\n\r",sys::millis(),len,MAX_PAYLOAD_SIZE););
    return false;
  }

  //Divide the message payload into chunks of max_frame_payload_size
  uint8_t fragment_id = (len % max_frame_payload_size != 0) + ((len ) / max_frame_payload_size);  //the number of fragments to send = ceil(len/max_frame_payload_size)

  uint8_t msgCount = 0;

 // IF_SERIAL_DEBUG_FRAGMENTATION(printf("%lu: FRG Total message fragments %d\n\r",sys::millis(),fragment_id););
  
  if(header.to_node != 0100){
    networkFlags |= FLAG_FAST_FRAG;
	radio::disableRx();
  }

  uint8_t retriesPerFrag = 0;
  uint8_t type = header.type;
  bool ok = 0;
  
  while (fragment_id > 0) {

    //Copy and fill out the header
    //RF24NetworkHeader fragmentHeader = header;
   header.reserved = fragment_id;

    if (fragment_id == 1) {
      header.type = NETWORK_LAST_FRAGMENT;  //Set the last fragment flag to indicate the last fragment
      header.reserved = type; //The reserved field is used to transmit the header type
    } else {
      if (msgCount == 0) {
        header.type = NETWORK_FIRST_FRAGMENT;
      }else{
        header.type = NETWORK_MORE_FRAGMENTS; //Set the more fragments flag to indicate a fragmented frame
      }
    }
	
    uint16_t offset = msgCount*max_frame_payload_size;
	uint16_t fragmentLen = rf24_min((uint16_t)(len-offset),max_frame_payload_size);

    //Try to send the payload chunk with the copied header
    frame_size = sizeof(RF24NetworkHeader)+fragmentLen;
	ok = _write(header,((char *)message)+offset,fragmentLen,writeDirect);

	if (!ok) {
	   sys::delayInMs(2);
	   ++retriesPerFrag;

	}else{
	  retriesPerFrag = 0;
	  fragment_id--;
      msgCount++;
	}
	
    //if(writeDirect != 070){ delay(2); } //Delay 2ms between sending multicast payloads
 
	if (!ok && retriesPerFrag >= 3) {
        //IF_SERIAL_DEBUG_FRAGMENTATION(printf("%lu: FRG TX with fragmentID '%d' failed after %d fragments. Abort.\n\r",sys::millis(),fragment_id,msgCount););
		break;
    }

	
    //Message was successful sent
    #if defined SERIAL_DEBUG_FRAGMENTATION_L2 
	  //printf("%lu: FRG message transmission with fragmentID '%d' sucessfull.\n\r",sys::millis(),fragment_id);
	#endif

  }
  header.type = type;

  if(networkFlags & FLAG_FAST_FRAG){	
    //ok = radio.txStandBy(txTimeout);
    radio::disableRx();
    radio::disableAutoAck(Nrf24Pipe::RX_PIPE0);
  }  
  networkFlags &= ~FLAG_FAST_FRAG;
  
  if(!ok){
       return false;
  }
  //int frag_delay = uint8_t(len/48);
  //delay( rf24_min(len/48,20));

  //Return true if all the chunks where sent successfully
 
  //IF_SERIAL_DEBUG_FRAGMENTATION(printf("%u: FRG total message fragments sent %i. \n",sys::millis(),msgCount); );
  if(fragment_id > 0){
    txTime = sys::millis();
	return false;
  }
  return true;
  
#endif //Fragmentation enabled
}
/******************************************************************/

bool RF24Network::_write(RF24NetworkHeader& header,const void* message, uint16_t len, uint16_t writeDirect)
{
  // Fill out the header
  header.from_node = node_address;
  

  sys::delayInMs(10);
  // Build the full frame to send
  memcpy(frame_buffer,&header,sizeof(RF24NetworkHeader));
  
  //IF_SERIAL_DEBUG(printf_P(PSTR("%lu: NET Sending %s\n\r"),sys::millis(),header.toString()));

  if (len){
	  memcpy(frame_buffer + sizeof(RF24NetworkHeader),message,len);
	
	//IF_SERIAL_DEBUG(uint16_t tmpLen = len;printf_P(PSTR("%lu: NET message "),sys::millis());const uint8_t* charPtr = reinterpret_cast<const uint8_t*>(message);while(tmpLen--){ printf("%02x ",charPtr[tmpLen]);} printf_P(PSTR("\n\r") ) );
  }

  // If the user is trying to send it to himself
  /*if ( header.to_node == node_address ){
	#if defined (RF24_LINUX)
	  RF24NetworkFrame frame = RF24NetworkFrame(header,message,rf24_min(MAX_FRAME_SIZE-sizeof(RF24NetworkHeader),len));	
	#else
      RF24NetworkFrame frame(header,len);
    #endif
	// Just queue it in the received queue
    return enqueue(frame);
  }*/
    // Otherwise send it out over the air	
	
	
	if(writeDirect != 070){
		uint8_t sendType = USER_TX_TO_LOGICAL_ADDRESS; // Payload is multicast to the first node, and routed normally to the next
	    
		if(header.to_node == 0100){
		  sendType = USER_TX_MULTICAST;
		}
		if(header.to_node == writeDirect){
		  sendType = USER_TX_TO_PHYSICAL_ADDRESS; // Payload is multicast to the first node, which is the recipient
		}
		return write(writeDirect,sendType);				
	}
	return write(header.to_node,TX_NORMAL);
	
}

/******************************************************************/

bool RF24Network::write(uint16_t to_node, uint8_t directTo)  // Direct To: 0 = First Payload, standard routing, 1=routed payload, 2=directRoute to host, 3=directRoute to Route
{
  bool ok = false;
  bool isAckType = false;
  if(frame_buffer[6] > 64 && frame_buffer[6] < 192 ){ isAckType=true; }
  
  /*if( ( (frame_buffer[7] % 2) && frame_buffer[6] == NETWORK_MORE_FRAGMENTS) ){
	isAckType = 0;
  }*/
  
  // Throw it away if it's not a valid address
  if ( !is_valid_address(to_node) )
    return false;  
  
  //Load info into our conversion structure, and get the converted address info
  logicalToPhysicalStruct conversion = { to_node,directTo,0};
  logicalToPhysicalAddress(&conversion);
  

  //IF_SERIAL_DEBUG(printf_P(PSTR("%lu: MAC Sending to 0%o via 0%o on pipe %x\n\r"),sys::millis(),to_node,conversion.send_node,conversion.send_pipe));

  /**Write it*/
  uart::send((int)conversion.multicast);
  ok=write_to_pipe(conversion.send_node, conversion.send_pipe, conversion.multicast);  	
  
  
    if(!ok){	
    	uart::send("Send failed");
	}
 
	if( directTo == TX_ROUTED && ok && conversion.send_node == to_node && isAckType){
			
			RF24NetworkHeader* header = (RF24NetworkHeader*)&frame_buffer;
			header->type = NETWORK_ACK;				    // Set the payload type to NETWORK_ACK			
			header->to_node = header->from_node;          // Change the 'to' address to the 'from' address			

			conversion.send_node = header->from_node;
			conversion.send_pipe = TX_ROUTED;
			conversion.multicast = 0;
			logicalToPhysicalAddress(&conversion);
			
			//Write the data using the resulting physical address
			frame_size = sizeof(RF24NetworkHeader);
			write_to_pipe(conversion.send_node, conversion.send_pipe, conversion.multicast);
			
			//dynLen=0;
			   //IF_SERIAL_DEBUG_ROUTING( printf_P(PSTR("%lu MAC: Route OK to 0%o ACK sent to 0%o\n"),sys::millis(),to_node,header->from_node); );

	}
 


	if( ok && conversion.send_node != to_node && (directTo==0 || directTo==3) && isAckType){
	    #if !defined (DUAL_HEAD_RADIO)
          // Now, continue listening
		  if(networkFlags & FLAG_FAST_FRAG){
			// radio.txStandBy(txTimeout);
             networkFlags &= ~FLAG_FAST_FRAG;
             radio::disableAutoAck(Nrf24Pipe::RX_PIPE0);
		  }
          radio::enableRx();
        #endif
		uint32_t reply_time = sys::millis();

		while( update() != NETWORK_ACK){
			if(sys::millis() - reply_time > routeTimeout){
				  //IF_SERIAL_DEBUG_ROUTING( printf_P(PSTR("%lu: MAC Network ACK fail from 0%o via 0%o on pipe %x\n\r"),sys::millis(),to_node,conversion.send_node,conversion.send_pipe); );
				uart::sendLine("MAC Network Ack Failed");
				ok=false;
				break;					
			}
		}
    }
    if( !(networkFlags & FLAG_FAST_FRAG) ){
	   #if !defined (DUAL_HEAD_RADIO)
         // Now, continue listening
         radio::enableRx();
       #endif	
	}

#if defined ENABLE_NETWORK_STATS
  if(ok == true){
			++nOK;
  }else{	++nFails;
  }
#endif
  return ok;
}

/******************************************************************/

	// Provided the to_node and directTo option, it will return the resulting node and pipe
bool RF24Network::logicalToPhysicalAddress(logicalToPhysicalStruct *conversionInfo){

  //Create pointers so this makes sense.. kind of
  //We take in the to_node(logical) now, at the end of the function, output the send_node(physical) address, etc.
  //back to the original memory address that held the logical information.
  uint16_t *to_node = &conversionInfo->send_node;
  uint8_t *directTo = &conversionInfo->send_pipe;
  bool *multicast = &conversionInfo->multicast;
  
  // Where do we send this?  By default, to our parent
  uint16_t pre_conversion_send_node = parent_node; 

  // On which pipe
  uint8_t pre_conversion_send_pipe = parent_pipe;
  
  PRINT("Send node: ", (*conversionInfo).send_node, ENDL)

 if(*directTo > TX_ROUTED ){    
	pre_conversion_send_node = *to_node;
	*multicast = 1;
	//if(*directTo == USER_TX_MULTICAST || *directTo == USER_TX_TO_PHYSICAL_ADDRESS){
		pre_conversion_send_pipe=0;
	//}	
  }     
  // If the node is a direct child,
  else
  if ( is_direct_child(*to_node) )
  {   
    // Send directly
    pre_conversion_send_node = *to_node;
    // To its listening pipe
    pre_conversion_send_pipe = 5;
  }
  // If the node is a child of a child
  // talk on our child's listening pipe,
  // and let the direct child relay it.
  else if ( is_descendant(*to_node) )
  {
    pre_conversion_send_node = direct_child_route_to(*to_node);
    pre_conversion_send_pipe = 5;
  }
  
  *to_node = pre_conversion_send_node;
  *directTo = pre_conversion_send_pipe;

  return 1;
  
}

/********************************************************/


bool RF24Network::write_to_pipe( uint16_t node, uint8_t pipe, bool multicast )
{
  bool ok = false;
  uint8_t out_pipe[5];
  pipe_address( node, pipe, out_pipe );
  
  for(int i = 0; i < sizeof(out_pipe); ++i) { uart::send(out_pipe[i]); };
  uart::sendLine();
  uart::send(node);
  uart::sendLine();

  // Open the correct pipe for writing.
  // First, stop listening so we can talk

  if(!(networkFlags & FLAG_FAST_FRAG)){
    radio::disableRx();
  }
  
  if(multicast){ radio::disableAutoAck(Nrf24Pipe::RX_PIPE0);}else{radio::enableAutoAck(Nrf24Pipe::RX_PIPE0);}
  
  radio::openPipe(Nrf24Pipe::TX_PIPE,out_pipe);

  ok = radio::writePayload(frame_buffer, frame_size,multicast);
  
  if(!(networkFlags & FLAG_FAST_FRAG)){
    //ok = radio.txStandBy(txTimeout);
    radio::disableAutoAck(Nrf24Pipe::RX_PIPE0);
  }


/*  #if defined (__arm__) || defined (RF24_LINUX)
  IF_SERIAL_DEBUG(printf_P(PSTR("%u: MAC Sent on %x %s\n\r"),sys::millis(),(uint32_t)out_pipe,ok?PSTR("ok"):PSTR("failed")));
  #else
  IF_SERIAL_DEBUG(printf_P(PSTR("%lu: MAC Sent on %lx %S\n\r"),sys::millis(),(uint32_t)out_pipe,ok?PSTR("ok"):PSTR("failed")));
  #endif
*/  
  return ok;
}

/******************************************************************/

const char* RF24NetworkHeader::toString(void) const
{
  //static char buffer[45];
  //snprintf_P(buffer,sizeof(buffer),PSTR("id %04x from 0%o to 0%o type %c"),id,from_node,to_node,type);
  //sprintf_P(buffer,PSTR("id %u from 0%o to 0%o type %d"),id,from_node,to_node,type);
  return 0;
}

/******************************************************************/

bool RF24Network::is_direct_child( uint16_t node )
{
  bool result = false;

  // A direct child of ours has the same low numbers as us, and only
  // one higher number.
  //
  // e.g. node 0234 is a direct child of 034, and node 01234 is a
  // descendant but not a direct child

  // First, is it even a descendant?
  if ( is_descendant(node) )
  {
    // Does it only have ONE more level than us?
    uint16_t child_node_mask = ( ~ node_mask ) << 3;
    result = ( node & child_node_mask ) == 0 ;
  }
  return result;
}

/******************************************************************/

bool RF24Network::is_descendant( uint16_t node )
{
  return ( node & node_mask ) == node_address;
}

/******************************************************************/

void RF24Network::setup_address(void)
{
  // First, establish the node_mask

	uint16_t node_mask_check = 0xFFFF;
  #if defined (RF24NetworkMulticast)
	multicast_level = 0;
  #endif

  while ( (node_address & node_mask_check) > 0 ){
    node_mask_check <<= 3;
  #if defined (RF24NetworkMulticast)
    ++multicast_level;
  }

  #else
  }
  #endif

  uint16_t parent_mask = (~node_mask_check) >> 3;

  // parent node is the part IN the mask
  parent_node = node_address & parent_mask;

  // parent pipe is the part OUT of the mask
  uint16_t i = node_address;
  while (parent_mask)
  {
    i >>= 3;
    parent_mask >>= 3;
  }
  parent_pipe = i;

  //IF_SERIAL_DEBUG_MINIMAL( printf_P(PSTR("setup_address node=0%o mask=0%o parent=0%o pipe=0%o\n\r"),node_address,node_mask,parent_node,parent_pipe););

}

/******************************************************************/
uint16_t RF24Network::addressOfPipe( uint16_t node, uint8_t pipeNo )
{
		//Say this node is 013 (1011), mask is 077 or (00111111)
		//Say we want to use pipe 3 (11)
        //6 bits in node mask, so shift pipeNo 6 times left and | into address		
	uint16_t m = node_mask >> 3;
	uint8_t i=0;
	
	while (m){ 	   //While there are bits left in the node mask
		m>>=1;     //Shift to the right
		i++;       //Count the # of increments
	}
    return node | (pipeNo << i);	
}

/******************************************************************/

uint16_t RF24Network::direct_child_route_to( uint16_t node )
{
  // Presumes that this is in fact a child!!
  uint16_t child_mask = ( node_mask << 3 ) | 0x07;
  return node & child_mask;
  
}

/******************************************************************/
/*
uint8_t RF24Network::pipe_to_descendant( uint16_t node )
{
  uint16_t i = node;       
  uint16_t m = node_mask;

  while (m)
  {
    i >>= 3;
    m >>= 3;
  }

  return i & 0B111;
}*/

/******************************************************************/

bool RF24Network::is_valid_address( uint16_t node )
{
  bool result = true;

  while(node)
  {
    uint8_t digit = node & 0x07;
	#if !defined (RF24NetworkMulticast)
    if (digit < 1 || digit > 5)
	#else
	if (digit < 0 || digit > 5)	//Allow our out of range multicast address
	#endif
    {
      result = false;
      //IF_SERIAL_DEBUG_MINIMAL(printf_P(PSTR("*** WARNING *** Invalid address 0%o\n\r"),node););
      break;
    }
    node >>= 3;
  }

  return result;
}

/******************************************************************/
#if defined (RF24NetworkMulticast)
void RF24Network::multicastLevel(uint8_t level){
  multicast_level = level;
  //radio.stopListening();  
	uint8_t pipe[5];
	pipe_address(levelToAddress(level),0,pipe);
	radio::openPipe(Nrf24Pipe::RX_PIPE0,pipe);
  //radio.startListening();
  }
  
uint16_t levelToAddress(uint8_t level){
	
	uint16_t levelAddr = 1;
	if(level){
		levelAddr = levelAddr << ((level-1) * 3);
	}else{
		return 0;		
	}
	return levelAddr;
}  
#endif
/******************************************************************/

static const uint8_t address_translation[] = { 0xc3,0x3c,0x33,0xce,0x3e,0xe3,0xec };

void pipe_address( uint16_t node, uint8_t pipe, uint8_t* return_address )
{
  
  memset(return_address, 0xCC, 5);

  // Translate the address to use our optimally chosen radio address bytes
	uint8_t count = 1; uint16_t dec = node;

	while(dec){
	  #if defined (RF24NetworkMulticast)
	  if(pipe != 0 || !node)
      #endif
		  return_address[count]=address_translation[(dec % 8)];		// Convert our decimal values to octal, translate them to address bytes, and set our address
	  
	  dec /= 8;	
	  count++;
	}
    
	#if defined (RF24NetworkMulticast)
	if(pipe != 0 || !node)
	#endif
		return_address[0] = address_translation[pipe];
	#if defined (RF24NetworkMulticast)
	else
		return_address[1] = address_translation[count-1];
	#endif

  //IF_SERIAL_DEBUG(uint32_t* top = reinterpret_cast<uint32_t*>(out+1);printf_P(PSTR("%lu: NET Pipe %i on node 0%o has address %lx%x\n\r"),sys::millis(),pipe,node,*top,*out));
}


/************************ Sleep Mode ******************************************/


#if defined ENABLE_SLEEP_MODE

#if !defined(__arm__) && !defined(__ARDUINO_X86__)

void wakeUp(){
  wasInterrupted=true;
  sleep_cycles_remaining = 0;
}

ISR(WDT_vect){
  --sleep_cycles_remaining;
}


bool RF24Network::sleepNode( unsigned int cycles, int interruptPin ){


  sleep_cycles_remaining = cycles;
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  if(interruptPin != 255){
    wasInterrupted = false; //Reset Flag
  	attachInterrupt(interruptPin,wakeUp, LOW);
  }    

  #if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  WDTCR |= _BV(WDIE);
  #else
  WDTCSR |= _BV(WDIE);
  #endif

  while(sleep_cycles_remaining){
    sleep_mode();                        // System sleeps here
  }                                     // The WDT_vect interrupt wakes the MCU from here
  sleep_disable();                     // System continues execution here when watchdog timed out
  detachInterrupt(interruptPin);

  #if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
	WDTCR &= ~_BV(WDIE);
  #else
	WDTCSR &= ~_BV(WDIE);
  #endif
  
  return !wasInterrupted;
}

void RF24Network::setup_watchdog(uint8_t prescalar){

  uint8_t wdtcsr = prescalar & 7;
  if ( prescalar & 8 )
    wdtcsr |= _BV(WDP3);
  MCUSR &= ~_BV(WDRF);                      // Clear the WD System Reset Flag

  #if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  WDTCR = _BV(WDCE) | _BV(WDE);            // Write the WD Change enable bit to enable changing the prescaler and enable system reset
  WDTCR = _BV(WDCE) | wdtcsr | _BV(WDIE);  // Write the prescalar bits (how long to sleep, enable the interrupt to wake the MCU
  #else
  WDTCSR = _BV(WDCE) | _BV(WDE);            // Write the WD Change enable bit to enable changing the prescaler and enable system reset
  WDTCSR = _BV(WDCE) | wdtcsr | _BV(WDIE);  // Write the prescalar bits (how long to sleep, enable the interrupt to wake the MCU
  #endif
}


#endif // not ATTiny
#endif // Enable sleep mode
