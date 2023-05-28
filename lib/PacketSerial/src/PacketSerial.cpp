
/*!
* @file /*!
* @file PacketSerial.cpp
*
* @mainpage Serial/HMI Display Library
*
* @section intro_sec_Introduction
*
* An Arduino C++ library for interfacing with serial displays.
* 
* @section author Author
* 
* Gerhard Malan for GM Consult Pty Ltd
* 
 * @section license License
 * 
 * This library is open-source under the BSD 3-Clause license and 
 * redistribution and use in source and binary forms, with or without 
 * modification, are permitted, provided that the license conditions are met.
 * 
*/

#include <PacketSerial.h>

uint16_t * PacketSerial::getHeaders(){
    return headers.data();
};

uint8_t  PacketSerial::available(void){        
    return uxQueueMessagesWaiting(rxQueue);
};

bool PacketSerial::read(ps_byte_array_t & packet){  
    return (xQueueReceive(rxQueue, &( packet ), ( TickType_t ) 10 ) == pdPASS );
};

 
bool PacketSerial::write(ps_byte_array_t * frame){    
    return send_to_frame_queue(txQueue, frame);
};

void PacketSerial:: serial_tx(void ){
    for(;;){
        // vTaskDelay(100/portTICK_RATE_MS);
            ps_byte_array_t frame;
            if(xQueueReceive(txQueue, &( frame ), ( TickType_t ) 10 ) == pdPASS ){
                // ps_frame_t frm = PS_FRAME(&frame);      
                onSerialTx(&frame);        
                ps_length_t lenD = frame.data[2];
                std::vector<uint8_t> v;                    
                v.insert(v.end(), &frame.data[0], &frame.data[lenD+3]);
                ps_serial_port->write(v.data(), v.size());
            }            
    }
};

void PacketSerial:: onSerialTx(ps_byte_array_t * frame){};

bool PacketSerial::headerValid(ps_header_t header){
    bool isValid = false;    
    for (const auto& t : headers) { // reference avoids copying element
        isValid = isValid || (t == header);      // element can not be changed
    }       
    return isValid;
};

void PacketSerial::serial_rx(void){
    // Serial.println("serial_rx(void)");
    ps_length_t length;        
    ps_byte_array_t frame;           // the frame being received
    memset(frame.data,0, sizeof(frame.data));
    uint8_t bc;                 // byte counter
    uint32_t ticks = 0;
    for(;;){
        while (ps_serial_port->available()>0){
            ps_header_t header;
            ticks = 0;                
            uint8_t d = (uint8_t)ps_serial_port->read();            
            if (bc>0){
                header = ((frame.data[bc-1]) << 8) | d;
            }
            switch (bc){
            case 0: // first character, do nothing               
                break;
            case 1: // second character - if not a valid header, reset byte counter and frame                         
                if (!headerValid(header)){
                    #ifdef PS_DEBUG                
                    Serial.println ("case 1");
                    frame.print();
                    Serial.println();
                    #endif        
                    memset(frame.data,0, sizeof(frame.data));
                    bc = 0;                           
                }
                break;                          
            default: //third or higher character - if a new header was received, send the previous packet
                if (headerValid(header) ){
                    frame.length = bc - 1;
                    onSerialRx(&frame);
                    send_to_frame_queue(rxQueue, &frame);
                    uint8_t first = (frame.data[bc-1]);
                    bc = 1;
                    memset(frame.data,0, sizeof(frame.data));
                    frame.data[0]= first;
                }           
            }
            frame.data[bc] = d;
            bc++;
            frame.length = bc;

        }    
        vTaskDelay(1/portTICK_PERIOD_MS);    
        ticks++;
        if (ticks > 100 && bc > 2){            
            if (headerValid(frame.header())){
                onSerialRx(&frame);
                send_to_frame_queue(rxQueue, &frame);
                bc = 0;
                memset(frame.data,0, sizeof(frame.data));
                ticks = 0;
            } else {
                #ifdef PS_DEBUG                
                Serial.println ("case else");
                frame.print();
                Serial.println();
                #endif        
            }
        }
    }
};

bool PacketSerial::send_to_frame_queue(QueueHandle_t q, ps_byte_array_t * frame ){
    bool error = true;
    bool hv = headerValid(frame->header());
    #ifdef PS_DEBUG
    if (!hv){
        frame->print();
        Serial.println();
    }
    #endif
    error = error && hv;    
    if (uxQueueSpacesAvailable(q) < 1) {
        ps_byte_array_t poppedFrame;
        while(uxQueueSpacesAvailable(q) < 1){
            xQueueReceive(q, &(poppedFrame), ( TickType_t ) 10 ) ;            
        }       
        #ifdef PS_DEBUG
        Serial.printf("The queue was full, packets were lost\n");
        #endif                             
    } 
    return xQueueSend(q, ( void * ) frame, (TickType_t ) 10 );
   
};

bool PacketSerial::onSerialRx(ps_byte_array_t * frame){
    return true; 
};


bool PacketSerial::onStartup(){
    return true; 
};

bool PacketSerial::create_rx_queue(){
    rxQueue = xQueueCreate(rx_queue_length, sizeof(ps_byte_array_t)); 
    return rxQueue != NULL;
};

bool PacketSerial::create_tx_queue(){       
    txQueue = xQueueCreate(tx_queue_length,sizeof(ps_byte_array_t));  // create the TX queue
        return txQueue != NULL;
};

void PacketSerial::serial_rx_impl(void* _this){
    static_cast<PacketSerial*>(_this)->serial_rx();
};

void PacketSerial::serial_tx_impl(void* _this){
    static_cast<PacketSerial*>(_this)->serial_tx();
};

bool PacketSerial::start_rx_task(){
    // create the RX task
    return (xTaskCreatePinnedToCore(
        this->serial_rx_impl,
        PS_RX_TASK_NAME,
        stack_size,
        this,
        task_priority,
        NULL,
        core) == pdPASS);
        
};

bool PacketSerial::start_tx_task(){
    // create the TX task
    return (xTaskCreatePinnedToCore(
        this->serial_tx_impl,
        PS_TX_TASK_NAME,
        stack_size,
        this,
        task_priority,
        NULL,
        core) == pdPASS);
};

    uint8_t PacketSerial::setBitValues(uint8_t oldValue, uint8_t newValue, uint8_t mask){
    return (oldValue & ~mask) | (newValue & mask);
};


