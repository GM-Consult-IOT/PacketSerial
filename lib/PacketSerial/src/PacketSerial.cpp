
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
* Gerhard Malan for GM COnsult Pty Ltd
* 
 * @section license License
 * 
 * This library is open-source under the BSD 3-Clause license and 
 * redistribution and use in source and binary forms, with or without 
 * modification, are permitted, provided that the license conditions are met.
 * 
*/

#include <Arduino.h>
#include <PacketSerial.h>

PacketSerial:: PacketSerial(std::vector<uint16_t> headers_,
            #ifdef PS_USE_SOFTWARE_SERIAL
                SoftwareSerial * port
            #else
                HardwareSerial * port
            #endif
            ) :
            headers(headers_),   
            ps_serial_port(port), 
            rxQueue(NULL), 
            txQueue(NULL),
            errQueue(NULL){

    };

    uint16_t * PacketSerial::getHeaders(){
        return headers.data();
    };

    ps_err_t PacketSerial::begin(){
        uint8_t retVal = 0x00;
        retVal += create_err_queue();
        create_rx_queue();
        create_tx_queue();
        start_rx_task();
        start_tx_task();
        if (retVal> 0){
            onError(PS_ERR_START_UP_FAIL);
            return PS_ERR_START_UP_FAIL;
        } else {
            onStartup();
            return PS_PASS;
        }
    };


    UBaseType_t  PacketSerial::available(void){
        return uxQueueMessagesWaiting(rxQueue);
    };

    ps_frame_t PacketSerial::read(){
         ps_frame_t msg;
        if (uxQueueMessagesWaiting(rxQueue)>0){
            ps_byte_array_t frame;
            if(xQueueReceive(rxQueue, &( frame ), ( TickType_t ) 10 ) == pdPASS ){               
                msg = PS_FRAME(&frame);
                // msg.header = ((frame.data[0]) << 8) | frame.data[1];
                // msg.length = frame.data[2];
                // for (uint8_t i = 3; i<3+msg.length; i++){
                //     msg.data.push_back(frame.data[i]);
                // }
            }
        } 
        return msg;       
    };
    
uint8_t PacketSerial::write(ps_frame_t * frame){    
    ps_byte_array_t packet;
    packet.data[0] = highByte(frame->header);
    packet.data[1] = lowByte(frame->header);
    packet.data[2] = frame->length;
    for (uint8_t i = 0; i <frame->length; i++){
        packet.data[i+3] = frame->data[i];
    }
    return send_to_frame_queue(txQueue, &packet);
};

    /// @brief  Sends a data frame to the display.    
    /// @param command The command to be used.
    /// @param address The starting address of the data
    /// @param data The uint8_t-array to be appended, to the frame
    void PacketSerial:: serial_tx(void ){
        for(;;){
            vTaskDelay(100/portTICK_RATE_MS);
                ps_byte_array_t frame;
                if(xQueueReceive(txQueue, &( frame ), ( TickType_t ) 10 ) == pdPASS ){
                    ps_frame_t frm = PS_FRAME(&frame);      
                    onSerialTx(&(frm));        
                    ps_length_t lenD = frame.data[2];
                    std::vector<uint8_t> v;                    
                    v.insert(v.end(), &frame.data[0], &frame.data[lenD+3]);
                    ps_serial_port->write(v.data(), v.size());
                }            
        }
    };



    /// @brief Called whenever a new frame is transmitted to the serial port.
    ///
    /// The base class implementation sends the frame to the rxQueue. Implementing
    /// classes can override [onSerialRx] to filter the frames or populate device
    /// properties (e.g. configuration) from the received frame.
    ///
    /// @param frame The frame reveived from the display.
    void PacketSerial:: onSerialTx(ps_frame_t * frame){};


    /// @brief  Returns true if the [header] is in the [headers] list.
    /// @param header The header to validate.
    /// @return true if the [header] is in the [headers] list.
    ps_err_t PacketSerial::headerValid(ps_byte_array_t * byte_array){
        bool isValid = false;
        ps_header_t header = ((byte_array->data[0]) << 8) | byte_array->data[1];
        for (const auto& t : headers) { // reference avoids copying element
           isValid = isValid || (t == header);      // element can not be changed
        }      
        if (!isValid){    
            onError(PS_ERR_INVALID_HEADER); 
        }
        return isValid? PS_PASS : PS_ERR_INVALID_HEADER;
    };

    /// @brief Parses data on the serial port RX.
    ///
    /// Infinite loop used as a FreeRTOS task . Uses a finite
    /// state machine (FSM) to evaluate each byte coming off the RX buffer. State is 
    /// maintained as the byte count in the current frame (inclusive of header).
    /// - start in end-of-frame (EOF) state (bc == 0), waiting for the frame.header 
    ///   bytes;
    /// - parses the frame.length (bc == 2) if both header bytes (bc == 0 and bc == 1) 
    ///   are received and the header is valid;
    /// - if bc == (3 + data length), checks if a frame with a valid header exists. 
    ///   Sends frame to the rxQueue and also calls [onSerialRx], passing the frame. 
    ///   Reinitializes the frame and resets bc = 0.
    void PacketSerial::serial_rx(void){
        ps_length_t length;        
        ps_byte_array_t frame;           // the frame being received
        memset(frame.data,0, sizeof(frame.data));
        uint8_t bc;                 // byte counter
        uint8_t maxBytes = 0xff;
        ps_length_t lenF;
        for(;;){
            vTaskDelay(10/portTICK_PERIOD_MS);
            if (ps_serial_port->available()){                
                 while (ps_serial_port->available()>0){
                    uint8_t d = (uint8_t)ps_serial_port->read();
                    switch (bc){
                        case 0:                            
                                frame.data[bc]=d;
                                bc++;                           
                        break;
                        case 1:
                            frame.data[bc] = d;
                            bc++;
                            if (!headerValid(&frame) == PS_PASS){
                                memset(frame.data,0, sizeof(frame.data));
                                bc = 0;                           
                            }
                        break;
                        case 2:
                            frame.data[bc] = d;
                                bc++;
                                lenF = d;
                                maxBytes = 3 + d;
                        break;
                        default:
                            if(bc < maxBytes){
                                frame.data[bc]=d;
                                 bc++;
                                 lenF--;
                            }
                        break;
                    }
                     if (bc > maxBytes -1 && headerValid(&frame) == PS_PASS){
                        ps_frame_t frm = PS_FRAME(&frame);
                        onSerialRx(&frm);
                        send_to_frame_queue(rxQueue, &frame);
                        bc = 0;
                        lenF = 0;
                        maxBytes = 0xff;
                        memset(frame.data,0, sizeof(frame.data));

                     }
                }

            }
        }
    };

    ps_err_t PacketSerial::send_to_frame_queue(QueueHandle_t q, ps_byte_array_t * frame ){
        ps_err_t error = PS_PASS;
        if (uxQueueSpacesAvailable(q) <1) {
            ps_byte_array_t poppedFrame;
            while(uxQueueSpacesAvailable(q) < 1){
                xQueueReceive(q, &(poppedFrame), ( TickType_t ) 10 ) ;            
            }
            ps_err_t error = q==rxQueue?PS_ERR_RX_QUEUE_FULL : PS_ERR_TX_QUEUE_FULL;
            onError(error);                                     
        } 
        xQueueSend(q, ( void * ) frame, (TickType_t ) 10 );
        return error;
    };

    /// @brief Called whenever a new frame is received from the serial port.
    ///
    /// Sends the frame to the rxQueue. Implementing classes can override 
    /// [onSerialRx] to filter the frames or populate device properties 
    /// (e.g. configuration) from the received frame.
    ///
    /// @param frame The frame reveived from the display.
    void PacketSerial::onSerialRx(ps_frame_t * frame){
        // printFrame(frame->data, frame->data[2]+3); // debugging only
        
    };

    /// @brief called when the [begin] method completes.
    void PacketSerial::onStartup(){

    };

    /// @brief Call [onError] to send the error code to the errQueue.
    ///
    /// The [error] is sent to the errQueue. If the [errQueue] is full, the oldest error 
    /// is popped before the new [error] is pushed.
    ///
    /// In debug mode the error is also printed to the serial monitor.
    /// @param error The [PS_ERR] error code as uint8_t.
    void PacketSerial::onError(uint8_t error){
        #ifdef PS_DEBUG
            Serial.print("Exception [");
            Serial.print(toHEX(error));
            Serial.println("] was thrown in class [PacketSerial].");
        #endif   
        if (uxQueueSpacesAvailable(errQueue) <2) {
            uint8_t poppedErr;
            while(uxQueueSpacesAvailable(errQueue) < 2){
                xQueueReceive(errQueue, &(poppedErr), ( TickType_t ) 10 ) ;
                #ifdef PS_DEBUG
                    Serial.print("The errQueue is full. Error code [");
                    Serial.print(poppedErr);
                    Serial.println("] popped to make space.");
                #endif     
            }
            uint8_t full_err = PS_ERR_ERR_QUEUE_FULL;
            xQueueSend(errQueue, ( void * ) &full_err, (TickType_t ) 10) ;
            xQueueSend(errQueue, ( void * ) &error, (TickType_t ) 10) ;           
        } else {
            xQueueSend(errQueue, ( void * ) &error, (TickType_t ) 10) ;
        }
    };

    /// @brief Reads the error the [errQueue].
    /// @return Returns the next error code from the [errQueue] as ps_frame_t. 
    ///         Returns 0x00 if the [errQueue] is empty.
    uint8_t PacketSerial::readError(){
        uint8_t retVal = 0x00;
        xQueueReceive(errQueue, &(retVal), ( TickType_t ) 10 ) ;
        return retVal;

    }

    /// @brief Create the serial RX queue.
    ps_err_t PacketSerial::create_rx_queue(){
        rxQueue = xQueueCreate(PS_RX_QUEUE_LENGTH, sizeof(ps_byte_array_t)); 
        if (rxQueue == NULL){
            onError(PS_ERR_RX_QUEUE_CREATE_FAIL);
            return PS_ERR_RX_QUEUE_CREATE_FAIL;
        }
        return PS_PASS;
    };

    /// @brief Create the serial TX queue.
    ps_err_t PacketSerial::create_tx_queue(){
        txQueue = xQueueCreate(PS_TX_QUEUE_LENGTH,sizeof(ps_byte_array_t));  // create the TX queue
         if (txQueue == NULL){
            onError(PS_ERR_TX_QUEUE_CREATE_FAIL);
            return(PS_ERR_TX_QUEUE_CREATE_FAIL);
        }
        return PS_PASS;
    };

        /// @brief Create the serial TX queue.
    ps_err_t PacketSerial::create_err_queue(){
        errQueue = xQueueCreate(PS_ERR_QUEUE_LENGTH,sizeof(uint8_t));  // create the TX queue
         if (errQueue == NULL){
            onError(PS_ERR_ERR_QUEUE_CREATE_FAIL);
            return(PS_ERR_ERR_QUEUE_CREATE_FAIL);
        }
        return PS_PASS;
    };

    
    /// @brief The task that processes with data received from the display on [serialPort].
    /// @param parameter NULL
   void PacketSerial::serial_rx_impl(void* _this){
        static_cast<PacketSerial*>(_this)->serial_rx();
    };

    /// @brief The task that processes with data sent to the display on [serialPort].
    /// @param parameter NULL
    void PacketSerial::serial_tx_impl(void* _this){
        static_cast<PacketSerial*>(_this)->serial_tx();
    };
    
    ps_err_t PacketSerial::start_rx_task(){
        // create the RX task
        if (xTaskCreatePinnedToCore(
            this->serial_rx_impl,
            PS_RX_TASK_NAME,
            PS_RX_STACK_SIZE,
            this,
            PS_TASK_PRIORITY,
            NULL,
            PS_CORE) == pdPASS){
                return PS_PASS;
            } 
        onError(PS_ERR_RX_TASK_START_FAIL);
        return PS_ERR_RX_TASK_START_FAIL;
           
    };

    ps_err_t PacketSerial::start_tx_task(){
        // create the TX task
        if(xTaskCreatePinnedToCore(
            this->serial_tx_impl,
            PS_TX_TASK_NAME,
            PS_TX_STACK_SIZE,
            this,
            PS_TASK_PRIORITY,
            NULL,
            PS_CORE) == pdPASS){
            return PS_PASS; 
        };  
        onError(PS_ERR_TX_TASK_START_FAIL);
        return PS_ERR_TX_TASK_START_FAIL;
    };

    #ifdef PS_DEBUG
    // Returns an address string from the [address].
    String PacketSerial::toHEX(uint8_t address){
    // prefix with "0x"
    String addressStr = "0x";
        if (address<16) {
            // add "0" if less than 16
            addressStr = addressStr + "0";
        }
        // add the address and return the string
        return addressStr+String(address, HEX);
    };
    
    void PacketSerial::printFrame(uint8_t data[], uint8_t dLen){
      Serial.println("-----------------------------------------");
        Serial.print("frame = [");
        for (uint8_t i = 0; i < dLen; i++){
        if (i>0){ 
            Serial.print(", ");  
        }
        Serial.print(toHEX(data[i]));
        }
        Serial.println("]");
    };
    #endif //PS_DEBUG




