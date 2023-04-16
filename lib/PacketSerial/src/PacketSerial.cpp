
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

/// @brief Returns the array of valid headers used by the PacketSerial instance.
/// @return The array of valid headers used by the PacketSerial instance.
uint16_t * PacketSerial::getHeaders(){
    return headers.data();
};

/// @brief Returns true if the receive queue contains data.
/// @return true if the receive queue contains data.
UBaseType_t  PacketSerial::available(void){        
    return uxQueueMessagesWaiting(rxQueue);
};

/// @brief Reads the next frame from the [rxQueue].
/// @return Returns the next frame from the [rxQueue] as ps_frame_t. 
/// Returns an empty frame if the buffer is empty.
ps_frame_t read();
ps_frame_t PacketSerial::read(){
        ps_frame_t msg;
    if (uxQueueMessagesWaiting(rxQueue)>0){
        ps_byte_array_t frame;
        if(xQueueReceive(rxQueue, &( frame ), ( TickType_t ) 10 ) == pdPASS ){               
            msg = PS_FRAME(&frame);
        }
    } 
    return msg;       
};

/// @brief Writes a frame to the txQueue for transmitting by the [serialTx] task.
/// @param frame the frame that will be transmitted.    
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

/// @brief Create the serial TX queue.
ps_err_t PacketSerial::create_err_queue(){
    errQueue = xQueueCreate(err_queue_length,sizeof(uint8_t));  // create the TX queue
        if (errQueue == NULL){
        onError(PS_ERR_ERR_QUEUE_CREATE_FAIL);
        return(PS_ERR_ERR_QUEUE_CREATE_FAIL);
    }
    return PS_PASS;
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

/// @brief Sends the frame to the rxQueue or txQueue. 
///
/// if the queue is full it will pop the oldest frame and push the new
/// frame, returning an error that frames were lost.
ps_err_t PacketSerial::send_to_frame_queue(QueueHandle_t q, ps_byte_array_t * frame ){
    ps_err_t error = PS_PASS;        
    if (uxQueueSpacesAvailable(q) <1) {
        ps_byte_array_t poppedFrame;
        while(uxQueueSpacesAvailable(q) < 1){
            xQueueReceive(q, &(poppedFrame), ( TickType_t ) 10 ) ;            
        }
        ps_err_t error = q == rxQueue? PS_ERR_RX_QUEUE_FULL: PS_ERR_TX_QUEUE_FULL;
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
uint8_t PacketSerial::onSerialRx(ps_frame_t * frame){
    return PS_PASS; 
};

/// @brief called when the [begin] method completes.
uint8_t PacketSerial::onStartup(){
    return PS_PASS; 
};

/// @brief Create the serial RX queue.
ps_err_t PacketSerial::create_rx_queue(){
    rxQueue = xQueueCreate(rx_queue_length, sizeof(ps_byte_array_t)); 
    if (rxQueue == NULL){
        onError(PS_ERR_RX_QUEUE_CREATE_FAIL);
        return PS_ERR_RX_QUEUE_CREATE_FAIL;
    }
    return PS_PASS;
};

/// @brief Create the serial TX queue.
ps_err_t PacketSerial::create_tx_queue(){       
    txQueue = xQueueCreate(tx_queue_length,sizeof(ps_byte_array_t));  // create the TX queue
        if (txQueue == NULL){
        onError(PS_ERR_TX_QUEUE_CREATE_FAIL);
        return(PS_ERR_TX_QUEUE_CREATE_FAIL);
    }
    return PS_PASS;
};

/// @brief The static delegate of [serial_rx].
/// @param parameter NULL
void PacketSerial::serial_rx_impl(void* _this){
    static_cast<PacketSerial*>(_this)->serial_rx();
};

/// @brief The static delegate of [serial_tx].
/// @param parameter NULL
void PacketSerial::serial_tx_impl(void* _this){
    static_cast<PacketSerial*>(_this)->serial_tx();
};

/// @brief Starts the the serial RX task.
ps_err_t PacketSerial::start_rx_task(){
    // create the RX task
    if (xTaskCreatePinnedToCore(
        this->serial_rx_impl,
        PS_RX_TASK_NAME,
        stack_size,
        this,
        task_priority,
        NULL,
        core) == pdPASS){
            return PS_PASS;
        } 
    onError(PS_ERR_RX_TASK_START_FAIL);
    return PS_ERR_RX_TASK_START_FAIL;
        
};

/// @brief Starts the serial TX task.
ps_err_t PacketSerial::start_tx_task(){
    // create the TX task
    if(xTaskCreatePinnedToCore(
        this->serial_tx_impl,
        PS_TX_TASK_NAME,
        stack_size,
        this,
        task_priority,
        NULL,
        core) == pdPASS){
        return PS_PASS; 
    };  
    onError(PS_ERR_TX_TASK_START_FAIL);
    return PS_ERR_TX_TASK_START_FAIL;
};

/// @brief Sets the bits in [oldValue] from [newValue] using the [mask].
/// @param oldValue The byte that will be changed.
/// @param newValue The new value from which the bits will be copied.
/// @param mask The mask used to copy bits from [newValue] to [oldValue].
/// @return A clone of [oldValue] with only the [mask] bits changed to match
/// [newValue].   
    uint8_t PacketSerial::setBitValues(uint8_t oldValue, uint8_t newValue, uint8_t mask){
    uint8_t retval = oldValue & ~mask;
    retval = retval | (newValue & mask);
    return (oldValue & ~mask) | (newValue & mask);
};


