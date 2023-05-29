
/*!
* @file PacketSerial.h
*
* @mainpage Serial/HMI Display Library
*
* @section intro_sec_Introduction
*
* An Arduino C++ library for interfacing with serial devices that communicate with
* data packets that start with a header word and length byte.
* 
* IMPORTANT:
* Configuration settings should be set directly in the /src/ps_config.h file. 
* Attempting to override the settings in your main.cpp is likely to lead to unexpected
* errors or failure to compile.
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

#ifndef PACKET_SERIAL   // HEADER GUARD, do not remove
#define PACKET_SERIAL   // HEADER GUARD, do not remove

#include "ps_config.h"

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <vector>

/*! @brief Loads the FreeRTOS libraries if the platform is not ESP32
*  The ESP32 platform includes FreeRTOS and the FreeRTOS libraries do
*  not have to be loaded seperately.*/
#if !PLATFORM_IS_ESP32
    #include "freeRTOS/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/queue.h"
#endif //__PLATFORM_IS_ESP32__

#if PS_USE_SOFTWARE_SERIAL
    #include <SoftwareSerial.h>             // featherfly/SoftwareSerial@^1.0
#else
    #include <HardwareSerial.h>
#endif // PS_USE_SOFTWARE_SERIAL


/// @brief The number of data characters in the frame.
///
/// Data characters start at index 3 of the data frame (after 
/// the header and length bytes). The sd_frame_length is always
/// 3 less than the total number of characters transmitted in the frame.
/// and cannot exceed 253 (the maximum length of a frame).
typedef uint8_t ps_length_t;

/// @brief The first two characters of a frame.
typedef uint16_t ps_header_t;

/// @brief Serial data frame consisting of up to 256 bytes.
///
/// A frame always starts with a header (two characters) followed
/// by a character that specifies the number of data bits to follow.
typedef struct PS_BYTE_ARRAY{

    uint8_t data[MAX_FRAME_LENGTH];

    /// @brief The number of data characters used.
    uint8_t length;

    /// @brief Returns a 16-bit word from the first two bytes in [data].
    /// @return A 16-bit word from the first two bytes in [data].
    ps_header_t header(){
        return data[0]<<8 | data[1];
    };
    
    PS_BYTE_ARRAY(){};

    PS_BYTE_ARRAY(uint8_t length_, uint8_t * data_){        
        // Serial.println("Header: 0x" + String(header_, HEX));
        length = length_;
        // data[0] = (header_ >> 8) & 0xff;        
        // // Serial.println("data[0]: 0x" + String(data[0], HEX));
        // data[1] = header_ & 0xff;        
        // Serial.println("data[1]: 0x" + String(data[1], HEX));
        for (uint8_t i = 0; i < length_; i++){
            data[i] = data_[i];
        }

    }

    #if PS_DEBUG
    void print(){
        for (uint8_t i = 0; i < length; i++){
        if (i>0){ 
            Serial.print(", ");  
        }
        Serial.print(data[i] > 0x0f? 
            "0x" + String(data[i], HEX): 
            "0x0" + String(data[i], HEX));
        }
    };
    # endif // PS_DEBUG
    
    } ps_byte_array_t;


/*!
* @brief Serial communication wrapper for interfacing with serial devices 
* that use data packets that start with a header word and length byte.
*/
class PacketSerial{
    
public:
    
/// @brief Instantiates a [PacketSerial] instance.
/// @param headers_ Valid headers that are placed at the start of a serial data packet.
/// @param port The serial port that the class instance will use.
PacketSerial(std::vector<uint16_t> headers_,
            #if PS_USE_SOFTWARE_SERIAL
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


/// @brief Initializes the PacketSerial.
uint8_t begin(){
    bool retVal = true;
    while (retVal){
    #if PS_DEBUG
    #endif // PS_DEBUG
    retVal = retVal && create_rx_queue();
    retVal = retVal && create_tx_queue();
    retVal = retVal && start_rx_task();
    retVal = retVal && start_tx_task();        
    return onStartup();            
    }
    return false;
};

/// @brief Returns true if the receive queue contains data.
/// @return the number of items in the queue.
uint8_t available(void);

/// @brief Reads the next frame from the [rxQueue].
/// @return Returns the next frame from the [rxQueue] as ps_frame_t. 
///         Returns an empty frame if the buffer is empty.
bool read(ps_byte_array_t & packet);

/// @brief Writes a frame to the txQueue for transmitting by the [serial_tx] task.
/// @param frame the frame that will be transmitted.
bool write (ps_byte_array_t * frame);

/// @brief Returns the array of valid headers used by the PacketSerial instance.
/// @return The array of valid headers used by the PacketSerial instance.
uint16_t * getHeaders();

protected:

#if PS_USE_SOFTWARE_SERIAL
    SoftwareSerial * ps_serial_port;
#else
    HardwareSerial * ps_serial_port;
#endif // PS_USE_SOFTWARE_SERIAL

/// @brief The valid headers used by the PacketSerial instance.
std::vector<ps_header_t> headers;

/// @brief Called whenever a new frame is received from the serial port.
///
/// The base class implementation sends the frame to the rxQueue. Implementing
/// classes can override [onSerialRx] to filter the frames or populate device
/// properties (e.g. configuration) from the received frame.
///
/// Frames can be filtered by return false.
///
/// @param frame The frame received from the display.
/// @return false will prevent a frame from being sent to the frame receive queue.
virtual bool onSerialRx(ps_byte_array_t * frame);

/// @brief Called whenever a new frame is transmitted to the serial port.
///
/// The base class implementation sends the frame to the rxQueue. Implementing
/// classes can override [onSerialRx] to filter the frames or populate device
/// properties (e.g. configuration) from the received frame.
///
/// @param frame The frame sent to the display.
virtual void onSerialTx(ps_byte_array_t * frame);

/// @brief called when the [begin] method completes.
virtual bool onStartup();

/// @brief The queue containing frames received from the display.
QueueHandle_t txQueue;

/// @brief The queue containing frames to be sent to the display.
QueueHandle_t rxQueue;

/// @brief The queue containing frames to be sent to the display.
QueueHandle_t errQueue;

/// @brief The task that processes with data received from the display on [serialPort].
/// @param parameter NULL
void serial_rx(void);

/// @brief The task that processes with data sent to the display on [serialPort].
/// @param parameter NULL
void serial_tx(void);

/// @brief  Returns true if the [header] is in the [headers] list.
/// @param header The header to validate.
/// @return true if the [header] is in the [headers] list.
bool headerValid(ps_header_t header);

/// @brief Sends the frame to the rxQueue. 
///
/// if the queue is full it will pop the oldest frame and push the new
/// frame, returning an error that frames were lost.
bool send_to_rx_queue(ps_byte_array_t * frame );

/// @brief Sends the frame to the txQueue. 
///
/// if the queue is full it will pop the oldest frame and push the new
/// frame, returning an error that frames were lost.
bool send_to_tx_queue(ps_byte_array_t * frame );

/// @brief Create the serial RX queue.
bool create_rx_queue();

/// @brief Create the serial TX queue.
bool create_tx_queue();

/// @brief Starts the the serial RX task.
bool start_rx_task();

/// @brief Starts the serial TX task.
bool start_tx_task();

/// @brief The static delegate of [serial_rx].
/// @param parameter NULL
static void serial_rx_impl(void* _this);

    /// @brief The static delegate of [serial_tx].
    /// @param _this NULL
    static void serial_tx_impl(void* _this);

    /// @brief Sets the bits in [oldValue] from [newValue] using the [mask].
    /// @param oldValue The byte that will be changed.
    /// @param newValue The new value from which the bits will be copied.
    /// @param mask The mask used to copy bits from [newValue] to [oldValue].
    /// @return A clone of [oldValue] with only the [mask] bits changed to match
    /// [newValue].   
    static uint8_t setBitValues(uint8_t oldValue, uint8_t newValue, uint8_t mask);

private:

/// @brief The priority of the serial monitor tasks.
    uint8_t task_priority = PS_TASK_PRIORITY;

/// @brief The stack size for the serial port TX task
    uint16_t stack_size = PS_STACK_SIZE;

    /// @brief The length of the RX queue.
    /// 
    /// Increasing the queue length may require an increase in stack size. 
    uint8_t rx_queue_length = PS_RX_QUEUE_LENGTH;

    /// @brief The length of the TX queue.
    /// 
    /// Increasing the queue length may require an increase in stack size. 
    uint8_t tx_queue_length = PS_TX_QUEUE_LENGTH;

/// @brief The processor core that runs the serial port processes.
    uint8_t core = PS_CORE;

};



#endif //PACKET_SERIAL