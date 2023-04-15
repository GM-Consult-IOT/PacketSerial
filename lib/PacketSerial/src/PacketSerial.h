
/*!
* @file /*!
* @file PacketSerial.h
*
* @mainpage Serial/HMI Display Library
*
* @section intro_sec_Introduction
*
* An Arduino C++ library for interfacing with serial devices that communicate with
* data packets that start with a header word and length byte.
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

#ifndef PACKET_SERIAL

#define PACKET_SERIAL

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
    // #define PS_HWS_RX_GPIO 16               // The RX GPIO.
    // #define PS_HWS_TX_GPIO 17               // The TX GPIO.
    // #define PS_HWS_INVERT_LOGIC false       // Invert the pin logic to active high.
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


#if PS_DEBUG



#endif // PS_DEBUG


/// @brief Serial data frame consisting of up to 256 bytes.
///
/// A frame always starts with a header (two characters) followed
/// by a character that specifies the number of data bits to follow.
typedef struct PS_BYTE_ARRAY{
    uint8_t data[MAX_FRAME_LENGTH];
    
    PS_BYTE_ARRAY(){};

    PS_BYTE_ARRAY(std::vector<uint8_t> * v){
        for (uint8_t i = 0; i < v->size(); i++){
            data[i] = v->data()[i];
        }
    }

    #if PS_DEBUG
    void print(uint8_t dLen = MAX_FRAME_LENGTH){
        for (uint8_t i = 0; i < dLen; i++){
        if (i>0){ 
            Serial.print(", ");  
        }
        Serial.print(data[i] > 16? 
            "0x" + String(data[i], HEX): 
            "0x0" + String(data[i], HEX));
        }
    };
    # endif // PS_DEBUG
    
    } ps_byte_array_t;



/// @brief Enumeration of PacketSerial errors (range 0x00 - 0x0f).
typedef enum PS_ERR{
    
    /// @brief The operation completed without error
    PS_PASS  = 0x00,

    // @brief One or more errors occurred during execution of the [begin] method.
    PS_ERR_START_UP_FAIL  = 0xff,

    /// @brief The frame header is not in the [headers] list.
    PS_ERR_INVALID_HEADER = 0X02,

    /// @brief Failed to create [rxQueue]
    PS_ERR_RX_QUEUE_CREATE_FAIL = 0X03,

    /// @brief Failed to create [txQueue]
    PS_ERR_TX_QUEUE_CREATE_FAIL = 0X04,

    /// @brief Failed to create [errQueue]
    PS_ERR_ERR_QUEUE_CREATE_FAIL = 0X05,

    /// @brief Failed to add a frame to the [rxQueue]. The queue may be full.
    PS_ERR_RX_QUEUE_FULL = 0X06,

    /// @brief Failed to add a frame to the [txQueue]. The queue may be full.
    PS_ERR_TX_QUEUE_FULL = 0x07,

    /// @brief The errQueue is full.
    PS_ERR_ERR_QUEUE_FULL = 0x08,

    /// @brief Failed to start the [serial_rx] task.
    PS_ERR_RX_TASK_START_FAIL = 0X09,

    /// @brief Failed to start the [serial_tx] task.
    PS_ERR_TX_TASK_START_FAIL = 0X0A,

} ps_err_t;


/// @brief A datapacket exchanged with the display via serial communication.
/// If [header] is 0x0000 then the packet is considered empty/null.
typedef struct PS_FRAME{

    ps_header_t header;

    /// @brief  The number of words (bytes) after the byte count uint16_t.
    ps_length_t length;

    /// @brief The frame data as a uint8_t-array.
    std::vector<uint8_t> data;

    PS_FRAME(){};

    PS_FRAME(ps_header_t header_,  ps_length_t length_, std::vector<uint8_t> data_):
        header(header_),length(length_),data(data_) {};

    PS_FRAME(ps_byte_array_t * frame){
        header = ((frame->data[0]) << 8) | frame->data[1];
                length = frame->data[2];
                for (uint8_t i = 3; i<3+length; i++){
                    data.push_back(frame->data[i]);
                };
    };

    #if PS_DEBUG

    String _toHEX(uint8_t b){
        return b > 16? 
            "0x" + String(b, HEX): 
            "0x0" + String(b, HEX);
            };

    /// @brief Prints the frame all on one line with commas between the bytes.
    void print(){       
        Serial.print(_toHEX(highByte(header)));
        Serial.print(", ");
        Serial.print(_toHEX(lowByte(header)));
        Serial.print(", ");
        Serial.print(_toHEX(length));
        Serial.print(", ");
        ps_byte_array_t arr = PS_BYTE_ARRAY(&data);
        arr.print(data.size());
        
    };

    #endif // PS_DEBUG
} ps_frame_t;    

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

 ~PacketSerial(){};

/// @brief Initializes the PacketSerial.
virtual uint8_t begin();

/// @brief Returns true if the receive queue contains data.
/// @return true if the receive queue contains data.
UBaseType_t available(void);

/// @brief Reads the next frame from the [rxQueue].
/// @return Returns the next frame from the [rxQueue] as ps_frame_t. 
///         Returns an empty frame if the buffer is empty.
ps_frame_t read();

/// @brief Reads the error the [errQueue].
/// @return Returns the next error code from the [errQueue] as ps_frame_t. 
///         Returns 0x00 if the [errQueue] is empty.
uint8_t readError();

/// @brief Writes a 
/// @param lenD 
/// @param data 
uint8_t write (ps_frame_t * frame);

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
std::vector<uint16_t> headers;

/// @brief Called whenever a new frame is received from the serial port.
///
/// The base class implementation sends the frame to the rxQueue. Implementing
/// classes can override [onSerialRx] to filter the frames or populate device
/// properties (e.g. configuration) from the received frame.
///
/// @param frame The frame reveived from the display.
virtual uint8_t onSerialRx(ps_frame_t * frame);

/// @brief Called whenever a new frame is transmitted to the serial port.
///
/// The base class implementation sends the frame to the rxQueue. Implementing
/// classes can override [onSerialRx] to filter the frames or populate device
/// properties (e.g. configuration) from the received frame.
///
/// @param frame The frame sent to the display.
virtual void onSerialTx(ps_frame_t * frame);

/// @brief called when the [begin] method completes.
virtual uint8_t onStartup();


/// @brief Call [onError] to send the error code to the errQueue.
///
/// The [error] is sent to the errQueue. If the [errQueue] is full, the oldest error 
/// is popped before the new [error] is pushed.
///
/// In debug mode the error is also printed to the serial monitor.
/// @param error The [PS_ERR] error code as uint8_t.
void onError(uint8_t error);


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
ps_err_t headerValid(ps_byte_array_t * byte_array);

/// @brief Sends the frame to the rxQueue or txQueue. 
///
/// if the queue is full it will pop the oldest frame and push the new
/// frame, returning an error that frames were lost.
ps_err_t send_to_frame_queue(QueueHandle_t q, ps_byte_array_t * frame );

/// @brief Create the serial RX queue.
ps_err_t create_rx_queue();

/// @brief Create the serial TX queue.
ps_err_t create_tx_queue();

/// @brief Create the serial TX queue.
ps_err_t create_err_queue();

/// @brief Starts the the serial RX task.
ps_err_t start_rx_task();

/// @brief Starts the serial TX task.
ps_err_t start_tx_task();

/// @brief The task that processes with data received from the display on [serialPort].
/// @param parameter NULL
static void serial_rx_impl(void* _this);

/// @brief The task that processes with data sent to the display on [serialPort].
/// @param parameter NULL
static void serial_tx_impl(void* _this);

/// @brief Sets the bits in [oldValue] from [newValue] using the [mask].
/// @param oldValue The byte that will be changed.
/// @param newValue The new value from which the bits will be copied.
/// @param mask The mask used to copy bits from [newValue] to [oldValue].
/// @return A clone of [oldValue] with only the [mask] bits changed to match
/// [newValue].   
    uint8_t setBitValues(uint8_t oldValue, uint8_t newValue, uint8_t mask);

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

    /// @brief The length of the ERROR queue.
    /// 
    /// Increasing the queue length may require an increase in stack size. 
     uint8_t err_queue_length = PS_ERR_QUEUE_LENGTH;

    /// @brief The processor core that runs the serial port processes.
     uint8_t core = PS_CORE;

};



#endif //PACKET_SERIAL