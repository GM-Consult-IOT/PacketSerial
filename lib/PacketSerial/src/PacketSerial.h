
/*!
* @file /*!
* @file PacketSerial.h
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

#include <PacketSerial_Typedef.h>
#include <Arduino.h>
#include <vector>
#include <HardwareSerial.h>

#ifndef __PLATFORM_IS_ESP32__
    #include "freeRTOS/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/queue.h"
#endif //__PLATFORM_IS_ESP32__



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
    uint8_t data[256];
    } ps_byte_array_t;

/// @brief Enumeration of PacketSerial errors (range 0x00 - 0x0f).
typedef enum PS_ERR{
    
    /// @brief The operation completed without error
    PS_PASS  = 0x00,

    // @brief One or more errors occurred during execution of the [begin] method.
    PS_ERR_START_UP_FAIL  = 0x01,

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
/// If [address] is 0x0000 then the packet is considered empty/null.
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
    }

} ps_frame_t;    

class PacketSerial{
    
public:
    
/// @brief Instantiates a [PacketSerial] instance.
/// @param headers_ 
/// @param port 
PacketSerial(std::vector<uint16_t> headers_,
    #ifdef PS_USE_SOFTWARE_SERIAL
        SoftwareSerial * port
    #else
        HardwareSerial * port
    #endif
);
// virtual ~PacketSerial();

/// @brief Initializes the PacketSerial.
virtual ps_err_t begin();

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

#ifdef PS_USE_SOFTWARE_SERIAL
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
virtual void onSerialRx(ps_frame_t * frame);

/// @brief Called whenever a new frame is transmitted to the serial port.
///
/// The base class implementation sends the frame to the rxQueue. Implementing
/// classes can override [onSerialRx] to filter the frames or populate device
/// properties (e.g. configuration) from the received frame.
///
/// @param frame The frame sent to the display.
virtual void onSerialTx(ps_frame_t * frame);

/// @brief called when the [begin] method completes.
virtual void onStartup();


#ifdef PS_DEBUG
/// @brief Returns an address string from the [address].
String toHEX(uint8_t address);

/// @brief Prints a frame to the debug serial port.
void printFrame(uint8_t data[], uint8_t dLen);
#endif //PS_DEBUG

/// @brief Call [onError] to send the error code to the errQueue.
///
/// The [error] is sent to the errQueue. If the [errQueue] is full, the oldest error 
/// is popped before the new [error] is pushed.
///
/// In debug mode the error is also printed to the serial monitor.
/// @param error The [PS_ERR] error code as uint8_t.
void onError(uint8_t error);

private:


/// @brief  Returns true if the [header] is in the [headers] list.
/// @param header The header to validate.
/// @return true if the [header] is in the [headers] list.
ps_err_t headerValid(ps_byte_array_t * byte_array);


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

};

