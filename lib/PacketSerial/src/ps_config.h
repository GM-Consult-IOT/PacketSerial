/*! IMPORTANT: Configuration settings should only be set in this file. 
* Attempting to override the settings in your main.cpp is likely to lead 
* to unexpected errors or failure to compile.
*/

#ifndef __PS_CONFIG__   // HEADER GUARD, do not remove
#define __PS_CONFIG__   // HEADER GUARD, do not remove

    #ifndef PS_DEBUG
    /// @brief Set to true to enable printing of debug information to Serial.
    #define PS_DEBUG true
    #endif // PS_DEBUG
        
    #ifndef PS_USE_SOFTWARE_SERIAL
    /// @brief Set to true to use software serial.
    #define PS_USE_SOFTWARE_SERIAL false
    #endif // PS_USE_SOFTWARE_SERIAL

    #ifndef PLATFORM_IS_ESP32
    /// @brief Set to true if the platform is ESP32.
    #define PLATFORM_IS_ESP32 true
    #endif // PLATFORM_IS_ESP32

/*! IMPORTANT: CHANGING ANY OF THE FOLLOWING SETTINGS SHOULD 
* NOT BE NECESSARY
* Proceed with caution
--------------------------------------------------------------
*/

    #ifndef MINIMUM_TASK_HIGHWATER_MARK
    /// @brief If the remaining stack falls below this value
    /// A warning will be printed to the SerialPort if PS_DEBUG 
    /// is true
    #define MINIMUM_TASK_HIGHWATER_MARK 0X200
    #endif //MINIMUM_TASK_HIGHWATER_MARK

    #ifndef MAX_FRAME_LENGTH
    /// @brief The maximum length of a data packet.
    #define MAX_FRAME_LENGTH 0xffU
    #endif // MAX_FRAME_LENGTH

    #ifndef PS_TASK_PRIORITY
    /// @brief The priority of the serial monitor tasks.
    #define PS_TASK_PRIORITY 1
    #endif // PS_TASK_PRIORITY

    #ifndef PS_STACK_SIZE
    /// @brief The stack size for the serial port RX task.
    #define PS_STACK_SIZE 0x900U
    #endif // PS_STACK_SIZE

    #ifndef PS_RX_QUEUE_LENGTH
    /// @brief The length of the RX queue.
    #define PS_RX_QUEUE_LENGTH 10
    #endif // PS_RX_QUEUE_LENGTH

    #ifndef PS_TX_QUEUE_LENGTH
    /// @brief The length of the TX queue.
    #define PS_TX_QUEUE_LENGTH 10
    #endif // PS_TX_QUEUE_LENGTH

    #ifndef PS_BAUD
    /// @brief The default serial port speed.
    #define PS_BAUD 115200
    #endif // PS_BAUD

    #ifndef PS_CORE
    /// @brief The processor core that runs the serial port processes.
    #define PS_CORE 1
    #endif // PS_CORE

    #ifndef PS_RX_TASK_NAME
    /// @brief The name of the serial port RX task.
    #define PS_RX_TASK_NAME "PS_RX_TASK"
    #endif // PS_RX_TASK_NAME

    #ifndef PS_TX_TASK_NAME
    /// @brief The name of the serial port TX task.
    #define PS_TX_TASK_NAME "PS_TX_TASK"
    #endif // PS_TX_TASK_NAME

    #endif