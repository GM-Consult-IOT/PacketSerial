/*!
* @file /*!
* @file PacketSerial_Typedef.h
*
* @mainpage Library that sends and receives frames to a serial port. To
* change the default settings copy this section to your own code and include 
* BEFORE <frame_serial.h>.
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

#ifndef __FRAME_SERIAL__
#define __FRAME_SERIAL__

    
    /* Comment out this next define if your MCU is not an ESP32.
    *  The ESP32 platform includes FreeRTOS and the FreeRTOS libraries do
    *  not have to be loaded seperately.*/
    #define __PLATFORM_IS_ESP32__

    /* Comment out this next define to silence debug printing to the
    * terminal.
    */
    #define PS_DEBUG
    /* Uncomment this define to use software serial. */
    // #define PS_USE_SOFTWARE_SERIAL

    #ifdef PS_USE_SOFTWARE_SERIAL
        #include <SoftwareSerial.h>             // featherfly/SoftwareSerial@^1.0
    #else
        #include <HardwareSerial.h>
        // #define PS_HWS_RX_GPIO 16               // The RX GPIO.
        // #define PS_HWS_TX_GPIO 17               // The TX GPIO.
        // #define PS_HWS_INVERT_LOGIC false       // Invert the pin logic to active high.
    #endif // PS_USE_SOFTWARE_SERIAL
    
    /// @brief The default serial port speed.
    #define PS_BAUD 115200

    /// @brief The processor core that runs the serial port processes.
    #define PS_CORE 0

    /// @brief The name of the serial port RX task.
    #define PS_RX_TASK_NAME "PS_RX_TASK"

    /// @brief The name of the serial port TX task.
    #define PS_TX_TASK_NAME "PS_TX_TASK"

    /// @brief The priority of the serial monitor tasks.
    #define PS_TASK_PRIORITY 1

    /// @brief The stack size for the serial port RX task.
    #define PS_RX_STACK_SIZE 10800

    /// @brief The stack size for the serial port TX task
    #define PS_TX_STACK_SIZE 10800

    /// @brief The length of the RX queue.
    /// 
    /// Increasing the queue length may require an increase in stack size. 
    #define PS_RX_QUEUE_LENGTH 5

    /// @brief The length of the TX queue.
    /// 
    /// Increasing the queue length may require an increase in stack size. 
    #define PS_TX_QUEUE_LENGTH 5

   /// @brief The length of the ERROR queue.
    /// 
    /// Increasing the queue length may require an increase in stack size. 
    #define PS_ERR_QUEUE_LENGTH 256
    

#endif //__FRAME_SERIAL__

