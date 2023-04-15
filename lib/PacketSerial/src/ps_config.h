

#ifndef PS_CONFIG

#define PS_CONFIG

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

    /// @brief The maximum length of a data packet.
    #ifndef MAX_FRAME_LENGTH
    #define MAX_FRAME_LENGTH 255
    #endif // MAX_FRAME_LENGTH

    /// @brief The priority of the serial monitor tasks.
    const int PS_TASK_PRIORITY = 1;

    /// @brief The stack size for the serial port RX task.
    const int PS_RX_STACK_SIZE = 10800;

    /// @brief The stack size for the serial port TX task
    const int PS_TX_STACK_SIZE = 10800;

    /// @brief The length of the RX queue.
    /// 
    /// Increasing the queue length may require an increase in stack size. 
    const int PS_RX_QUEUE_LENGTH = 10;

    /// @brief The length of the TX queue.
    /// 
    /// Increasing the queue length may require an increase in stack size. 
    const int PS_TX_QUEUE_LENGTH = 10;

    /// @brief The length of the ERROR queue.
    /// 
    /// Increasing the queue length may require an increase in stack size. 
    const int PS_ERR_QUEUE_LENGTH = 255;

    /// @brief The default serial port speed.
    const int PS_BAUD = 115200;

    /// @brief The processor core that runs the serial port processes.
    const int PS_CORE = 1;

    /// @brief The name of the serial port RX task.
    const char PS_RX_TASK_NAME[] = "PS_RX_TASK";

    /// @brief The name of the serial port TX task.
    const char PS_TX_TASK_NAME[] = "PS_TX_TASK";

#endif // PS_CONFIG