#include <Arduino.h>
#include <HardwareSerial.h>
#include <PacketSerial.h>




    // #define __PACKET_SERIAL_CONFIG__

        
    //     /* Comment out this next define if your MCU is not an ESP32.
    //     *  The ESP32 platform includes FreeRTOS and the FreeRTOS libraries do
    //     *  not have to be loaded seperately.*/
    //     #define __PLATFORM_IS_ESP32__

    //     /* Comment out this next define to silence debug printing to the
    //     * terminal.
    //     */
    //     /* Uncomment this define to use software serial. */
    //     // #define PS_USE_SOFTWARE_SERIAL

    //     #ifdef PS_USE_SOFTWARE_SERIAL
    //         #include <SoftwareSerial.h>             // featherfly/SoftwareSerial@^1.0
    //     #else
    //         #include <HardwareSerial.h>
    //         // #define PS_HWS_RX_GPIO 16               // The RX GPIO.
    //         // #define PS_HWS_TX_GPIO 17               // The TX GPIO.
    //         // #define PS_HWS_INVERT_LOGIC false       // Invert the pin logic to active high.
    //     #endif // PS_USE_SOFTWARE_SERIAL
        
    //     /// @brief The default serial port speed.
    //     #define PS_BAUD 115200

    //     /// @brief The processor core that runs the serial port processes.
    //     #define PS_CORE 0

    //     /// @brief The name of the serial port RX task.
    //     #define PS_RX_TASK_NAME "PS_RX_TASK"

    //     /// @brief The name of the serial port TX task.
    //     #define PS_TX_TASK_NAME "PS_TX_TASK"

    //     /// @brief The priority of the serial monitor tasks.
    //     #define PS_TASK_PRIORITY 1

    //     /// @brief The stack size for the serial port RX task.
    //     #define PS_RX_STACK_SIZE 10800

    //     /// @brief The stack size for the serial port TX task
    //     #define PS_TX_STACK_SIZE 10800

    //     /// @brief The length of the RX queue.
    //     /// 
    //     /// Increasing the queue length may require an increase in stack size. 
    //     #define PS_RX_QUEUE_LENGTH 5

    //     /// @brief The length of the TX queue.
    //     /// 
    //     /// Increasing the queue length may require an increase in stack size. 
    //     #define PS_TX_QUEUE_LENGTH 5

    // /// @brief The length of the ERROR queue.
    //     /// 
    //     /// Increasing the queue length may require an increase in stack size. 
    //     #define PS_ERR_QUEUE_LENGTH 256

/// @brief The valid headers that can be sent by the device.
static const std::vector<uint16_t> headers{0x5aa5};

/// @brief The hardware serial port to which the device is connected.
HardwareSerial displayPort(2);

/// @brief The device instance with its headers and serial port.
PacketSerial display = PacketSerial(headers, & displayPort);

/* Our imaginary device is a DWIN serial display that sends compass heading data.
*  The data is sent in frames consisting of 9 bytes in the sequence 
*  {HH, HL, DL, CD, AH, AL, PH, CH, CL }:
*  - HH and HL are the high and low bytes of the 16-bit header. The header is alwys
*    0x5AA5 and is the only value in the [headers] vector.
*  - DL is the number of data bits following the DL byte.
*  - CD is the command bit. For values from the display's variable registers 
     its is always 0x83.
*  - AH and AL are the high and low bytes of the register address on the display
*    that holds the compass heading. In our imaginary device the address is 0x5000.
*  - PH is a placeholder. The value is always 0x01.
*  - CH and CL are the high and low bytes of the compass heading in half-degrees.
*  }

*/

uint16_t heading = -1;

void getHeading(){
  ps_frame_t frame = display.read();
  uint16_t address = ((frame.data[1]) << 8) | frame.data[2];
  if (address == 0x5000){
     heading = uint16_t(360 - (((frame.data[4]) << 8) | frame.data[5]) / 2);
  }

}

void displayReset(){
        ps_frame_t frame {headers[0], 7, {0x82, 0x00, 0x04, 0x55, 0xAA, 0x5A, 0xA5}};
        display.write(&frame);  
    };

void setPage(uint16_t page){
        uint8_t data[] = {0x82, 0x00, 0x84, 0x5a, 0x01, highByte(page), lowByte(page)};
        ps_frame_t frame {headers[0], 
                    7, 
                    {0x82, 0x00, 0x84, 0x5a, 0x01, highByte(page), lowByte(page)}};
        display.write(&frame);        
    };



/// @brief A flag for when the display is successfully initialized.
bool displayready = false;

void setup() {

    // start serial coms with USB on UART0
  Serial.begin(115200);

    // start serial comms with display on UART2
  displayPort.begin(115200);

  // handshake for debugging
  Serial.println("Up and running...");

  // initialize the display
  if (display.begin() != PS_PASS){
    uint8_t error = 0xff;

    // The [begin] method returns PS_PASS (0x00) if it completes successfully.
    while (error != PS_PASS){

      // Read all errors from the device error queue and print to serial monitor
      error = display.readError();
      Serial.print("The error code is "); Serial.println(error, HEX);
    }

  } else {
      // set the flag
      displayReset();
      delay(1000);
      displayready = true;
    // Do other stuff if the display initialized succesfully
    delay(2500);
    setPage(1);
  }
}


void loop() {
  
  // long delay alows testing of exception handler
  delay(50);                          
  
  uint8_t error = 0xff;                 // placeholder for errors

  /* Read errors from display regularly or the error queue will fill up, 
  /* causing error codes to be popped off the buffer. */
  while (error != PS_PASS){             
    error = display.readError();        // read errors until 0x00 is returned
    if (error>0){                       // print the error
      Serial.print("Exception [");
      Serial.print(error, HEX);
      Serial.println("] was thrown.");
    }
  }
  /* Check for data from display regularly or the RX queue will fill up, 
  /* causing frames to be lost. The default queue is only 5 frames.*/
  
  if (display.available()){
    while(display.available()>0){
      getHeading();
      if (heading<361 && heading >=0)
      {
        Serial.print("Heading: ");              // print the heading value
        Serial.println(heading);      
      }
    }
  }
 
}