
#define PS_DEBUG true

#include <PacketSerial.h>

/// @brief The valid headers that can be sent by the device.
static const std::vector<uint16_t> Wt901headers{0x5550, // Time
                                           0x5551, // Acceleration
                                           0x5552, // Angular Velocity
                                           0x5553, // Angle
                                           0x5554, // Magnetic Flux
                                           0x5555, // Data Output Port Status
                                           0x5556, // Atmospheric Pressure                                           
                                           0x5557, // Longitude and Latitude                                  
                                           0x5558, // Ground Speed                
                                           0x5559, // Quaternion                                      
                                           0x555A, // Satellite Positioning Accuracy
                                           };

/// @brief The hardware serial port to which the device is connected.
HardwareSerial WT901B_Port(1);


/// @brief The device instance with its headers and serial port.
PacketSerial device = PacketSerial(Wt901headers, & WT901B_Port);

uint16_t heading = -1;

void getHeading(){
  
  // #if PS_DEBUG 

  //   lib.print();  

  // #endif      
  ps_byte_array_t frame = device.read();
  #if PS_DEBUG
  #endif //PS_DEBUG
  uint16_t header = ((frame.data[0]) << 8) | frame.data[1];
  if (header == 0x5553){
    Serial.print("Frame: { ");
    frame.print();
    Serial.println(" }");
     heading = 360 - uint16_t(((frame.data[7] << 8) | frame.data[6])* 180 / 32768);
  }

}


/// @brief A flag for when the display is successfully initialized.
bool displayready = false;

void setup() {

    // start serial coms with USB on UART0
  Serial.begin(115200);

    // start serial comms with display on UART2
  WT901B_Port.begin(9600,SERIAL_8N1, 12, 13);

  device.begin();

  // handshake for debugging
  Serial.println("Up and running...");

}

void loop() {
   while(device.available()>0){
      getHeading();
      // if (heading<361 && heading >=0)
      {
        Serial.print("Heading: ");              // print the heading value
        Serial.println(heading);      
      }
    }  
  delay(1000);
}

void _loop() {
  
  // long delay alows testing of exception handler
  delay(50);  

                    
  
  uint8_t error = 0xff;                 // placeholder for errors

  #if PS_DEBUG
  /* Read errors from display regularly or the error queue will fill up, 
  /* causing error codes to be popped off the buffer. */
  while (error != PS_PASS){             
    error = device.readError();        // read errors until 0x00 is returned
    if (error>0){                       // print the error
      Serial.print("Exception [");
      Serial.print(error, HEX);
      Serial.println("] was thrown.");
    }
  }
  #endif
  /* Check for data from display regularly or the RX queue will fill up, 
  /* causing frames to be lost. The default queue is only 5 frames.*/
  
  if (device.available()){
    while(device.available()>0){
      getHeading();
      // if (heading<361 && heading >=0)
      {
        Serial.print("Heading: ");              // print the heading value
        Serial.println(heading);      
      }
    }
  }
 
}