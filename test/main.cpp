

#include <DWIN_T5L_Display.h>

#define SD_IS_TOUCH_DISPLAY true

#define TONE_OUTPUT_PIN 25              // GPIO 25 as PWM output
#define TONE_PWM_CHANNEL  0 
#define BACKLIGHT_OUTPUT_PIN 26              // GPIO 25 as PWM output
#define BACKLIGHT_PWM_CHANNEL  1 
#define PWM_RESOLUTION 8       // 16-bit resolution
#define PWM_FREQ 16000           // 1kHz, so we can achieve 16-bit resolution
const int PWM_MAX_DUTY_CYCLE = (int)(pow(2, PWM_RESOLUTION) - 1);

void init_buzzer();

/// @brief The valid headers that can be sent by the device.
static const std::vector<uint16_t> headers{0x5aa5};

/// @brief The valid headers that can be sent by the device.
static const std::vector<uint16_t> variables{0x5000};

/// @brief The hardware serial port to which the device is connected.
HardwareSerial displayPort(2);

/// @brief The device instance with its headers and serial port.
DWIN_T5L_Display display = DWIN_T5L_Display(headers, SD_IS_TOUCH_DISPLAY, variables, & displayPort);

/* Our device is a DWIN serial display that sends compass heading data.
/* The data is sent in frames consisting of 9 bytes in the sequence 
/* {HH, HL, DL, CD, AH, AL, PH, CH, CL }:
/* - HH and HL are the high and low bytes of the 16-bit header. The header is alwys
/*   0x5AA5 and is the only value in the [headers] vector.
/* - DL is the number of data bits following the DL byte.
/* - CD is the command bit. For values from the display's variable registers 
/*   its is always 0x83.
/* - AH and AL are the high and low bytes of the register address on the display
/*   that holds the compass heading. In our imaginary device the address is 0x5000.
/* - PH is a placeholder. The value is always 0x01.
/* - CH and CL are the high and low bytes of the compass heading in half-degrees.*/


uint16_t heading = -1;


TP_Event touchEvent;

void getTouchEvent(){
  if (display.touchEventAvailable()){
    while (display.touchEventAvailable()){
      touchEvent = display.readTouchEvent();
      #if SD_DEBUG
      touchEvent.print();
      #else
      Serial.println(touchEvent.type, HEX);
      #endif
    }
  }

}

uint16_t headingTimestamp = 0;

bool getHeading(){
  Serial.print("");
  ps_frame_t frame = display.read();
  sd_reg_value_16_t value = display.readVariable(0x5000);
  if (value.timestamp != headingTimestamp){
    headingTimestamp = value.timestamp;
    Serial.println("0x" + String(value.address, HEX) + 
      ", " + String(value.timestamp) + ", " +
      String(value.value));
    heading = 360 - uint16_t( (value.value)/2);
    return true;
  // // #if PS_DEBUG
  // //   Serial.print("Frame: { ");
  // //   frame.print();
  // //   Serial.println(" }");
  // // #endif //PS_DEBUG
  // uint16_t address = ((frame.data[1]) << 8) | frame.data[2];
  // if (address == 0x5000){
  //    heading = uint16_t(360 - (((frame.data[4]) << 8) | frame.data[5]) / 2);
  // }
  }
  return false;
}

void get_System_Config(){
  ps_frame_t frame = PS_FRAME(0x5aa5, 4, {0x83, 0x00, 0x80, 0x02});
  display.write( &frame);

}


uint16_t bitmap[]={
      0x00F0, 0x00F0, 0xf800, 
      0x00F1, 0x00F0, 0xf800, 
      0x00F2, 0x00F0, 0xf800, 
      0x00F3, 0x00F0, 0xf800, 
      0x00F0, 0x00F1, 0xf800, 
      0x00F1, 0x00F1, 0xf800, 
      0x00F2, 0x00F1, 0xf800, 
      0x00F3, 0x00F1, 0xf800, 
      0x00F0, 0x00F2, 0xf800, 
      0x00F1, 0x00F2, 0xf800, 
      0x00F2, 0x00F2, 0xf800, 
      0x00F3, 0x00F2, 0xf800, 
      0x00F0, 0x00F3, 0xf800, 
      0x00F1, 0x00F3, 0xf800, 
      0x00F2, 0x00F3, 0xf800, 
      0x00F3, 0x00F3, 0xf800, 

      0x00E0, 0x00E0, 0x0500, 
      0x00E1, 0x00E0, 0x0500, 
      0x00E2, 0x00E0, 0x0500, 
      0x00E3, 0x00E0, 0x0500, 
      0x00E0, 0x00E1, 0x0500, 
      0x00E1, 0x00E1, 0x0500, 
      0x00E2, 0x00E1, 0x0500, 
      0x00E3, 0x00E1, 0x0500, 
      0x00E0, 0x00E2, 0x0500, 
      0x00E1, 0x00E2, 0x0500, 
      0x00E2, 0x00E2, 0x0500, 
      0x00E3, 0x00E2, 0x0500, 
      0x00E0, 0x00E3, 0x0500, 
      0x00E1, 0x00E3, 0x0500, 
      0x00E2, 0x00E3, 0x0500, 
      0x00E3, 0x00E3, 0x0500, 
      
      0x00D0, 0x00D0, 0x001F, 
      0x00D1, 0x00D0, 0x001F, 
      0x00D2, 0x00D0, 0x001F, 
      0x00D3, 0x00D0, 0x001F, 
      0x00D0, 0x00D1, 0x001F, 
      0x00D1, 0x00D1, 0x001F, 
      0x00D2, 0x00D1, 0x001F, 
      0x00D3, 0x00D1, 0x001F, 
      0x00D0, 0x00D2, 0x001F, 
      0x00D1, 0x00D2, 0x001F, 
      0x00D2, 0x00D2, 0x001F, 
      0x00D3, 0x00D2, 0x001F, 
      0x00D0, 0x00D3, 0x001F, 
      0x00D1, 0x00D3, 0x001F, 
      0x00D2, 0x00D3, 0x001F, 
      0x00D3, 0x00D3, 0x001F, 
         
      0x00A0, 0x00A0, 0x001F, 
      0x00A1, 0x00A0, 0x001F, 
      0x00A2, 0x00A0, 0x001F, 
      0x00A3, 0x00A0, 0x001F, 
      0x00A0, 0x00A1, 0x001F, 
      0x00A1, 0x00A1, 0x001F, 
      0x00A2, 0x00A1, 0x001F, 
      0x00A3, 0x00A1, 0x001F, 
      0x00A0, 0x00A2, 0x001F, 
      0x00A1, 0x00A2, 0x001F, 
      0x00A2, 0x00A2, 0x001F, 
      0x00A3, 0x00A2, 0x001F, 
      0x00A0, 0x00A3, 0x001F, 
      0x00A1, 0x00A3, 0x001F, 
      0x00A2, 0x00A3, 0x001F, 
      0x00A3, 0x00A3, 0x001F, 
};

void drawPixels(uint16_t address, uint32_t pixels, uint16_t * bitmap){
    uint32_t words = pixels * 6;
    uint16_t pixelCounter = 0;
    uint32_t wordCounter = 0;
    uint8_t maxPixels = 0x20;
    uint8_t frameLengthCounter = 0x07;
    uint16_t header = headers[0];
    uint8_t command = 0x82;
    // std::vector<uint8_t> pixelBytes;
    uint16_t termination = 0xff00;
    ps_frame_t frame = PS_FRAME(0x5aa5, 0, 
      {0x82, highByte(address), lowByte(address), 0x00, 0x01});
  
   for (uint32_t i = 0; i< pixels; i ++){
      uint16_t x = bitmap[i*3];
      uint16_t y = bitmap[i*3 + 1];
      uint16_t c = bitmap[i*3+2];
      frame.data.insert(frame.data.end(), {highByte(x), lowByte(x),
          highByte(y), lowByte(y),
          highByte(c), lowByte(c)});
      pixelCounter++;
      Serial.println("pixelCounter: " + String(pixelCounter));
      if ((i == pixels - 1) || pixelCounter==0x20){
        frame.data.insert(frame.data.begin()+5,{highByte(pixelCounter), lowByte(pixelCounter)});
        if (i == pixels){
        frame.data.insert(frame.data.end(),{0xff, 0x00});}
        frame.length = frame.data.size();
        frame.print();        
        Serial.println();
        address += pixelCounter;
        // Serial.println(frame.length,HEX);
        display.write(&frame);
        frame = PS_FRAME(0x5aa5, 0, 
          {0x82, highByte(address), lowByte(address), 0x00, 0x01});
        pixelCounter = 0;
      }
   }
    // ps_frame_t frame = PS_FRAME(0x5aa5, 0xC9,{0x82, 0x54, 0x40, 
    //   0x00, 0x01, 0x02, 0x00,
    //   0x00, 0xE0, 0x00, 0xE0, 0x05, 0x00, 
    //   0x00, 0xE1, 0x00, 0xE0, 0x05, 0x00, 
    //   0x00, 0xE2, 0x00, 0xE0, 0x05, 0x00, 
    //   0x00, 0xE3, 0x00, 0xE0, 0x05, 0x00, 
    //   0x00, 0xE0, 0x00, 0xE1, 0x05, 0x00, 
    //   0x00, 0xE1, 0x00, 0xE1, 0x05, 0x00, 
    //   0x00, 0xE2, 0x00, 0xE1, 0x05, 0x00, 
    //   0x00, 0xE3, 0x00, 0xE1, 0x05, 0x00, 
    //   0x00, 0xE0, 0x00, 0xE2, 0x05, 0x00, 
    //   0x00, 0xE1, 0x00, 0xE2, 0x05, 0x00, 
    //   0x00, 0xE2, 0x00, 0xE2, 0x05, 0x00, 
    //   0x00, 0xE3, 0x00, 0xE2, 0x05, 0x00, 
    //   0x00, 0xE0, 0x00, 0xE3, 0x05, 0x00, 
    //   0x00, 0xE1, 0x00, 0xE3, 0x05, 0x00, 
    //   0x00, 0xE2, 0x00, 0xE3, 0x05, 0x00, 
    //   0x00, 0xE3, 0x00, 0xE3, 0x05, 0x00, 
    //   0xff, 0x00});   
      // display.write(&frame);
}

void drawLine(){
   ps_frame_t frame = PS_FRAME(0x5aa5, 0x17,{0x82, 0x54, 0x40, 
      0x00, 0x02, 0x00, 0x02,
      0xF8, 0x00, 0x00, 0xFC, 0x01, 0x68, 
      0x01, 0x5E, 0x01, 0x68, 0x01, 0x5E, 0x01, 0x36, 
      0xff, 0x00});   
      frame.print(); 
      display.write(&frame);
}

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

    #if PS_DEBUG
    // The [begin] method returns PS_PASS (0x00) if it completes successfully.
    while (error != PS_PASS){

      // Read all errors from the device error queue and print to serial monitor
      error = display.readError();
      Serial.print("The error code is "); Serial.println(error, HEX);
    }
    #endif // PS_DEBUG
  } else {
      // set the flag
    displayready = true; 
    // init_buzzer();
    // ledcSetup(TONE_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    // /* Attach the LED PWM Channel to the GPIO Pin */
    // ledcAttachPin(TONE_OUTPUT_PIN, TONE_PWM_CHANNEL);
    
    //   ledcSetup(BACKLIGHT_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    // /* Attach the LED PWM Channel to the GPIO Pin */
    // ledcAttachPin(BACKLIGHT_OUTPUT_PIN, TONE_PWM_CHANNEL);
    #if SD_DEBUG    
    display.t5LSystemConfig().print();
    #endif // PS_DEBUG
    display.setOrientation(DisplayOrientation_180); 
    // display.autoUpload(false);
    // display.beepEnabled(false);
    display.setTimeout(30, 50); // dim to 10% after 30 seconds of inactivity
    // display.setBrightness(1);
     #if SD_DEBUG   
    display.t5LSystemConfig().print();
    #endif // PS_DEBUG
    // Do other stuff if the display initialized succesfully
    
    // Disable Incremental Adjustment touch control [0x00] on page [0x01]
    // display.touchControlEnabled(false,0x0001,0x0002); 
    delay(2500);    
    // display.autoUpload(true);    
    // display.beepEnabled(true);
    // display.setBrightness(100);
    // delay(1000); //  wait for settings to be sent
    display.t5LSystemConfig().print();    
    display.setPage(1);
    // drawPixels(0x5440,0x20,bitmap);
    // drawLine();
    delay(1000);
    pinMode(26,OUTPUT);
    digitalWrite(26, HIGH);
    
  }
}

void loop() {
  
  // long delay alows testing of exception handler
  delay(50);                          
  
  uint8_t error = 0xff;                 // placeholder for errors

  /* Read errors from display regularly or the error queue will fill up, 
  /* causing error codes to be popped off the buffer. */
   #if PS_DEBUG
  while (error != PS_PASS){             
    error = display.readError();        // read errors until 0x00 is returned
    if (error>0){                       // print the error
      Serial.print("Exception [");
      Serial.print(error, HEX);
      Serial.println("] was thrown.");
    }
  }
  # endif //PS_DEBUG
  /* Check for data from display regularly or the RX queue will fill up, 
  /* causing frames to be lost. The default queue is only 5 frames.*/
  #if SD_IS_TOUCH_DISPLAY
  getTouchEvent();
  #endif

  if (getHeading()){
    Serial.print("Heading: ");              // print the heading value
    Serial.println(heading); 
  }
  // if (display.available()){
  //   while(display.available()>0){
  //     getHeading();
  //     if (heading<361 && heading >=0)
  //     {
  //       Serial.print("Heading: ");              // print the heading value
  //       Serial.println(heading);      
  //     }
  //   }
  // }
 
}



