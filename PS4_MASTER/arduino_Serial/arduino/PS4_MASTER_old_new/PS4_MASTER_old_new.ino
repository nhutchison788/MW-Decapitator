#include <PS4BT.h>
#include <usbhub.h>
#include<Servo.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>


USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd);
Servo driveRightVictor;
Servo driveLeftVictor;


//motorPins
short driveRightVictorPin = 8; //black
short driveLeftVictorPin = 2; //green


//pResistor
const int pResistor = A0;
int value;
int count = 0;

//serial bytes
byte inputByte[12];         // aa array to hold incoming data
bool arrayComplete = false;  // whether the array is complete

void setup() {
  driveRightVictor.attach(driveRightVictorPin);
  driveLeftVictor.attach(driveLeftVictorPin);

  Serial.begin(9600);
#if !defined(__MIPSEL__)
  while (!Serial);
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
  Serial.print(F("\r\nPS4 Bluetooth Library Started"));
}

void loop() {
                                                                            //begin serial stuff i added to main

    if (arrayComplete) {
    for(int x = 0; x < 12; x++)
    {
    Serial.println(inputByte[x]);
    }


    ////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////
    ///PUT YOUR CODE FOR MOTORS CONTROLLED BY SERIAL VALUES HERE
    ////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////
    

    for(int x = 0; x < 12; x++)
    {
      inputByte[x] = 0;
    }
    
    arrayComplete = false;
  }
                                                                                //end serial stuff i added to main
  
  Usb.Task();
  value = analogRead(pResistor);

//  if (value < 800) {
//    count++;
////    Serial.println(count);
//  }

  //  if (count == 1) { //end count = 220
  //    //autonomous
  //
  //    Serial.println(value);
  //  }


  if (PS4.connected()) { //end count = 425
    driveRightVictor.write(map(PS4.getAnalogHat(RightHatY), 0, 255, 135, 45));
    driveLeftVictor.write(map(PS4.getAnalogHat(LeftHatY), 255, 0, 135, 45));


    if (PS4.getButtonClick(PS)) {
      PS4.disconnect();
    }
  }
}


/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    byte count = 0;
    inputByte[count] = (byte)Serial.read();
    // add it to the inputString:
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inputByte[count] == 0xFF) {
      arrayComplete = true;
    }
    count ++;
  }
}
