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
Servo liftVictor;
Servo shooterVictor;
Servo gatherVictor;

short shooterVictorPin = 32; //blue
short liftVictorPin = 14; //white
short gatherVictorPin = 42; //


byte outputByte[12];                            //this array will be sent to the other bord


//int value;
//int count = 0;

void setup() {

  
outputByte[0] = 0xAA;                           //do not use the first byte. it is used to tell the master it is ready
outputByte[11] = 0xFF;                          //do not use the last byte. it is used to tell the master it is done

  
  shooterVictor.attach(shooterVictorPin);
  liftVictor.attach(liftVictorPin);
  gatherVictor.attach(gatherVictorPin);

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
  Usb.Task();

  if (PS4.connected()) {
    
    ////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////
    ///PUT YOUR CODE FOR MOTORS SPEEDS TO BE SENT BY SERIAL HERE
    ////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////

   liftVictor.write(map(PS4.getAnalogHat(RightHatY), 0, 255, 135, 45));
    gatherVictor.write(map(PS4.getAnalogHat(LeftHatY), 0, 255, 135, 45));


    //right trigger
    if (PS4.getButtonPress(R2)) {
      shooterVictor.write(135);
    } else {
      shooterVictor.write(90);
    }

    if (PS4.getButtonClick(PS)) {
      PS4.disconnect();
    }
  }
  for(int x = 0; x < 12; x++)
  {
    delay(30);                                                    //this delay has to be here, you should play with its value until it works for you.
    Serial.write(outputByte[x]);
  }
}
