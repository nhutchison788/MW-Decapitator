#include <PS4BT.h>
#include <PS3BT.h>
#include <usbhub.h>
#include <Servo.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>


//======================================================================
//================= Begin New Gyro Code =====================
//======================================================================
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
float _gyroDegrees;
int _gyroRotations = 0;
float _gyroOffset = 0;

float GetGyroX()
{
	uint8_t mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	uint16_t fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024)
	{
		// reset so we can continue cleanly
		mpu.resetFIFO();

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else if (mpuIntStatus & 0x02)
	{
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize)
			fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
	}

	float prev = _gyroDegrees;
	float dif = prev - (ypr[0] * 180 / M_PI);
	if (abs(dif) > 180)
	{
		if (dif < 0)
		{
			_gyroRotations--;
		}
		else
		{
			_gyroRotations++;
		}
	}

	_gyroDegrees = (ypr[0] * 180 / M_PI);

	return (_gyroDegrees + _gyroOffset) + (360 * _gyroRotations);
}
void ZeroGyroX()
{
	GetGyroX();
	_gyroOffset = -_gyroDegrees;
	_gyroRotations = 0;
}

//======================================================================
//================= END New Gyro Code =====================
//======================================================================

void halt();
void flags_red();
void flags_blue();

USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd);
PS3BT PS3(&Btd);
Servo driveRightVictor;
Servo driveLeftVictor;
Servo liftVictor;
Servo shooterVictor;
Servo gatherVictor;
Servo hookVictor;

//motorPins
short driveRightVictorPin = 11;
short driveLeftVictorPin = 2;
short shooterVictorPin = 28;
short liftVictorPin = 5;
short gatherVictorPin = 42;
short hookVictorPin = 14;

//pResistor
const int pResistor = A0; // Photoresistor at Arduino analog pin A0
int value;				  // Store value from photoresistor (0-1023)
int count = 0;
bool red = false;
bool blue = false;
bool oneMin = false;

void setup()
{
	driveRightVictor.attach(driveRightVictorPin);
	driveLeftVictor.attach(driveLeftVictorPin);
	shooterVictor.attach(shooterVictorPin);
	liftVictor.attach(liftVictorPin);
	gatherVictor.attach(gatherVictorPin);
	hookVictor.attach(hookVictorPin);
	pinMode(pResistor, INPUT); // Set pResistor - A0 pin as an input (optional)

	Serial.begin(115200);
#if !defined(__MIPSEL__)
	//  while (!Serial);
#endif
	if (Usb.Init() == -1)
	{
		//    Serial.print(F("\r\nOSC did not start"));
		while (1)
			; // Halt
	}
	//  Serial.print(F("\r\nPS4 Bluetooth Library Started"));


//======================================================================
//================= BEGIN New Gyro Code =====================
//======================================================================
	mpu.initialize();
	mpu.dmpInitialize();
	mpu.setDMPEnabled(true);
	packetSize = mpu.dmpGetFIFOPacketSize();
//======================================================================
//================= END New Gyro Code =====================
//======================================================================

}

void loop()
{

	Usb.Task();

	value = analogRead(pResistor);

	if (value < 700)
	{			 //photoResistor
		count++; // autonomous end count = 220
	}

	if (PS4.connected() && PS3.PS3Connected)
	{

		driveRightVictor.write(map(PS4.getAnalogHat(RightHatY), 0, 255, 135, 45));
		driveLeftVictor.write(map(PS4.getAnalogHat(LeftHatY), 255, 0, 135, 45));

		liftVictor.write(map(PS3.getAnalogHat(RightHatY), 0, 255, 135, 45));
		gatherVictor.write(map(PS3.getAnalogHat(LeftHatY), 0, 255, 135, 45));

		if (PS4.getButtonPress(L1))
		{
			hookVictor.write(135);
		}
		else if (PS4.getButtonPress(R1))
		{
			hookVictor.write(45);
		}
		else
		{
			hookVictor.write(90);
		}

		if (PS3.getButtonPress(R2))
		{
			shooterVictor.write(30);
		}
		else
		{
			shooterVictor.write(90);
		}

		//    if (PS4.getButtonClick(TRIANGLE)) {
		//      //      backward
		//      driveRightVictor.write(50);
		//      driveLeftVictor.write(135);
		//      delay(3300); //5700 for 5 squares
		//      //      stop
		//      halt();
		//
		//    }

		if (PS4.getButtonClick(CIRCLE))
		{
			red = true;
		}
		if (PS4.getButtonClick(CROSS))
		{
			blue = true;
		}
		if (PS4.getButtonClick(START) || PS4.getButtonClick(SELECT))
		{
			blue = false;
			red = false;
		}

		if (red == true)
		{				 // && (count > 10 && count < 240)) {
			flags_red(); //shooting works fine
		}
		if (blue == true)
		{				  // && (count > 10 && count < 240)) {
			flags_blue(); //shooting works fine
		}

		if (PS3.getButtonClick(PS))
		{
			PS3.disconnect();
		}
		if (PS4.getButtonClick(PS))
		{
			PS4.disconnect();
		}

		delay(.1);
	}
}

void flags_red()
{ //flags_red()
	//      aim
	driveLeftVictor.write(45);
	delay(300);
	driveLeftVictor.write(90);
	delay(100);
	//      shoot
	shooterVictor.write(30);
	delay(400);
	shooterVictor.write(90);
	//      center
	driveLeftVictor.write(135);
	delay(350);
	driveLeftVictor.write(90);
	delay(100);
	//  //      forward to flag
	//  driveRightVictor.write(132);
	//  driveLeftVictor.write(45);
	//  delay(2700);                    //fix
	//  halt();
	//      back up to disc
	//  driveRightVictor.write(45);
	//  driveLeftVictor.write(135);
	//  delay(2700);                    //fix
	//  halt();
}

void flags_blue()
{ //flags_blue()
	//      aim
	driveRightVictor.write(135);
	delay(300);
	driveRightVictor.write(90);
	delay(100);
	//      shoot
	shooterVictor.write(30);
	delay(400);
	shooterVictor.write(90);
	//      center
	driveLeftVictor.write(45);
	delay(350);
	driveLeftVictor.write(90);
	delay(100);
	//      forward to flag
	driveRightVictor.write(132);
	driveLeftVictor.write(45);
	delay(2700); //fix
	halt();
	//     back up to disc
	driveRightVictor.write(45);
	driveLeftVictor.write(135);
	delay(2700); //fix
	halt();
}

void disc_red()
{ //disc_red()
	//      turn right
	driveRightVictor.write(45);
	driveLeftVictor.write(45);
	gatherVictor.write(135);
//======================================================================
//================= BEGIN New Gyro Code =====================
//======================================================================
	//Run Zero GyroX so that your current heading sets to ZERO
	ZeroGyroX();
	//Run a while loop until you reach your desired degrees. Delay 10ms between checks so nothing gets overwhelmed.
	while(GetGyroX() < 90) { delay(10); }
//======================================================================
//================= END New Gyro Code =====================
//======================================================================

	//Removed the delay since this is no longer needed.
	//delay(1300); //check
	halt(); //halt shuts down the motors sicne we've reached 90 degrees.
	
	//      forward/flip
	driveRightVictor.write(132);
	driveLeftVictor.write(45);
	delay(2700); //fix
	halt();
	//      backward to wall
	gatherVictor.write(90);
	driveRightVictor.write(45);
	driveLeftVictor.write(135);
	delay(2300); //fix
	halt();
	//      turn left
	driveRightVictor.write(135);
	driveLeftVictor.write(135);
	delay(1500); //check
	halt();
}

void disc_blue()
{ //disc_blue()
	//      turn left
	driveRightVictor.write(135);
	driveLeftVictor.write(135);
	delay(1500); //check
	halt();
	//      forward/flip
	gatherVictor.write(135);
	driveRightVictor.write(132);
	driveLeftVictor.write(45);
	delay(2300); //fix
	halt();
	//      backward to wall
	gatherVictor.write(90);
	driveRightVictor.write(45);
	driveLeftVictor.write(135);
	delay(2300); //fix
	halt();
	//      turn right
	driveRightVictor.write(45);
	driveLeftVictor.write(45);
	delay(1300); //check
	halt();
}

void lowPlatform_red_side()
{ //lowPlatform_red_side()
	//      align w/ platform
	driveRightVictor.write(45);
	driveLeftVictor.write(135);
	delay(1300); //fix
	halt();
	//      turn left
	driveRightVictor.write(135);
	driveLeftVictor.write(135);
	delay(1500); //check
	halt();
	//      get on red platform
	driveRightVictor.write(45);
	driveLeftVictor.write(135);
	delay(1300); //fix
	halt();
}

void lowPlatform_blue_side()
{ //lowPlatform_blue_side()
	//      align w/ platform
	driveRightVictor.write(45);
	driveLeftVictor.write(135);
	delay(1000); //fix
	halt();
	//      turn right
	driveRightVictor.write(45);
	driveLeftVictor.write(45);
	delay(1300); //fix
	halt();
	//      get on blue platform
	driveRightVictor.write(45);
	driveLeftVictor.write(135);
	delay(1300); //fix
	halt();
} //lowPlatform_blue_side()

void lowPlatform_red()
{ //lowPlatform_red()
	//      turn
	driveRightVictor.write(135);
	driveLeftVictor.write(135);
	delay(1500); //fix
	halt();
	//      backwards
	driveRightVictor.write(45);
	driveLeftVictor.write(135);
	delay(2200); //fix
	halt();
} //lowPlatform_red()

void lowPlatform_blue()
{ //lowPlatform_blue()
	//      turn
	driveRightVictor.write(45);
	driveLeftVictor.write(45);
	delay(1300); //fix
	halt();
	//      backwards
	driveRightVictor.write(45);
	driveLeftVictor.write(135);
	delay(2300); //fix
	halt();
}

void get_bb()
{
	driveRightVictor.write(49);
	driveLeftVictor.write(135);
	delay(5700); //check
	halt();
}

void grab_bb()
{ //works
	//      hook up
	hookVictor.write(45);
	delay(900);
	hookVictor.write(90);
	delay(600);
	//      backward
	driveRightVictor.write(45);
	driveLeftVictor.write(135);
	delay(300);
	halt();
	//      hook down
	hookVictor.write(135);
	delay(800);
	hookVictor.write(90);
	delay(600);
	//      hook up
	hookVictor.write(45);
	delay(1100);
	hookVictor.write(90);
}

void park_bb()
{ //check distances
	//      forward
	driveRightVictor.write(132);
	driveLeftVictor.write(45);
	delay(2500);
	halt();
	//      turn
	driveRightVictor.write(135);
	driveLeftVictor.write(135);
	delay(1500);
	halt();
	//      backwards
	driveRightVictor.write(45);
	driveLeftVictor.write(135);
	delay(3300);
	halt();
}

void halt()
{
	driveRightVictor.write(90);
	driveLeftVictor.write(90);
	delay(500);
} //halt()

void blue_auto()
{
	flags_blue();
	//disc_blue();
	//backwards
	//lowPlatform_blue();
	blue = false;
}

void red_auto()
{
	flags_red();
	//disc_red();
	//backwards
	//lowPlatform_red();
	red = false;
}

void oneMin_auto()
{
	flags_red();
	/*
     flip front disc (red disc)
     turn left
     backward
     flip back disc (red_disc)
     backward
     turn left
     backward
     pick up buddy bot
  */
	get_bb();
	grab_bb();
	park_bb();
	oneMin = false;
}
