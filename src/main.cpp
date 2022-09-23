#include <Arduino.h>
#include <AccelStepper.h>
#include <EEPROM.h>
#include "Wire.h"
#include "AS5600.h"
//#include <HX710B.h>

// Probe endstops setup
const int Probend_pin = 11;
const int Probstart_pin = 26;

// VacuumSensor
const int IsSuck_pin = 5;

// Hall sensor setup
// SDA-33 SCL-34

// Linear EndPin
const int LinearEnd_pin = 12;
// Pump setup
const int Pump_pin = 15;

// stepper setup____________________________
// ROTOR Platform
const int dir1_pin = 9;
const int step1_pin = 10;
const int en1_pin = 17;
AccelStepper Platform_step(AccelStepper::DRIVER, step1_pin, dir1_pin, en1_pin);
//  Probe
const int dir2_pin = 0;
const int step2_pin = 1;
const int en2_pin = 7;
AccelStepper Probe_step(AccelStepper::DRIVER, step2_pin, dir2_pin, en2_pin);

// Linear
const int dir3_pin = 4;
const int step3_pin = 3;
const int en3_pin = 2;
AccelStepper Linear_step(AccelStepper::DRIVER, step3_pin, dir3_pin, en3_pin);

// DataVARS________________________________

// State of
bool suckcock = 0;	// Compressor
bool isGrabbed = 0; // is Grab? (from vacuum sensor)

// Calucation Constants
const int disCHAMB = 200 * 96 / 20 / 7; // steps(200)*reduction(4.8)/chambersCount(7)  //distance between Chambers
const int PSPG = 23.799 * PI / 200;		// Probe: mm per step

// Distance Data
long int Platform_pos = 0;
long int Probe_pos;
long int Linear_pos;

// EndStops Data
int Probe_homed = 0;
int Linear_homed = 0;

// Encoder Configuration
const int enc_scl = 16;
const int enc_sda = 13;
const int enc_dir = 6;
const float ang_offset = 86;
float enc_rawAngle;

AS5600 as5600;

// HX710B pressure_sensor; // Создаем объект

void setup()
{
	// Declare pins as Outputs
	pinMode(step1_pin, OUTPUT);
	pinMode(dir1_pin, OUTPUT);
	pinMode(en1_pin, OUTPUT);

	pinMode(dir2_pin, OUTPUT);
	pinMode(step2_pin, OUTPUT);
	pinMode(en2_pin, OUTPUT);

	pinMode(dir3_pin, OUTPUT);
	pinMode(step3_pin, OUTPUT);
	pinMode(en3_pin, OUTPUT);

	Platform_step.setMaxSpeed(200);
	Platform_step.setAcceleration(50);

	Probe_step.setMaxSpeed(100);
	Probe_step.setAcceleration(75);

	Linear_step.setMaxSpeed(100);
	Linear_step.setAcceleration(75);

	pinMode(Probend_pin, INPUT_PULLDOWN);
	pinMode(Probstart_pin, PULLDOWN);
	pinMode(LinearEnd_pin, INPUT_PULLDOWN);

	pinMode(Pump_pin, OUTPUT);

	Serial.begin(115200);
	Serial.println("StartWork");
	// pressure_sensor.begin(DOUT, SCLK); // Инициализируем датчик
	// I2c Begin
	Wire.begin(enc_sda, enc_scl);

	// Encoder Begin
	as5600.begin(enc_sda, enc_scl, enc_dir); //  set direction pin.
	as5600.setDirection(AS5600_CLOCK_WISE);	 // default, just be explicit.
}
float GetPlatformAngle()
{
	enc_rawAngle = as5600.rawAngle() * AS5600_RAW_TO_DEGREES - ang_offset;
	if (enc_rawAngle < 0)
	{
		return 360 + enc_rawAngle;
	}
	else
	{
		return enc_rawAngle;
	}
}
void ProbeHome()
{
	Probe_homed = 0;
	Probe_step.setSpeed(50);
	Probe_step.move(10);
	Probe_step.setSpeed(-50);
	if (digitalRead(Probstart_pin) == 0 && Probe_homed == 0)
	{
		Probe_step.runSpeed();
		if (digitalRead(Probstart_pin) == 1)
		{
			Probe_homed = 1;
			Probe_pos = 0;
		}
	}
}
// Start Compressor suck
void Grab(bool t)
{
	if (t == 1)
	{
		digitalWrite(Pump_pin, HIGH);

		if (analogRead(IsSuck_pin) > 500)
		{
			isGrabbed = 1;
		}
		else
		{
			isGrabbed = 0;
		}
	}
	else
	{
		digitalWrite(Pump_pin, LOW);
		isGrabbed = 0;
	}
}
int PlFMspeed;
float dbChamb;
void PlatformPos(float Ang)
{

	if ((abs(GetPlatformAngle() - Ang) > 1))
	{
		Platform_step.run();
		// Serial.println("Running");
	}
	else
	{
		Platform_step.stop();
		// Serial.println("STOP");
	}

	if (abs(Ang - GetPlatformAngle()) <= 180)
	{ // 30 - 60 = -30
		if (Ang - GetPlatformAngle() > 0)
		{
			Platform_step.setSpeed(-PlFMspeed);
		}
		else
		{
			Platform_step.setSpeed(PlFMspeed);
		}
	}
	else
	{
		if (Ang - GetPlatformAngle() > 0)
		{
			Platform_step.setSpeed(PlFMspeed);
		}
		else
		{
			Platform_step.setSpeed(-PlFMspeed);
		}
	}
	if (abs(GetPlatformAngle() - Ang) < 180)
	{
		dbChamb = abs(GetPlatformAngle() - Ang);
	}
	else
	{
		dbChamb = 360 - abs(GetPlatformAngle() - Ang);
	}
	if (dbChamb > 75)
	{
		PlFMspeed = 75;
	}
	if (dbChamb < 75)
	{
		PlFMspeed = dbChamb;
	}
	if (dbChamb < 30)
	{
		PlFMspeed = (30);
	}
	Serial.println(Platform_step.speed());

	// Platform_step.moveTo(POS);
	//  Serial.println(Platform_step.currentPosition());
}

void PlatformCont(int Chamb)
{

	if (Platform_step.currentPosition() != Platform_step.targetPosition())
	{
		Platform_step.run();
	}
	else
	{
		Platform_step.stop();
	}
	// Serial.println(Platform_step.currentPosition());
	switch (Chamb)
	{
	case 0:
		Platform_step.moveTo(0);
		// Serial.println(Platform_step.currentPosition());
		break;
	case 1:
		Platform_step.moveTo(Chamb * disCHAMB);
		// Serial.println(Platform_step.currentPosition());
		break;
	case 2:
		Platform_step.moveTo(Chamb * disCHAMB);
		// Serial.println(Platform_step.currentPosition());
		break;
	case 3:
		Platform_step.moveTo(Chamb * disCHAMB);

		break;
	case 4:
		Platform_step.moveTo(Chamb * disCHAMB);

		break;
	case 5:
		Platform_step.moveTo(Chamb * disCHAMB);

		break;
	case 6:
		Platform_step.moveTo(Chamb * disCHAMB);

		break;
	}
}

void LinearPos(int POS) // MM
{

	if (Linear_step.currentPosition() != Linear_step.targetPosition())
	{
		Linear_step.run();
	}
	else
	{
		Platform_step.stop();
	}
	if (POS < 55 && POS > 0)
	{
		Platform_step.moveTo(POS * (56.52 / 200));
	}
	if (POS <= 0)
	{
		Platform_step.moveTo(0);
	}
	if (POS >= 55)
	{
		Platform_step.moveTo(55 * (56.52 / 200));
	}
	// Serial.println(Platform_step.currentPosition());
}

// Homing Linear module position hhh
void LinearHome()
{
	Linear_homed = 0;
	Linear_step.setSpeed(50);
	Linear_step.move(10);
	Linear_step.setSpeed(-50);
	if (digitalRead(LinearEnd_pin) == 0 && Linear_homed == 0)
	{
		Linear_step.runSpeed();
		if (digitalRead(LinearEnd_pin) == 1)
		{
			Linear_homed = 1;
			Linear_step.setCurrentPosition(0);
		}
	}
}

void Pick()
{
	if (suckcock == 0)
	{
		Grab(1);
		Probe_step.setSpeed(50);
		Probe_step.runSpeed();

		// Serial.println("go DOWN");
		Probe_pos = Probe_step.currentPosition();
		if (digitalRead(Probend_pin) == 0)
		{
			Grab(1);
			Probe_step.move(15);
			Probe_step.runToPosition();
		}
	}
	if (digitalRead(Probend_pin) == 0 && suckcock == 0)
	{
		Grab(1);
		delay(100);

		// Serial.println("Pick");
		suckcock = 1;
	}
	if (suckcock == 1)
	{
		if (Probe_pos < PSPG * 60)
		{
			Probe_step.moveTo(-Probe_pos);
			Probe_step.run();
			// Serial.println("go UP");
		}
		else
		{
			Probe_step.moveTo(-PSPG * 60);
			Probe_step.run();
			// Serial.println("go UP");
		}
	}
}

double angle_from_sin_cos(double sinx, double cosx) // result in -pi to +pi range
{
	if (cosx > 1)
	{
		cosx = 1;
	}
	if (cosx < -1)
	{
		cosx = -1;
	}
	if (sinx > 1)
	{
		sinx = 1;
	}
	if (sinx < -1)
	{
		sinx = -1;
	}
	double ang_from_cos = acos(cosx);
	double ang_from_sin = asin(sinx);
	double sin2 = sinx * sinx;
	if (sinx < 0)
	{
		ang_from_cos = -ang_from_cos;
		if (cosx < 0) // both negative
			ang_from_sin = -PI - ang_from_sin;
	}
	else if (cosx < 0)
		ang_from_sin = PI - ang_from_sin;
	// now favor the computation coming from the
	// smaller of sinx and cosx, as the smaller
	// the input value, the smaller the error
	return (((1.0 - sin2) * ang_from_sin + sin2 * ang_from_cos) / 3.14 * 180) + 180;
}

int pos = 0;
void loop()
{

	// Serial.println(GetPlatformAngle());
	PlatformPos(0);
	// Switch between chambers
	/*switch (pos)
	{
	case 0:
		PlatformCont(1);
		if (Platform_step.distanceToGo() == 0)
		{
			pos++;
		}
		break;
	case 1:
		Pick();
		if (Probe_step.distanceToGo() == 0 && suckcock == 1)
		{
			pos++;
		}

		break;
	case 2:
		PlatformCont(0);
		if (Platform_step.distanceToGo() == 0)
		{
			pos = 0;
			suckcock = 0;
			Grab(0);
		}
		break;
	}*/
}
