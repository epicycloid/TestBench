// BenchTest_v1.ino
// 
// Framework for bench test configuration of prototype lathe hardware, consisting of:
// - Teensy 3.1 (or 3.2)  https://www.pjrc.com/store/teensy31.html & https://www.pjrc.com/store/teensy32_pins.html
// - Mounted on a Teensy 3* Proto Board  https://www.tindie.com/products/freto/teensy-3-breakout-board-and-shield/
// - ILI9341 TFT screen  https://www.pjrc.com/store/display_ili9341.html
// - 2 rotary encoders  PEC11-4215F-S24  http://www.adafruit.com/datasheets/pec11.pdf
// - 4 user interface / menu buttons  (push-to-break, e.g. normally closed)
// - E-Stop button (emergency stop)  (push-to-break, e.g. normally closed)
// - 2 Pololu DRV8825 stepper drivers  https://www.pololu.com/product/2133
// - 2 Vexta PX245-02AA-C8 stepper motors  http://www.interinar.com/public_docs/PK245-02AA.pdf
//		-> N.B. stepper motors 0.57A/phase, Vref for driver = current/2 = 0.285V on DRV8825
// - DC/DC voltage converter  Recom R-78C5.0-1.0  http://www.recom-power.com/pdf/Innoline/R-78Cxx-1.0.pdf
// 
// v1 -- Initial version, sending duplicate HW to Cary. 9-18-15
// v2 -- 
// v3 -- 
// v4 -- 
// 
// 
// 

/*
 * >>>>>>>>>>>>  Pin Assignments  <<<<<<<<<<<<
 * Teensy 3.x digital pins
 * >>>>>>>>>>>>  top side pins  <<<<<<<<<<<<
 *  0 -- (RX1) n/c
 *  1 -- (TX1) n/c
 *  2 -- eStop (emergency stop button, NC)
 *  3 -- SPINDLE_STEP_PIN
 *  4 -- SPINDLE_DIR_PIN
 *  5 -- SPINDLE_ENABLE_PIN
 *  6 -- SLIDEREST_STEP_PIN
 *  7 -- SLIDEREST_DIR_PIN
 *  8 -- SLIDEREST_ENABLE_PIN
 *  9 -- tftDC
 * 10 -- tftCS
 * 11 -- tftDOUT
 * 12 -- tftDIN
 * 13 -- tftSCK
 * 14 -- encoderLeftA
 * 15 -- encoderLeftB
 * 16 -- encoderLeftButton
 * 17 -- encoderRightA
 * 18 -- encoderRightB
 * 19 -- encoderRightButton
 * 20 -- menuLeft      [4x user interface buttons]
 * 21 -- menuLeftInner
 * 22 -- menuRightInner
 * 23 -- menuRight
 * >>>>>>>>>>>>  botton side pins  <<<<<<<<<<<<
 * 24 -- 
 * 25 -- sliderestLeftEnd (end stop)  [for convenience pins 25, 27 & 29 are adjacent on the Teensy 3* Proto Board]
 * 26 -- 
 * 27 -- sliderestRightEnd (end stop)
 * 28 -- 
 * 29 -- spindleIndex (end stop)
 * 30 -- 
 * 31 -- 
 * 32 -- 
 * 33 -- 
 * >>>>>>>>>>>>    <<<<<<<<<<<<
 */

//	-----------------------------------------------------------------
//	--------------------- included libraries ------------------------
//	-----------------------------------------------------------------

// Since I keep libraries in local sketch directory, should these be "..." instead of <...> ??
#include <AccelStepper.h>
#include <SPI.h>  // ILI9341.h tries to include SPI.h, but fails in the compile, so added it here
#include <ILI9341_t3.h>  // PJRC's TFT display  320 x 240
#include <Encoder.h>    // PJRC's encoder library

//	-----------------------------------------------------------------
//	--------------------- compiler directives -----------------------
//	-----------------------------------------------------------------

// Default is to use interrupts for encoders. Comment this line to use interrupts.
#define ENCODER_DO_NOT_USE_INTERRUPTS

// PRECISION is used for integer multiplication, to scale the division for motor calculations
#define	PRECISION	100

// Pin numbers for the two stepper motors - step, direction & enable
#define SPINDLE_STEP_PIN 3
#define SPINDLE_DIR_PIN 4
#define SPINDLE_ENABLE_PIN 5
#define SLIDEREST_STEP_PIN 6
#define SLIDEREST_DIR_PIN 7
#define SLIDEREST_ENABLE_PIN 8

// #define SPINDLE_CHIME_PIN <TBD>	// normally open (0) until opto slot blocked, then closed (1)

// For the PJRC TFT, these are the default
#define TFT_DC  9
#define TFT_CS 10


//	-----------------------------------------------------------------
//	--------------------- function prototypes -----------------------
//	--------------------- (may not be needed) -----------------------

// void setup();
// void loop();
// void ReadEncoders();
// void DoMotorCalcs();

//	-----------------------------------------------------------------
//	--------------------- constructors ------------------------------
//	-----------------------------------------------------------------

// Define two steppers and the pins they will use
// For spindleMotor: Mode = Driver, Step pin, Dir pin (Enable pin = SPINDLE_ENABLE_PIN = 5)
AccelStepper spindleMotor(AccelStepper::DRIVER, SPINDLE_STEP_PIN, SPINDLE_DIR_PIN);
// For sliderestMotor: Mode = Driver, Step pin, Dir pin (Enable pin = SLIDEREST_ENABLE_PIN = 8)
AccelStepper sliderestMotor(AccelStepper::DRIVER, SLIDEREST_STEP_PIN, SLIDEREST_DIR_PIN);

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);


//	-----------------------------------------------------------------
//	------------------- global variables ----------------------------
//	-----------------------------------------------------------------

// Adjust these two target values for test scenarios. For this simple version of the code keep values
// for  target1 > target2, since the code to check magnitude was ripped out to simplify example.
long target1 = 32000;
long target2 = 16451;

long ratio;
long rem;
long nextPos1;
long nextPos2;
long cumError;


// Residual variables from more complicated version of code that swaps motors based on the one with longer
// distance to go. Code removed, but assignments below still use these placeholders.
long bigTarget;
long smallTarget;

long smallSteps;
long bigSteps;
long smallDelta;
long smallStepsTaken = 0;
long smallAccumulate = 0;

//	-----------------------------------------------------------------
//	------------------- Arduino setup() -----------------------------
//	-----------------------------------------------------------------
void setup()
{
  // initialize serial:
  Serial.begin(250000);  // max speed in Arduino Serial Monitor window is now 250000
  while (!Serial) {} //wait until Serial Monitor window opens -- needed for Teensy, Micro & Leonardo
  Serial.println("Coordinated motor motion... ");

//	The enable pin on the DRV8825 is:
//		LOW = enabled
//		HIGH = disabled
//	
//	ENABLE: This input is pulled down on the board with a 100KΩ resistor. You can leave it disconnected, or you
//  can drive it from your microcontroller. When low, the driver chip is enabled and the motor is energized.
//  When high, the driver chip is still enabled, but all of the final motor drive circuits are disabled and so
//  no current will flow to the motor.
//	
//	Call functions in this order:
//		motorName.setEnablePin(whatever); 
//		motorName.setPinsInverted(false, false, true); 
//		motorName.enableOutputs();
//
//	Call motorName.disableOutputs(); to turn outputs off
// commented out for initial HW testing...
/*
	spindleMotor.setEnablePin(SPINDLE_ENABLE_PIN);
	spindleMotor.setPinsInverted(false, false, true);
	spindleMotor.enableOutputs();
	
	sliderestMotor.setEnablePin(SLIDEREST_ENABLE_PIN);
	sliderestMotor.setPinsInverted(false, false, true);
	sliderestMotor.enableOutputs();
*/
	

	// Try at spindleMotor.setMaxSpeeds() of 4000 & 10000 just to see timing. Adjust acceleration too.
	spindleMotor.setMaxSpeed(8000);
	spindleMotor.setAcceleration(5000);
	// Try various values for both speed and accelerations. Doesn't seem to matter if they are the same as stepper1.
  sliderestMotor.setMaxSpeed(8000);
	// stepper2.setAcceleration(10000);
	sliderestMotor.setSpeed(8000);
	
	spindleMotor.moveTo(target1);
	Serial.println();
	Serial.print("target1 = ");
	Serial.print(target1);
	Serial.print("   target2 = ");
	Serial.println(target2);

	
	ratio = target1 / target2;
	rem = target1 % target2;
	nextPos1 = ratio;
	cumError = rem;
	nextPos2 = 0;

}


//	-----------------------------------------------------------------
//	------------------- Arduino loop() ------------------------------
//	-----------------------------------------------------------------
void loop()
{
	static int once = 0;
	long curPos1;

  if (spindleMotor.distanceToGo() != 0 || sliderestMotor.currentPosition() < nextPos2)
  {
  	spindleMotor.run();
  	curPos1 = spindleMotor.currentPosition();
  	if (curPos1 >= nextPos1)
  	{
  		nextPos2++;
  		// sliderestMotor.moveTo(nextPos2);
			// sliderestMotor.setSpeed(8000);
  		nextPos1 += ratio;
  		cumError += rem;
  		
  		if (cumError > target2)
  		{
  			cumError -= target2;
  			nextPos1 ++;
  		}
  		Serial.print("spindleMotor = ");
 		 	Serial.print(spindleMotor.currentPosition());
  		Serial.print("   sliderestMotor = ");
 		 	Serial.print(sliderestMotor.currentPosition() + 1);
      Serial.print("   ratio = ");
      Serial.print(ratio);
      Serial.print("   rem = ");
      Serial.print(rem);
      Serial.print("   nextPos1 = ");
      Serial.print(nextPos1);
      Serial.print("   cumError = ");
      Serial.println(cumError);

  	}
   if (sliderestMotor.currentPosition() < nextPos2)
  	sliderestMotor.runSpeed();
  }
  else if (once == 0)
    {
      Serial.print("spindleMotor = ");
      Serial.print(spindleMotor.currentPosition());
      Serial.print("   sliderestMotor = ");
      Serial.println(sliderestMotor.currentPosition());
      once = 1;
    }
    
}  // end of loop()
