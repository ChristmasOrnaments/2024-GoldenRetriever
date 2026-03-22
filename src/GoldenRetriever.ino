/*
 * ATTiny85_Internal_8MHz_Template.cpp
 *
 * Created: 11/5/2024
 * Charlieplexing functions modeled after the code found here:
 *  http://www.instructables.com/id/CharliePlexed-LED-string-for-the-Arduino/step7/Coding-the-Arduino/
 * 2019 - Added prescalar of 2 to accomodate 8MHz clock div/8 (low fuse 0x62)
 */ 

#ifdef F_CPU  
#undef F_CPU
#endif	
#define F_CPU 4000000 // 4 MHz

// ATMEL ATTINY85 / ARDUINO
//	            						  ____
//  		   (D5/A0) RST/PB5	1|*	  |8  VCC
//	 PWM   (D3/A3)	   PB3	2|	  |7  PB2  (D2/A1)
//	 PWM   (D4/A2)	   PB4	3|	  |6  PB1  (D1)		PWM
//	        				   GND	4|____|5  PB0  (D0)		PWM

#include <avr/pgmspace.h>
// Power Mgt:
#include <avr/sleep.h>
#include <avr/wdt.h>

// FUNCTION PROTOTYPES
void displayChar(int from, int through, int tranSpeed); // loads a Pattern from above. From & through specify what patterns to show. tranSpeed is the speed of the frames
//void ledSpecify(int highPin, int lowPin); // This allows you to manually control which pin goes low & which one goes high
void turnon(int led); // This turns on a certian led from the list of leds
void alloff(); // This turns all the LED's off
void setup_watchdog(int ii);
void system_sleep();

// ******************************************************************************************************************
//
// Much of the charlieplex code has been modified from http://www.instructables.com/id/CharliePlexed-LED-string-for-the-Arduino/step7/Coding-the-Arduino/
// The power management code is from 

#define LEDCount 12
byte pins[4];
//const byte LEDCount = 20;

// The attiny85 watchdog only sleeps for 8 seconds at a time, so for a longer period of time we need to setup a counter variable
//	 and check that against the # of times it wakes up.
// The internal clock isn't very accurate, so you need to factor that in. My tests show a deviation of ~0.9198 at 900 seconds,
//	 so a rough equation would be [ SleepTime = DesiredTime * 0.9198 ]
// Every MCU will be different, so some experimentation will be necessary if you want accurate intervals
// Some example times:
//	 5:00 -> 276
//	10:00 -> 552
//	15:00 -> 828
//	30:00 -> 1656
int SleepTime = 416; // Sleep time in seconds

int SleepCnt = SleepTime/8; // counter for watchdog wake timer

// blinkdelay is used by displayChar to set the microseconds the LED will be on, then off. It can affect brightness, but mostly affects speed
// blinkdelay could be replaced with a timeOn and timeOff to facilitate a type of PWM to increase LED brightness at low voltage 
//		(using a small Rs at higher voltage would be bad for LED unless turn-on time can be limited with this)
int blinkdelay = 90;   //Speed Param 1: smaller = faster
// runSpeed is the number of times displayChar will loop through each pattern before going to the next. Again, this affects speed
byte runSpeed = 90;		 //Speed Param 2: smaller = faster (0-255) - this can be set individually when calling displayChar()

// Random number
int RndNumber = 0;

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
volatile boolean f_wdt = 1;

const byte LEDConnections[LEDCount][2] ={	// This stores which outputs the LEDs are connected to: { Anode, Cathode }
	// Bottom Row
	{ 0 , 1 },	  //X1
	{ 1 , 2 },	  //X3
	{ 2 , 3 },	  //X5
	// Row 1
	{ 1 , 0 },	  //X2			  
	{ 2 , 1 },	  //X4
	{ 3 , 2 },	  //X6
	// Row 2
	{ 0 , 2 },	  //X9
	{ 1 , 3 },	  //X11
	// Row 3
	{ 2 , 0 },	  //X10
	{ 3 , 1 },	  //X12
	// Row 4
	{ 0 , 3 },	  //X15
	{ 3 , 0 }	  //X7
};

const byte displays[][LEDCount] PROGMEM ={	// This stores the array in Flash ROM.
	// All Off (0)
	{0,0,0,0,0,0,0,0,0,0,0,0},

	// Pattern 1: All LEDs Flash (1-2)
	{1,1,1,1,1,1,1,1,1,1,1,1},
	{0,0,0,0,0,0,0,0,0,0,0,0},

	// Pattern 2: 3 LED Race Clockwise (3-28)
	{ 0,0,1,0,0,0,0,0,0,0,0,0 }, //  3 x
	{ 0,0,1,0,0,0,0,0,0,1,0,0 }, //  4 xx
	{ 0,0,1,0,0,0,0,0,0,1,1,0 }, //  5 xxx
	{ 0,0,0,0,0,0,0,0,0,1,1,1 }, //  6  xxx
	{ 0,0,0,0,0,0,0,1,0,0,1,1 }, //  7   xxx
	{ 0,0,0,0,0,1,0,1,0,0,0,1 }, //  8    xxx
	{ 0,0,0,0,0,1,0,1,1,0,0,0 }, //  9     xxx
	{ 0,0,0,0,1,1,0,0,1,0,0,0 }, // 10      xxx
	{ 0,1,0,0,1,0,0,0,1,0,0,0 }, // 11       xxx
	{ 1,1,0,0,1,0,0,0,0,0,0,0 }, // 12        xxx
	{ 1,1,0,1,0,0,0,0,0,0,0,0 }, // 13         xxx
	{ 1,0,0,1,0,0,1,0,0,0,0,0 }, // 14          xxx
	{ 0,0,1,1,0,0,1,0,0,0,0,0 }, // 15 x         xx
	{ 0,0,1,0,0,0,1,0,0,1,0,0 }, // 16 xx         x
 
	{ 0,0,1,0,0,0,0,0,0,1,1,0 }, // 17 xxx -
	{ 0,0,0,0,0,0,0,0,0,1,1,1 }, // 18  xxx -
	{ 0,0,0,0,0,0,0,1,0,0,1,1 }, // 19   xxx -
	{ 0,0,0,0,0,1,0,1,0,0,0,1 }, // 20    xxx -
	{ 0,0,0,0,0,1,0,1,1,0,0,0 }, // 21     xxx -
	{ 0,0,0,0,1,1,0,0,1,0,0,0 }, // 22      xxx -
	{ 0,1,0,0,1,0,0,0,1,0,0,0 }, // 23       xxx -
	{ 1,1,0,0,1,0,0,0,0,0,0,0 }, // 24        xxx -
	{ 1,1,0,1,0,0,0,0,0,0,0,0 }, // 25         xxx -
	{ 1,0,0,1,0,0,1,0,0,0,0,0 }, // 26          xxx -
 
	{ 0,0,0,1,0,0,1,0,0,0,0,0 }, // 27           xx
	{ 0,0,0,0,0,0,1,0,0,0,0,0 }, // 28            x

	// Pattern 3: "+" rotation (29-40)
	{ 1,0,1,0,0,0,0,0,1,0,0,1 },
	{ 0,0,0,1,1,0,0,1,0,1,0,0 },
	{ 0,1,0,0,0,1,1,0,0,0,1,0 },
	{ 1,0,1,0,0,0,0,0,1,0,0,1 },
	{ 0,0,0,1,1,0,0,1,0,1,0,0 },
	{ 0,1,0,0,0,1,1,0,0,0,1,0 },
	{ 1,0,1,0,0,0,0,0,1,0,0,1 },
	{ 0,0,0,1,1,0,0,1,0,1,0,0 },
	{ 0,1,0,0,0,1,1,0,0,0,1,0 },
	{ 1,0,1,0,0,0,0,0,1,0,0,1 },
	{ 0,0,0,1,1,0,0,1,0,1,0,0 },
	{ 0,1,0,0,0,1,1,0,0,0,1,0 },

	// Pattern 4: 2 LEDs Racing back and forth (41-46)
	{ 0,0,0,1,0,0,0,1,0,0,0,0 },
	{ 1,0,0,0,0,1,1,0,0,0,0,1 },
	{ 0,1,1,0,0,0,0,0,1,0,1,0 },
	{ 0,0,0,0,1,0,0,0,0,1,0,0 },
	{ 0,1,1,0,0,0,0,0,1,0,1,0 },
	{ 1,0,0,0,0,1,1,0,0,0,0,1 },

	// Pattern 5: 2 LEDs race from top-left corner to bottom-right corner and back (47-65)
	{ 0,0,1,0,0,0,0,0,0,0,0,0 },
	{ 0,0,0,0,0,0,1,0,0,1,0,0 },
	{ 0,0,0,1,0,0,0,0,0,0,1,0 },
	{ 1,0,0,0,0,0,0,0,0,0,0,1 },
	{ 0,1,0,0,0,0,0,1,0,0,0,0 },
	{ 0,0,0,0,1,1,0,0,0,0,0,0 },
	{ 0,0,0,0,0,0,0,0,1,0,0,0 },
	{ 0,0,0,0,1,1,0,0,0,0,0,0 },
	{ 0,1,0,0,0,0,0,1,0,0,0,0 },
	{ 1,0,0,0,0,0,0,0,0,0,0,1 },
	{ 0,0,0,1,0,0,0,0,0,0,1,0 },
	{ 0,0,0,0,0,0,1,0,0,1,0,0 }, // line 58
	{ 0,0,1,0,0,0,0,0,0,0,0,0 },
	{ 0,0,0,0,0,0,1,0,0,1,0,0 },
	{ 0,0,0,1,0,0,0,0,0,0,1,0 },
	{ 1,0,0,0,0,0,0,0,0,0,0,1 },
	{ 0,1,0,0,0,0,0,1,0,0,0,0 },
	{ 0,0,0,0,1,1,0,0,0,0,0,0 },
	{ 0,0,0,0,0,0,0,0,1,0,0,0 },

	// Pattern 6: 3 LED Race Counter-Clockwise (66-91) 
	{ 0,0,0,0,0,0,1,0,0,0,0,0 },
	{ 0,0,0,1,0,0,1,0,0,0,0,0 },
	{ 1,0,0,1,0,0,1,0,0,0,0,0 },
	{ 1,1,0,1,0,0,0,0,0,0,0,0 },
	{ 1,1,0,0,1,0,0,0,0,0,0,0 },
	{ 0,1,0,0,1,0,0,0,1,0,0,0 },
	{ 0,0,0,0,1,1,0,0,1,0,0,0 },
	{ 0,0,0,0,0,1,0,1,1,0,0,0 },
	{ 0,0,0,0,0,1,0,1,0,0,0,1 },
	{ 0,0,0,0,0,0,0,1,0,0,1,1 },
	{ 0,0,0,0,0,0,0,0,0,1,1,1 },
	{ 0,0,1,0,0,0,0,0,0,1,1,0 },
	{ 0,0,1,0,0,0,1,0,0,1,0,0 },
	{ 0,0,1,1,0,0,1,0,0,0,0,0 },
	{ 1,0,0,1,0,0,1,0,0,0,0,0 },
	{ 1,1,0,1,0,0,0,0,0,0,0,0 },
	{ 1,1,0,0,1,0,0,0,0,0,0,0 },
	{ 0,1,0,0,1,0,0,0,1,0,0,0 },
	{ 0,0,0,0,1,1,0,0,1,0,0,0 },
	{ 0,0,0,0,0,1,0,1,1,0,0,0 },
	{ 0,0,0,0,0,1,0,1,0,0,0,1 },
	{ 0,0,0,0,0,0,0,1,0,0,1,1 },
	{ 0,0,0,0,0,0,0,0,0,1,1,1 },
	{ 0,0,1,0,0,0,0,0,0,1,1,0 },
	{ 0,0,1,0,0,0,0,0,0,1,0,0 },
	{ 0,0,1,0,0,0,0,0,0,0,0,0 }, 

	// Pattern 7: Twinkle (92-101)
/*	{ 0,0,0,0,0,0,1,0,0,0,0,0 },
	{ 1,0,0,0,0,0,0,0,0,0,1,0 },
	{ 0,0,0,0,1,1,1,0,0,0,0,0 },
	{ 0,1,0,0,0,0,0,0,0,1,0,0 },
	{ 1,0,1,0,0,1,0,0,0,0,0,0 },
	{ 0,0,0,0,0,0,0,0,0,0,0,1 },
	{ 0,0,0,0,0,0,1,0,1,1,0,0 },
	{ 1,0,1,0,0,0,0,0,0,0,0,0 },
	{ 0,0,1,0,1,0,0,0,0,0,1,0 },
	{ 0,0,0,0,0,0,0,0,1,0,0,0 },	*/

	// Pattern 8: LEDs race from L12 to L4 in opposite directions (102-114)
	{ 0,0,0,0,0,0,0,0,0,1,0,0 },
	{ 0,0,1,0,0,0,0,0,0,0,1,0 },
	{ 0,0,0,0,0,0,1,0,0,0,0,1 },
	{ 0,0,0,1,0,0,0,1,0,0,0,0 },
	{ 1,0,0,0,0,1,0,0,0,0,0,0 },
	{ 0,1,0,0,0,0,0,0,1,0,0,0 },
	{ 0,0,0,0,1,0,0,0,0,0,0,0 },

	{ 0,1,0,0,0,0,0,0,1,0,0,0 },
	{ 1,0,0,0,0,1,0,0,0,0,0,0 },
	{ 0,0,0,1,0,0,0,1,0,0,0,0 },
	{ 0,0,0,0,0,0,1,0,0,0,0,1 },
	{ 0,0,1,0,0,0,0,0,0,0,1,0 },
	{ 0,0,0,0,0,0,0,0,0,1,0,0 },

};

void setup()   {
	// Set prescalar so the clock runs at 4MHz
	// See here: https://forum.arduino.cc/index.php?topic=624890.0
	cli(); //disable interrupts - this is a timed sequence.
	CLKPR=0x80; //change enable
	CLKPR=0x01; //set prescaler to 2
	sei(); //enable interrupts
  
	// initialize the digital pin as an output:
	pins[0] = 0;
	pins[1] = 1;
	pins[2] = 2;
	pins[3] = 3;

	// Status LED
	//pinMode(4, OUTPUT);

	setup_watchdog(9); // Watchdog timer; approximately 8 seconds sleep
	randomSeed(analogRead(A2));	 // Seeds the random number generator for more randomness. A2 must be disconnected!
}

void loop()						
{
	int PastRnd = 0;
	if (f_wdt==1) {	 // wait for timed out watchdog / flag is set when a watchdog timeout occurs

		f_wdt=0;	   // reset flag
		if (SleepCnt >= (SleepTime/8)) {
			SleepCnt = 0; // Reset sleep counter
			// Code to execute after we wake from sleep:		  
			// displayChar( from, through, tranSpeed )
			for (int demos=0; demos<3; demos++) {
				// Generate a random number for pattern selection
				while (RndNumber == PastRnd) { RndNumber = random(1,9); }
				PastRnd = RndNumber;
				switch (RndNumber) {
				case 1: // Pattern 2  
					//displayChar(3,28, 70);
          //displayChar(3,28, 70); 
          displayChar(3,4, 70);
          displayChar(5,17, 70);
          for (int loop=0; loop<2; loop++) {
            displayChar(6,17, 70);
          }
          displayChar(6,14, 70);
          displayChar(27,28, 70);
					break;
				case 2: // Pattern 3  
					for (int loop=0; loop<2; loop++) {
					  displayChar(29,40, 70); 
					}							   
					break;
				case 3: // Pattern 4  
					for (int loop=0; loop<2; loop++) {
						  displayChar(41, 46, 70);
              displayChar(41, 46, 70);
              displayChar(41, 46, 70);
					}
					displayChar(41, 41, 70);
					break;			  
				case 4: // Pattern 1 - Blink them all  
					for (int loop=0; loop<4; loop++) {
						displayChar(0, 0, 210);
						delay(50);
						displayChar(1, 1, 210); 
						delay(150);
					}
					break;	   
				case 5: // Pattern 5  
					//for (int loop=0; loop<2; loop++) {
					//displayChar(47, 65, 70); 
          //displayChar(47, 65, 70);
          for (int loop=0; loop<4; loop++) {
            displayChar(47, 58, 70);
          }
          displayChar(47, 47, 70);
					break;
				case 6: // Pattern 6  
					displayChar( 66, 91, 70);
          displayChar( 66, 91, 70); 
					break;									  
/*				case 7: // Pattern 7  
					for (int loop=0; loop<3; loop++) {
					  displayChar( 92, 101, 100); 
					}							   
					break;		*/
				case 7: // Pattern 8  
					for (int loop=0; loop<4; loop++) {
					  //displayChar( 102, 114, 70); 
            displayChar( 92, 104, 70);
					}							   
					break;											
				}
			}
// End of wake code

			//pinMode(0,INPUT); // Normally you would set the pinMode to INPUT to save power, but since the charlieplexing code does this, we don't need to here
			system_sleep();
			//pinMode(0,OUTPUT); // Set the pinmode back to OUTPUT, as it was before	
		} else {
			SleepCnt++;
			//pinMode(0,INPUT); // Normally you would set the pinMode to INPUT to save power, but since the charlieplexing code does this, we don't need to here
			system_sleep();
			//pinMode(0,OUTPUT); // Set the pinmode back to OUTPUT, as it was before
		}
	}	
}

// Charlieplexing Code
// *********************************************************************************************

// Courtesy of "computergeek"
// http://www.instructables.com/id/CharliePlexed-LED-string-for-the-Arduino/step7/Coding-the-Arduino/
void displayChar(int from, int through, int tranSpeed)// loads a Pattern from above. From & through specify what patterns to show. tranSpeed is the speed of the frames (smaller=faster)
{
  boolean run = true;
  byte k;
  int t = from;
  while(run == true)
  {
	for(int i = 0; i < tranSpeed; i++)
	{
	  for(int j = 0; j < LEDCount; j++)
	  {
		  k = pgm_read_byte(&(displays[t][j]));
		  if (k == 2)
		  {
		    run = false;
		  }
		  else if(k == 1)
		  {
		    turnon(j);
		    delayMicroseconds(blinkdelay);
		    alloff();
		  }
		  else if(k == 0)
		  {
		    delayMicroseconds(blinkdelay);
		  }
	  }
	}
	if(through == t){
	  return;
	}
	t++;
  }
}

void turnon(int led) // This turns on a certian led from the list of leds
{
//	int pospin = LEDConnections[led][0] + 2;  //  <-- what is the +2 for?
//	int negpin = LEDConnections[led][1] + 2;
  int anode = LEDConnections[led][0];
  int cathode = LEDConnections[led][1];
  pinMode (anode, OUTPUT);
  digitalWrite (anode, HIGH);
  pinMode (cathode, OUTPUT);
  digitalWrite (cathode, LOW);
}

void alloff() // This turns all the LED's off
// TM - added sizeof(pins) to determine pin count (instead of hard-coded counts)
{
  for(int i = 0; i < sizeof(pins); i++)
  {
	pinMode (pins[i], INPUT);
  }
}

// Power Management
// *********************************************************************************************

// set system into the sleep state
// system wakes up when watchdog is timed out
void system_sleep() {
	cbi(ADCSRA,ADEN);					 // switch Analog to Digitalconverter OFF

	set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
	sleep_enable();

	sleep_mode();						 // System sleeps here

	sleep_disable();					 // System continues execution here when watchdog timed out
	sbi(ADCSRA,ADEN);					 // switch Analog to Digitalconverter ON
}

// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {

	byte bb;
	int ww;
	if (ii > 9 ) ii=9;
	bb=ii & 7;
	if (ii > 7) bb|= (1<<5);
	bb|= (1<<WDCE);
	ww=bb;

	MCUSR &= ~(1<<WDRF);
	// start timed sequence
	WDTCR |= (1<<WDCE) | (1<<WDE);
	// set new watchdog timeout value
	WDTCR = bb;
	WDTCR |= _BV(WDIE);
}

// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect) {
	f_wdt=1;  // set global flag
}
