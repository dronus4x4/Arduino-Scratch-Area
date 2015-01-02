//ZD30 Nissan Patrol Auto Lockup Controller
//Version: V0.0

//Development of controller to operate torque converter lockup solenoid in automatic RE4R03A transmission
//Concept is to use the TCU signal to perform the initial lock then hold the clutch locked until it needs to be realeased.
//Releasing the torque converter clutch will occur under the following conditions:
//  - throttle input greater than 80%
//  - enginer RPM less than 1600RPM (Wholesale Automatics indicated that lower RPM will not provide sufficient oil pressure in the gearbox to hold clutch packs from slipping)
//  - manual unlock switch activated
//  * if TCU signal would have the clutch locked under any of these conditions, the TCU signal will override the release, maintaining lockup.
//To provide maximum flexibility, a manual lock and manual unlock switch will be incorporated.

// Serial command from below library.
// Demo Code for SerialCommand Library
// Steven Cogswell
// May 2011

//NOTE: need new line character '\n' to activate command parser.

/* PWM Frequencies
Pins 5 and 6: controlled by Timer 0

Setting 	Divisor 	Frequency
0x01 	 	1 	 	62500
0x02  		8 	 	7812.5
0x03  		64 	 	976.5625
0x04 	 	256 	 	244.140625
0x05 	 	1024 	 	61.03515625

TCCR0B = TCCR0B & 0b11111000 | <setting>;

Pins 9 and 10: controlled by timer 1

Setting 	Divisor 	Frequency
0x01 	 	1 	 	31250
0x02 	 	8 	 	3906.25
0x03  		64 	 	488.28125
0x04  		256 	 	122.0703125
0x05 	 	1024 	 	30.517578125

TCCR1B = TCCR1B & 0b11111000 | <setting>;

Pins 11 and 3: controlled by timer 2

Setting 	Divisor 	Frequency
0x01 	 	1  		31250
0x02 	 	8 	 	3906.25
0x03  		32  		976.5625
0x04 	 	64 	 	488.28125
0x05 	 	128  		244.140625
0x06  		256  		122.0703125
0x07 	 	1024  		30.517578125

TCCR2B = TCCR2B & 0b11111000 | <setting>;

All frequencies are in Hz and assume a 16000000 Hz system clock.
*/

#include <SerialCommand.h>
#include <EEPROM.h> //Needed to access the eeprom read write functions


//define the pins on the microcontroller
#define arduinoLED 13     //Arduino LED on board
#define TCULock 4     //lock signal from TCU
#define lockSol 3      //PWM channel A to drive solenoid coil
#define lockSolDir 11  //PWM channel A direction (make sure solenoid is driven with +12V)
#define tachPin 2         //Tacho pulse for reading engine RPM
#define throttle A0     //Throttle position sensor reading into Analog Input 0
#define unlockSW 8     //Unlock switch input
#define lockSW 7       //Lock switch input
#define parkNeutralSW 5  //Park/Neutral switch input
#define greenLED 10      //Green LED output
#define redLED 9       //Red LED output
#define pwm_b 11
#define dir_b 13

//define the interrupt for reading the tacho pulse
#define tachInterruptNumber 0  //use interrupt 0 on digital pin 2 for reading the tacho signal

//define the EEPROM locations for parameters
#define firstRun 0    //store 0 here once the parameters have been setup for the first time. If any other number, put the default parameters into the other locations
#define rpm1600 2    //during setup, user can trigger recording of engine 1600 RPM tach frequency rather than using the default coded value

//define lock/unlock reference values
#define throttleUnlockLevel 755 //if throttle is pushed above 80%, unlock. 0-5V=0-1024. TPS 0.5-4.5 therefore 80% is 3.7V = 755
#define refTachMicros 9600   //define standard reference frequency of tach signal when tested engine is at 1600RPM (test vehicle had pulse length 9600us for ~1600RPM)


//global variables
SerialCommand sCmd;     // The SerialCommand object
boolean lockSwitch = 0, unlockSwitch = 0, inGear = 0, debug = 0, tachReadingDisable = 1, TCULockRequest = 0, lockFlag = 0;
int loopCounter = 0, TPSReading = 0, pwmBVal = 128, lockSolPWMVal = 0;
long tachReading = 0;

//volatile globals for interfacing with interrupt
volatile int tachReadingIn = 0;            //this is where the actual reading will be stored in the interrupt ready for reading in the loop
volatile unsigned long tachStartTime = 0;  //this is where the system time is stored at the start of the pulse
volatile boolean newTachReading = LOW;     //this is used to indicate to loop that a new reading is complete

//the setup/initilisation routine
void setup() {
  //setup the onboard LED pin and turn the LED off... may use this to show diagnostics at some stage
  //pinMode(arduinoLED, OUTPUT);      // Configure the onboard LED for output
  //digitalWrite(arduinoLED, LOW);    // default to LED off

  //setup the input pins
  pinMode(TCULock, INPUT);
  pinMode(tachPin, INPUT);
  pinMode(parkNeutralSW, INPUT);
  pinMode(unlockSW, INPUT);
  pinMode(lockSW, INPUT);
  digitalWrite(unlockSW, HIGH);  //set microcontroller internal pullup resistor
  digitalWrite(lockSW, HIGH);     //set microcontroller internal pullup resistor
  //analog input is defined as input by virute of the hardware and arduino libraries
  
  //setup the output pins
  pinMode(lockSol, OUTPUT);
  pinMode(lockSolDir, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_b, OUTPUT);

  //set the outputs off
  digitalWrite(lockSolDir, LOW);  //set the directon of the PWM output bridge so the solenoid is driven with +12V instead of connecting to 0V
  analogWrite(lockSol, 0);       //set the PWM output for the lockup solenoid off
  digitalWrite(greenLED, LOW);   //turn the green LED off
  digitalWrite(redLED, LOW);     //turn the red LED off
  digitalWrite(dir_b, LOW);      //set direction of PWM b output


  //setup the interrupt for reading the tacho pulse
  attachInterrupt(tachInterruptNumber, readTachInterrupt, CHANGE);
  
  //start the serial comms at 19200 baud (original 9600)
  Serial.begin(4800);


  // Setup callbacks for SerialCommand commands
  // have ? say hello to the serial interface so you can check that the system is responding
  sCmd.addCommand("?", sayHello);        // Echos the string argument back
  sCmd.addCommand("hello", sayHello);        // Echos the string argument back
  sCmd.addCommand("debug", toggleDebug);  //toggle debug on/off via serial
  sCmd.addCommand("d", toggleDebug);  //toggle debug on/off via serial
  sCmd.addCommand("tach", toggleTachReading);  //toggle tacho signal reading on/off via serial
  sCmd.addCommand("t", toggleTachReading);  //toggle tacho signal reading on/off via serial
  sCmd.setDefaultHandler(unrecognized);      // Handler for command that isn't matched  (says "What?")
  Serial.println("Ready");
}





//the main program loop (runs continuously)
void loop() {  
  String serialString;
  static int pwmLoopCounter;
  
  //use a loop counter to trigger status updates to be sent to the seril port
  loopCounter++;
  if(loopCounter > 15000 && debug == HIGH)
  {
    loopCounter = 0;
    
    //run status updates here
//    serialString = String(); //initilise serialString as a string before starting to concatenate into it
//    serialString = serialString + "T: " + tachReading;// + ", Above 1600RPM: " + engineAbove1600RPM();
    //serialString = serialString + " Throttle Position: " + TPSReading + " throttle<80%: " + throttleBelow80Pct();
    //serialString = serialString + " TCU Lock: " + TCULockRequest + " Lock Flag: " + lockFlag;
    //serialString = serialString + " lock switch: " + lockSwitch + ", unlock switch: " + unlockSwitch;
    //serialString = serialString + " PWM B: " + pwmBVal;
//    Serial.println(serialString);
    
    Serial.print("T: ");
    Serial.println(tachReading);
    
    Serial.print("TPS: ");
    Serial.println(TPSReading);
    
        
/*    //for testing, allow lock/unlock switch to change pwm_b output value
    if(lockSwitch == HIGH)
    {
      pwmBVal += 10;
      if(pwmBVal > 255){pwmBVal = 255;}
    }
    if(unlockSwitch == HIGH)
    {
      pwmBVal -= 10;
      if(pwmBVal < 0){pwmBVal = 0;}
    }
  */  
    //for testing, output dummy tacho signal on PWMb
    //analogWrite(pwm_b, pwmBVal);
  }


  //process commands from serial port
  sCmd.readSerial();

  //update switch inputs
  //check for lock request from TCU included in readSwitches
  readSwitches(100);
  
  //debugging - test if lock/unlock switches are working
  //digitalWrite(redLED, lockSwitch);
  //digitalWrite(greenLED, unlockSwitch);
  //digitalWrite(greenLED, inGear);
  //digitalWrite(redLED, TCULockRequest);
  
  //update tacho reading
  if(newTachReading == HIGH)
  {
    //copy new reading
    //tachReading = tachReadingIn;
    
    //if new reading is <100 or >50000, ignore it
    if(tachReadingIn < 100 || tachReadingIn > 50000)
    {
      //bad reading
      tachReadingIn = 0;
    }
    else
    {
      //good reading so process it
      
      //rolling average of tach reading (5 samples)
      tachReading = ((tachReading * 4) + tachReadingIn) / 5;
    }
    
    //clear new reading flag
    newTachReading = LOW;
    
    //debugging - show updates on red LED
    //digitalWrite(redLED, !digitalRead(redLED));
  }
  
  //debugging - drive green LED when engine below 1600RPM
  //digitalWrite(greenLED, engineAbove1600RPM());
  
 
  //update throttle position reading
  TPSReading = analogRead(throttle);
  
  //debugging - show TPS > 80% on red LED
  //digitalWrite(redLED, throttleBelow80Pct());
   
  //display status of TCU lock request on green LED
  digitalWrite(greenLED, TCULockRequest);
  
 
  //process "normal" lock conditions first, then unlock conditions. This makes sure the unlock conditions override lock
  //process the TCU lock request last. This allows TCU lock request to override unlock signals. Final unlock condition is park/neutral switch
  //lock if TCU requests lock, or lock switch pressed
  if(TCULockRequest == HIGH || lockSwitch == HIGH)
  {
    //TCU is signalling that it would have the lock solenoid on now
    lockFlag = HIGH;
  }
 
  //unlock if engine below 1600 RPM, or unlock switch pressed, or throttle greater than 80%
  if(engineAbove1600RPM() == LOW || unlockSwitch == HIGH) // || throttleBelow80Pct() == LOW)
  {
    //an unlock condition is true
    lockFlag = LOW;
  }
  
  //lock if TCU requests lock
  if(TCULockRequest == HIGH)
  {
    //TCU is signalling that it would have the lock solenoid on now
    lockFlag = HIGH;
  }

  //final unlock condition of park/neutral switch being in gear
  if(inGear == LOW)
  {
    //not in gear so don't allow lockup
    lockFlag = LOW;
  }
  
  //display lock status on red LED
  digitalWrite(redLED, lockFlag);
  
  //write lockFlag to lock solenoid
  if(lockFlag == HIGH)
  {
    //turn on lock solenoid to lock torque converter clutch
    //to soften locking action of clutch, use pwm to bring on softly. I think the TCU uses 400Hz PWM to do the same thing.
	//use a loop counter to slow down the clutch engagement
	pwmLoopCounter += 1;
	if(pwmLoopCounter > 500)
	{
		//reset loop counter and process lock sequence
		pwmLoopCounter = 0;
		
		//to soften locking action of clutch, use pwm to bring on softly. I think the TCU uses 400Hz PWM to do the same thing.
		if(lockSolPWMVal < 120)
		{
		   //starting lock operation from fully unlocked
		   //start at 50% duty cycle (255 is 100% so 127 ~50%)
                   //modified to start at 30% duty (76) - didn't feel like it started locking at this setting... took ~6sec to lock
                   //return to just under 50%
		   lockSolPWMVal = 120;
		}
		else
		{
		  //continuing lock operation
		  //increase to max 255 in steps of 1 per loop cycle
		  lockSolPWMVal += 1; 
		  if(lockSolPWMVal > 255)
		  {
			//max value is 255 for 100% duty
			lockSolPWMVal = 255;
		  }
		}
	}
  }
  else
  {
    //turn off lock solenoid
   lockSolPWMVal = 0;  //value 0 is 0% duty cycle
   
   //reset pwmLoopCounter
   pwmLoopCounter = 500;
  }
  
  //write value to PWM to lock/unlock clutch (solenoid driver)
  analogWrite(lockSol, lockSolPWMVal);
}







//interrupt for reading tacho signal (interrupt 0)
void readTachInterrupt()
{
  //interrupt is called on both edges of the puse. Need to check what level the signal is so we know whether we are at the start or end of the pulse
  if(digitalRead(tachPin) == LOW)
  {
    //this is the start of the pulse
    //use micros to get the starting time of the pulse
    tachStartTime = micros();
  }
  else
  {
    //this is the end of the pulse
    //create the new tacho reading. This will overwrite the old reading even if it has not yet been processed in loop
    tachReadingIn = (int) (micros() - tachStartTime);
    
    //reset start time for good practice
    tachStartTime = 0;
    
    //set flag so loop knows there is a new reading
    newTachReading = HIGH;
  }
}


 
void toggleDebug()
{
  if (debug == LOW)
  {
    debug = HIGH;
    Serial.println("Debug On");
  }
  else
  {
    debug = LOW;
    Serial.println("Debug Off");
  }
}


void toggleTachReading()
{
  if(tachReadingDisable == LOW)
  {
    tachReadingDisable = HIGH;
    Serial.println("Tach reading disabled");
  }
  else
  {
    tachReadingDisable = LOW;
    Serial.println("Tach reading enabled");
  }
}


void LED_on() {
  Serial.println("LED on");
  digitalWrite(arduinoLED, HIGH);
}

void LED_off() {
  Serial.println("LED off");
  digitalWrite(arduinoLED, LOW);
}

void sayHello() {
  char *arg;
  arg = sCmd.next();    // Get the next argument from the SerialCommand object buffer
  if (arg != NULL) {    // As long as it existed, take it
    Serial.print("Hello ");
    Serial.println(arg);
  }
  else {
    Serial.println("Hello, whoever you are");
  }
}


void processCommand() {
  int aNumber;
  char *arg;

  Serial.println("We're in processCommand");
  arg = sCmd.next();
  if (arg != NULL) {
    aNumber = atoi(arg);    // Converts a char string to an integer
    Serial.print("First argument was: ");
    Serial.println(aNumber);
  }
  else {
    Serial.println("No arguments");
  }

  arg = sCmd.next();
  if (arg != NULL) {
    aNumber = atol(arg);
    Serial.print("Second argument was: ");
    Serial.println(aNumber);
  }
  else {
    Serial.println("No second argument");
  }
}

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {
  Serial.println("What?");
}


//This function will write a 2 byte integer to the eeprom at the specified address and address + 1
void EEPROMWriteInt(int p_address, int p_value)
{
	byte lowByte = ((p_value >> 0) & 0xFF);
	byte highByte = ((p_value >> 8) & 0xFF);

	EEPROM.write(p_address, lowByte);
	EEPROM.write(p_address + 1, highByte);
}

//This function will read a 2 byte integer from the eeprom at the specified address and address + 1
unsigned int EEPROMReadInt(int p_address)
{
	byte lowByte = EEPROM.read(p_address);
	byte highByte = EEPROM.read(p_address + 1);

  	return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}


//Switch reading with debounce
void readSwitches(int debounceCycles)
{
  //use a counter to debounce the switchs rather than timers
  static int lockSwitchCounter, unlockSwitchCounter, inGearCounter, TCUlockReqCounter;
  
   //check for lock request from TCU
  if(digitalRead(TCULock) == HIGH)
  {
    TCUlockReqCounter += 1;
    if(TCUlockReqCounter >= debounceCycles)
    {
        TCUlockReqCounter = debounceCycles;
        TCULockRequest = HIGH;
    }
  }
  else
  {
    TCUlockReqCounter -= 1;
    if(TCUlockReqCounter <= 0)
    {
      TCUlockReqCounter = 0;
      TCULockRequest = LOW;
    }
  }


  //lock switch
  if(digitalRead(lockSW) == LOW)
  {
    lockSwitchCounter += 1;
    if(lockSwitchCounter >= debounceCycles)
    {
        lockSwitchCounter = debounceCycles;
        lockSwitch = HIGH;
    }
  }
  else
  {
    lockSwitchCounter -= 1;
    if(lockSwitchCounter <= 0)
    {
      lockSwitchCounter = 0;
      lockSwitch = LOW;
    }
  }
  
  
  //unlock switch
  if(digitalRead(unlockSW) == LOW)
  {
    unlockSwitchCounter += 1;
    if(unlockSwitchCounter >= debounceCycles)
    {
        unlockSwitchCounter = debounceCycles;
        unlockSwitch = HIGH;
    }
  }
  else
  {
    unlockSwitchCounter -= 1;
    if(unlockSwitchCounter <= 0)
    {
      unlockSwitchCounter = 0;
      unlockSwitch = LOW;
    }
  }
  
  
  //park/neutral switch
  if(digitalRead(parkNeutralSW) == LOW)
  {
    inGearCounter += 1;
    if(inGearCounter >= debounceCycles)
    {
        inGearCounter = debounceCycles;
        inGear = HIGH;
    }
  }
  else
  {
    inGearCounter -= 1;
    if(inGearCounter <= 0)
    {
      inGearCounter = 0;
      inGear = LOW;
    }
  }

}


//read tach signal
long readTach(int pin, boolean level) {
  //original code sourced from "http://tushev.org/articles/electronics/43-measuring-frequency-with-arduino" was for reading frequency assuming 50% duty cycle.
  //Tach signal is not likely to be 50% duty cycle but this should still work ok. May need to use pulseIn for LOW signal timing instead of HIGH signal timing.
  #define SAMPLES 2 //original code from "http://tushev.org/articles/electronics/43-measuring-frequency-with-arduino" had 4096
  long freq = 0;
  for(unsigned int j=0; j<SAMPLES; j++) freq+= 500000/pulseIn(pin, level, 250000);
  return freq / SAMPLES;
}


//check engine RPM is above 1600
boolean engineAbove1600RPM()
{
  //long currentReading = readTach(tach, LOW);
  long currentReading = tachReading;
  
  if(currentReading < refTachMicros)
  {
    return HIGH;
  }
  else
  {
    return LOW;
  }
}


boolean throttleBelow80Pct()
{
   if(TPSReading >= throttleUnlockLevel)
   {
     //throttle input greater than 80%
     return LOW;
   }
   else
   {
     //throttle input below 80%
     return HIGH;
   }
}
