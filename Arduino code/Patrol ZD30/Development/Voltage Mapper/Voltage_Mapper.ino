//*********************************************************
//*********************************************************
//**                                                     **
//**                 Voltage Mapper                      **
//**                                                     **
//**                  Alpha Version                      **
//**                   2014-12-28                        **
//**                                                     **
//**                   Developers:                       **
//**                   Jason Hurd                        **
//**                                                     **
//**                                                     **
//*********************************************************
//*********************************************************

// =========   PURPOSE:   ==================================
// Read voltage from air flow sensor, apply a bias (voltage offset) for a particular voltage range (load point), then send it to the ECU
// =========================================================

// =========   PROCESS:   ==================================
// Initalization first reads the parameters in the selected profile (0-XX) file from the SD card. Once all parameters are loaded into an
// array variable (settings[] the mapping of voltage will begin.

// =========   REQUIRED FILES:   ============================
// readProfile.ino
// =========================================================


// -- The magic of F() for strings in Serial.print --
// The F() macro tells the compiler to keep your strings in PROGMEM. This helps free up dynamic memory
// Using #define also helps free up a few bytes of dynamic memory

// ========= THINGS TO DO =======
// 1. Need to add code to make sure we have valid math (no negative voltage, higher than 5 volts) for example you'd only want a negative bias at loadpoint 255 (5v input)
//    Possibly a good idea to limit the bias value to something reasonable. What would be the maximum voltage difference required?
// 2. Wack the whole lot into interrupt.. or at least the voltage readings (Probably should have straight through voltage in/voltage out until all parameters are loaded.
//    This way there is no hesitation (other than arduino startup time) for the ECU to see air flow.
// 3. Read voltage at idle and mark the load point in the profileX.txt (as load points below idle are irrelavent)
// 4. Incorporate an interface

//  ========    Load Associated libraries   ========
#include <SD.h>
#include <SPI.h>

//  ======== Define Constants   =============
#define outputVoltagePWM 6 // Set arduino pin number to variable
#define debug 0 // Enable debugging Serial info
#define profileSelect 0 //Profile number to be used

// =======   Global Variables    ===========
int inputVoltageRAW; //Raw values (between 0-1023)
float inputVoltageREAL; //Real Voltage = (Raw * (MaxVoltage / MaxRawValue)
int loadOnce = 0; //Set to 1 if config has been loaded
signed int settings[255]; // A place to store all parameters from the profileX.txt
signed int biasRAW; // The voltage offset in relation to RAW input voltage (1023)
float loadPoint; // Voltage range. This is calculated from the input voltage, then rounded to nearest, hence a float variable.
// ====================================


void setup() {
  Serial.begin(9600); // Set baud rate for serial
  pinMode(outputVoltagePWM, OUTPUT); // Set the arduino pin to output

  if (debug == 1) {
    Serial.println(F(""));
    Serial.println(F("DEBUG MODE ENABLED"));
    Serial.println(F(""));
  }

  //SETUP SD CARD & TEST
  Serial.print(F("Initializing SD card"));
  if (!SD.begin(10)) { // Check if SD card is not there on pin 10
    Serial.println(F("   ...FAILED!!!"));
    return;
  } else {
    Serial.println(F("   ...OK"));
  }
  //========================
  SPI.setClockDivider(4); // Sets the SPI clock. 4 = one quarter of the system clock or 4Mhz
}



void loop() {
  if (loadOnce == 0) { //Run the function to load profile parameters if it hasn't already
    readProfile();
  } else {
    inputVoltageRAW = analogRead(A0); // Read raw input voltage for analog pin A0
    inputVoltageREAL = inputVoltageRAW * (5.0 / 1023.0); // Convert Raw input values, max 1023, to voltage, max 5 volts
   
    loadPoint = inputVoltageRAW * (255.0/1023.0); // Break the incoming voltage RAW down into 255 loadpoints by multiplying it by the number of load points by the max input voltage RAW, then round to nearest
    biasRAW = settings[(int)round(loadPoint)]; // The value of the calculated load point
    analogWrite(outputVoltagePWM, (inputVoltageRAW  + round(biasRAW))/ 4); // This is for bias in relation to 1023 (inputVoltageRAW  + round(biasRAW))/ 4) or 5mV per bias point, OR This is for bias in relation to 255((inputVoltageRAW / 4) + round(biasRAW)) or 20mV per bias point
    
    
    Serial.print(F("LoadPoint: "));
    Serial.print((int)round(loadPoint));
    Serial.print(F(",   "));
    
    Serial.print(F("Bias Value: "));
    Serial.print(biasRAW);
    Serial.print(F(",   "));
    
    Serial.print(F("InputVoltage: "));
    Serial.print(inputVoltageREAL);
    Serial.print(F(",   "));
    
    Serial.print(F("Voltage Difference: "));
    Serial.print(inputVoltageREAL - ((inputVoltageRAW  + round(biasRAW))/ 4) * (5.0 /255.0) );
    Serial.print(F(",   "));
    
    Serial.print(F("outputVoltage: "));
    Serial.println(((inputVoltageRAW  + round(biasRAW))/ 4) * (5.0 /255.0));

  }
}
