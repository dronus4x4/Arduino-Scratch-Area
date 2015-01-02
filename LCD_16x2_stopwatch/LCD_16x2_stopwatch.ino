/*

Standalone Arduino StopWatch
(Only needs the SainSmart LCD Keypad Shield)

By fgarci03 - 20/09/2013

USAGE: connect the Shield to your Arduino
       and upload the sketch;
       You can power it with batteries
       and make it a standalone StopWatch.

*/

// call the necessary libraries
#include <SPI.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(8, 13, 9, 4, 5, 6, 7); // these are the pins used on the shield for this sketch

unsigned long start, finished, elapsed; // variables used on more than 1 function need to be declared here


void setup()
{
  lcd.begin(16, 2); // inicialize the lcd (16 chars, 2 lines)
 
  // a little introduction :) 
  //  lcd.setCursor(0,0); // set the cursor to first character on line 1 - NOT needed (it sets automatically on lcd.begin()
  lcd.print("  The Arduino   ");
  lcd.setCursor(0,1); // set the cursor to first character on line 2
  lcd.print("   StopWatch    ");
  delay(5000); // wait 5 seconds
  
  lcd.clear(); // clear the display
  // lcd.setCursor(0,0); // set the cursor to first character on line 1 - again, not needed, lcd.clear(); sets it
  lcd.print("LFT - Start/Rst");
  lcd.setCursor(0,1);
  lcd.print("SEL - Elap. time");
}

void displayResult()
{
  // declare variables
  float h, m, s, ms;
  unsigned long over;

  // MATH time!!!
  elapsed = finished - start;
  
  h = int(elapsed / 3600000);
  over = elapsed % 3600000;
  m    = int(over / 60000);
  over = over % 60000;
  s    = int(over / 1000);
  ms   = over % 1000;


  lcd.setCursor(0,0);
  lcd.print("Elapsed time: ");

  // display the results
  lcd.setCursor(0,1);
  lcd.print(h, 0); // display variable 'h' - the 0 after it is the number of algorithms after a comma (ex: lcd.print(h, 2); would print 0,00
  lcd.print("h "); // and the letter 'h' after it
  lcd.print(m, 0);
  lcd.print("m ");
  lcd.print(s, 0);
  lcd.print("s ");
  lcd.print(ms, 0);
  lcd.print("ms");
}

void loop()
{
  int x; // declare variables
  x = analogRead (0); // assign 'x' to the Arduino's AnalogueInputs (Shield's buttons)
  if (x < 600 && x > 400) // if the button is LEFT
  {
    start = millis(); // saves start time to calculate the elapsed time
    delay(200); // for debounce
    lcd.clear();
    // lcd.setCursor(0,1);  // print on line 1 - NOT needed
    lcd.print("Started...");
  }
  else if (x < 800 && x > 600) // if the button is SELECT
  {
    finished = millis(); // saves stop time to calculate the elapsed time
    delay(200); // for debounce
    lcd.clear();
    //  lcd.setCursor(0,0); // NOT needed
    displayResult(); // display the results on the function
  }
}
