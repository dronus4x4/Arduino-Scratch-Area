/ *

The circuit:

* LCD RS pin to digital pin 8

* LCD Enable pin to digital pin 9

* LCD D4 pin to digital pin 4

* LCD D5 pin to digital pin 5

* LCD D6 pin to digital pin 6

* LCD D7 pin to digital pin 7

* LCD BL pin to digital pin 10

* KEY pin to analogl pin 0

* /

#include <LiquidCrystal.h>

LiquidCrystal LCD (8, 13, 9, 4, 5, 6, 7);

Char msgs[5][16] = {"Right Key OK",

"Up Key OK",

"Down Key OK",

"Left Key OK",

"Select Key OK"};

Int adc_key_val[5] ={50, 200, 400, 600, 800};

Int NUM_KEYS = 5;

Int adc_key_in;

Int key=-1;

Int oldkey=-1;

Void setup ()

{

(lcd.clear);

Lcd.begin (16, 2);

Lcd.setCursor (0,0);

Lcd.print ("ADC key testing");

}

Void loop ()

{

Adc_key_in = analogRead (0); / / read the value from the sensor

Key = get_key (adc_key_in); / / convert into key press

If (key! = oldkey) / / if keypress is detected

{

Delay (50); / / wait for debounce time

Adc_key_in = analogRead (0); / / read the value from the sensor

Key = get_key (adc_key_in); / / convert into key press

If (key! = oldkey)

{

Lcd.setCursor (0, 1);

Oldkey = key;

If (key >=0) {

Lcd.print (msgs[key]);

}

}

}

Delay 100.

}

/ / Convert ADC value to key number

Int get_key (unsigned int input)

{

Int k;

For (k = 0; K < NUM_KEYS; k++)

{

If (input < adc_key_val[k])

{

Return k;

}

}

If (k > = NUM_KEYS) k = -1; / / No valid key pressed

Return k;

}
