//geert eindwerk
/*
  //ingangen schakelaars
  //sw1 pin33
  //sw2 pin39
  //sw3 pin47
  //sw4 pin49

  //ingangen sensors
  //J3 pin36
  //J4 pin34

  //uitgangen
  //Q3 mosfet
  //K1 relais pin23
  //K2 relais pin25

  //LCD
   LCD RS pin to digital pin 32
   LCD Enable pin to digital pin20
   LCD D4 pin to digital pin 22
   LCD D5 pin to digital pin 24
   LCD D6 pin to digital pin 26
   LCD D7 pin to digital pin  28
   LCD R/W pin to ground
   LCD VSS pin to ground
   LCD VCC pin to 5V
   10K resistor:
   ends to +5V and ground
   wiper to LCD VO pin (pin 3)

*/

// include the library code:
#include <LiquidCrystal.h> // LCD 4bit mode

#include "TSIC.h"       // include the library tsic voor de tempsensors uit te lezen



//variabelen : 
uint16_t temperature = 0;
float Temperatur_C = 0;
String versie = "DEG V0.2" ; //sketch versie  

// defines pins numbers
const int K1_relais = 23; // relais 
const int K2_relais = 25; //

const int SW1_knop = 33; //knopjes
const int SW2_knop = 39;
const int SW3_knop = 47;
const int SW4_knop = 49;

const int J3_temp = 36; //tempsensor
const int J4_temp = 34;

 
// instantiate the library, representing the sensor
//TSIC Sensor1(4, 2);    // Signalpin, VCCpin, Sensor Type
TSIC Sensor1(J3_temp);    // only Signalpin, VCCpin unused by default
//TSIC Sensor1(4, NO_VCC_PIN, TSIC_50x);    // external powering of 50x-Sensor
//TSIC Sensor1(4, 2, TSIC_30x);    // Signalpin, VCCpin, Sensor Type
//TSIC Sensor2(5, 2, TSIC_50x);  // Signalpin, VCCpin, Sensor Type NOTE: we can use the same VCCpin to power several sensors




// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(32, 20, 22, 24, 26, 28); // RS , E , D4 , D5 , D6 , D7

void setup() {

  //Start everything up

  
  //save power https://www.youtube.com/watch?v=urLSDi7SD8M&t=1276s
  for (int p = 0 ; p < 32 ; p++) //low power besparing //check hoeveel poorten de mega heeft....
  {
    pinMode(p, OUTPUT); //alles output , en ook de overtollige pinnen omwille van low power besparing 
  }

  //behalve deze zijn inputs
  pinMode(SW1_knop, INPUT); // Sets   as an Input
  pinMode(SW2_knop, INPUT); // Sets   as an Input
  pinMode(SW3_knop, INPUT); // Sets   as an Input
  pinMode(SW4_knop, INPUT); // Sets   as an Input

  pinMode(J3_temp, INPUT); // Sets   as an Input
  pinMode(J4_temp, INPUT); // Sets   as an Input
  
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print(versie);
  delay(500);
}

void loop() {
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  lcd.print(millis() / 1000);



 if (Sensor1.getTemperature(&temperature)) {
    Serial.print("uint_16: ");
    Serial.println(temperature);
    Temperatur_C = Sensor1.calc_Celsius(&temperature);
    Serial.print("Temperature: ");
    Serial.print(Temperatur_C);
    Serial.println(" °C");
  }



  
}
