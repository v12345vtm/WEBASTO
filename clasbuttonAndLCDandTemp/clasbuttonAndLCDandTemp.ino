
/*
  geert eindwerk : deze sketch : https://github.com/v12345vtm/WEBASTO/tree/master/programma


  menustructuur http://avtanski.net/projects/lcd/




  info :
  Mega, Mega2560, MegaADK pinnen met interupt 2, 3, 18, 19, 20, 21


  //ingangen schakelaars  ,alles met pulldownweerstand
  //sw1_knop pin33
  //sw2_knop pin39
  //sw3_knop pin47
  //sw4_knop pin49

  //ingangen sensors : datasheet : https://github.com/v12345vtm/WEBASTO/blob/master/tsic%20206.pdf
  //J3_temp pin36
  //J4_temp pin34

  //uitgangen
  //Q3 mosfet  pin 45 signaal : https://github.com/v12345vtm/WEBASTO/blob/master/pomp1.bmp  datasheet : https://github.com/v12345vtm/WEBASTO/blob/master/mosfet.pdf
  //K1 relais pin23
  //K2 relais pin25

  //LCD datasheet : https://github.com/v12345vtm/WEBASTO/blob/master/ATM1602A.pdf
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

String versie = "DEG V0.3 clasbuttonLCD-TEMP" ; //sketch versie
byte gewensteTemp = 0; //hoe warm wil je het in de auto hebben?
byte alarmTemp = 60 ; // als de webasto 60gr is , stop het geheel
byte pwm_procent = 90 ; //hoeveel dutycyle moet de pomp draaien , instelbaar met knoppjes
byte menustructuur = 0 ;//menustructuur lcd en rs232 die de parameters instelt
int timer = 0 ; //teller in minuten waneer de verwarming moet opstarten
String knopfunctie1 = "pwm" ;//lcd menu toetsfuncties
String knopfunctie2 = "temp" ;
String knopfunctie3 = "alarm" ;
String knopfunctie4 = "timer" ;



byte stap = 1 ;//per1
byte nextstap = 5 ;//per5
// instantiate variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 1000;           // interval at which to check things (milliseconds)


// defines pins numbers
const int K1_relais = 23; // relais ventilator
const int K2_relais = 25; //
const int Q3_mosfet = 6; //pomp pwm 45 op de mega

const int SW1_knop = A0 ; //33; //knopjes
const int SW2_knop = A1 ; //39;
const int SW3_knop = A2 ; //47;
const int SW4_knop = A3 ; //49;

const int J3_temp = 12; //tempsensor 36 op de mega
const int J4_temp = 34;


// instantiate the library, representing the tempsensor
TSIC Sensor1(J3_temp);    // only Signalpin, VCCpin unused by default
uint16_t temperature = 0;
float Temperatur_C = 0;

// initialize the library with the numbers of the interface pins LCD
LiquidCrystal lcd(32, 20, 22, 24, 26, 28); // RS , E , D4 , D5 , D6 , D7


class Button {
  private:
    byte pin;
    byte flag;
    byte lastflag;
    byte flank;  //status : 2= hoog nivo  1=rijzende flank  0=dalende flank

  public:
    Button(byte pin) {
      this->pin = pin;
      lastflag = LOW;
      init();
    }
    void init() {
      pinMode(pin, INPUT);
      update();
    }


    void update() {
      // read the pushbutton input pin:
      flag = digitalRead(pin);
      flank = 2 ;
      // compare the SW1flag to its previous state
      if (flag != lastflag) {
        // if the state has changed, increment the counter
        if (flag == HIGH) {
          // if the current state is HIGH then the button went from off to on:
          flank = 1;
        } else {
          // if the current state is LOW then the button went from on to off:
          flank = 0;
        }
        // Delay a little bit to avoid bouncing
        delay(50);
      }
      // save the current state as the last state, for next time through the loop
      lastflag = flag;
    }



    byte getState() {
      update();
      return flag;
    }
    bool isPressed() {
      return (getState() == HIGH);
    }


    bool IsErFlank() {
      update();
      if (flank == 1) //0 is dalende flank  en 1 is stijgende flank
      {
        flank = 2 ;//flank is weg zet terug op status2
        return true ;
      }
      else
      {
        return false;
      }

    }




}; // don't forget the semicolon at the end of the class


// Create your objects in the global scope so you can


Button button1(A0); //33 op de mega
Button button2(A1); //39
Button button3(A2); //47
Button button4(A3); //49


void setup() {
  Serial.begin(115200);
  Serial.println(versie) ;



  //save power https://www.youtube.com/watch?v=urLSDi7SD8M&t=1276s
  for (int p = 0 ; p < 6 ; p++) //low power besparing //check hoeveel poorten de mega heeft....
  {
    pinMode(p, OUTPUT); //alles output , en ook de overtollige pinnen omwille van low power besparing
  }
}
void loop() {

  // controleerInputs();
  flowshartmenustructuurLCD ();












  timer_elke_seconde();
  set_pwm_pomp ( pwm_procent) ; //set_pwm_pomp (byte pwm) 0-100% als parameter meegeven



  if (Sensor1.getTemperature(&temperature)) {
    //  Serial.print("uint_16: ");
    // Serial.println(temperature);
    Temperatur_C = Sensor1.calc_Celsius(&temperature);
    //    Serial.print("Temp: ");
    //    Serial.print(Temperatur_C);
    //    Serial.println(" °C");
  }

}



void set_pwm_pomp (byte pwm)
{
  int dutycycle = map(pwm, 0, 100, 0, 255); // 90 met schaal conversie van 0-255 naar 0-100%
  analogWrite(Q3_mosfet, dutycycle) ; //pomp doen draaien op pwm constante
}


void controleerInputs()
{
  if ( button1.IsErFlank())
  {
    Serial.println("but1 flank+1");
    menustructuur = 1 ;
  }
  if ( button2.IsErFlank())
  {
    Serial.println("but2 flank+1");
    menustructuur = 2 ;
  }
  if ( button3.IsErFlank())
  {
    Serial.println("but3 flank+1");
    menustructuur = 3 ;
  }
  if ( button4.IsErFlank())
  {
    Serial.println("but4 flank+1");
    menustructuur = 4 ;
  }






}





void timer_elke_seconde() {
  // here is where you'd put code that needs to be running every second time.
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // hier staan we elke seconde
    Serial.print("\n\ngewensteTemp \t\t"); Serial.println(gewensteTemp);
    Serial.print("pwm_procent \t\t"); Serial.println(pwm_procent);
    Serial.print("alarmTemp \t\t"); Serial.println(alarmTemp);
    Serial.print("timer: \t\t\t"); Serial.println(timer);
    Serial.print("Temp1: \t\t\t"); Serial.print(Temperatur_C); Serial.println(" °C");
    Serial.print("Temp2: \t\t\t"); Serial.print(Temperatur_C); Serial.println(" °C");

    Serial.print("menu: \t\t\t"); Serial.println(menustructuur);
    Serial.print("knop1: \t\t\t"); Serial.println(knopfunctie1);
    Serial.print("knop2: \t\t\t"); Serial.println(knopfunctie2);
    Serial.print("knop3: \t\t\t"); Serial.println(knopfunctie3);
    Serial.print("knop4: \t\t\t"); Serial.println(knopfunctie4);


  }
}








void flowshartmenustructuurLCD() {
  // do something different depending on the range value:
  switch (menustructuur) {
    case 0:    // startpositie
      knopfunctie1 = "pwm";
      knopfunctie2 = "temp";
      knopfunctie3 = "alarm";
      knopfunctie4 = "timer";
      if ( button1.IsErFlank())
      {
        Serial.println("pwm gekozen");
        menustructuur = 1 ;
      }
      if ( button2.IsErFlank())
      {
        Serial.println("temp gekozen");
        menustructuur = 2 ;
      }
      if ( button3.IsErFlank())
      {
        Serial.println("alarm gekozen");
        menustructuur = 3 ;
      }
      if ( button4.IsErFlank())
      {
        Serial.println("timer gekozen");
        menustructuur = 4 ;
      }
      break;

    case 1:    // pwm editen
      knopfunctie1 = "+ " + String(stap);
      knopfunctie2 = "- " +  String(stap);
      knopfunctie3 = "per " +  String(nextstap);
      knopfunctie4 = "save pwm";
      if ( button1.IsErFlank())
      {
        pwm_procent = pwm_procent + stap ;
      }
      if ( button2.IsErFlank())
      {
        pwm_procent = pwm_procent - stap ;
      }
      if ( button3.IsErFlank())
      {
        if ( stap == 1)
        {
          stap = 5;
          nextstap = 1;
          return;
        }
        if ( stap == 5)
        {
          stap = 1;
          nextstap = 5;
          return;
        }
      }
      if ( button4.IsErFlank())
      {
        Serial.println("save pwm");
        menustructuur = 0 ; //0is home menu
      }
      break;


    case 2:    // gewensteTemp editen
      knopfunctie1 = "+ " + String(stap);
      knopfunctie2 = "- " +  String(stap);
      knopfunctie3 = "per " +  String(nextstap);
      knopfunctie4 = "save temp";
      if ( button1.IsErFlank())
      {
        gewensteTemp = gewensteTemp + stap ;
      }
      if ( button2.IsErFlank())
      {
        gewensteTemp = gewensteTemp - stap ;
      }
      if ( button3.IsErFlank())
      {
        if ( stap == 1)
        {
          stap = 5;
          nextstap = 1;
          return;
        }
        if ( stap == 5)
        {
          stap = 1;
          nextstap = 5;
          return;
        }
      }
      if ( button4.IsErFlank())
      {
        Serial.println("save gewensteTemp");
        menustructuur = 0 ; //0is home menu
      }
      break;

    case 3:    //  alarmTemp editen
      knopfunctie1 = "+ " + String(stap);
      knopfunctie2 = "- " +  String(stap);
      knopfunctie3 = "per " +  String(nextstap);
      knopfunctie4 = "save alarmTemp";
      if ( button1.IsErFlank())
      {
        alarmTemp = alarmTemp + stap ;
      }
      if ( button2.IsErFlank())
      {
        alarmTemp = alarmTemp - stap ;
      }
      if ( button3.IsErFlank())
      {
        if ( stap == 1)
        {
          stap = 5;
          nextstap = 1;
          return;
        }
        if ( stap == 5)
        {
          stap = 1;
          nextstap = 5;
          return;
        }
      }
      if ( button4.IsErFlank())
      {
        Serial.println("save alarmTemp");
        menustructuur = 0 ; //0is home menu
      }
      break;


    case 4:    //  timer editen
      knopfunctie1 = "+ " + String(stap);
      knopfunctie2 = "- " +  String(stap);
      knopfunctie3 = "per " +  String(nextstap);
      knopfunctie4 = "save timer doen we niet";
      if ( button1.IsErFlank())
      {
        timer = timer + stap ;
      }
      if ( button2.IsErFlank())
      {
        timer = timer - stap ;
      }
      if ( button3.IsErFlank())
      {
        if ( stap == 1)
        {
          stap = 5;
          nextstap = 1;
          return;
        }
        if ( stap == 5)
        {
          stap = 1;
          nextstap = 5;
          return;
        }
      }
      if ( button4.IsErFlank())
      {
       // Serial.println("save timer");
        menustructuur = 0 ; //0is home menu
      }
      break;
  }
}
