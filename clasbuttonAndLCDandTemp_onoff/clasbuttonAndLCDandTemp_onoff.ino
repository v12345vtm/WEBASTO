
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
#include <EEPROM.h> //opslaan user settings
//variabelen :

String versie = "DEG V0.3 clasbuttonLCD-TEMP_onoff-MEGA" ; //sketch versie

byte gewensteTemp = 0; //hoe warm wil je het in de auto hebben?
byte alarmTemp = 60 ; // als de webasto 60gr is , stop het geheel
byte pwm_procent = 90 ; //hoeveel dutycyle moet de pomp draaien , instelbaar met knoppjes

int Sleeptimer = 1 ; //teller in minuten waneer de verwarming moet opstarten
String knopfunctie1 = "pwm" ;//lcd menu toetsfuncties
String knopfunctie2 = "temp" ;
String knopfunctie3 = "alarm" ;
String knopfunctie4 = "timer" ;

byte menustructuur = 0 ;//menustructuur lcd en rs232 die de parameters instelt 0=hoofdmenu 1=pwmmenu 2=gewenstetempmenu ...

bool startvrijgave = 0 ; // 0=mag niet opstarten 1=start webasto


//limieten
#define procent_lower_limit 0 // 0 tot 100 procent
#define procent_upper_limit 100

#define temp_lower_limit 0 //0 tot 60 graden voor de gewenste temp instelling in de wagen
#define temp_upper_limit 60

#define alarm_lower_limit 0 //0tot 100 graden voor het systeem ontploft en we tijdig alles kunnen stoppen
#define alarm_upper_limit 100

#define sleeptimer_lower_limit 0 //0 tot 720 minuten timer eer de webasto mag opstarten , 0 is direkt starten
#define sleeptimer_upper_limit 720


String selected = " gekozen" ; //debug in rs232

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

const int SW1_knop = 33 ; // A0  33 op de mega; //knopjes
const int SW2_knop = 39 ; //A1   39op de mega;
const int SW3_knop = 47 ; // A2  47op de mega;
const int SW4_knop = 49 ; // A3   49op de mega;

const int J3_temp = 36; //12tempsensor 36 op de mega
const int J4_temp = 34; //11  34op de mega


// instantiate the library, representing the tempsensor
TSIC Sensor1(J3_temp);    // only Signalpin, VCCpin unused by default
TSIC Sensor2(J3_temp);    // only Signalpin, VCCpin unused by default
uint16_t temperature = 0;
uint16_t temperature2 = 0;
float Temperatur_C = 0; // 1e tempsenor
float Temperatur_C2 = 0; //2e tempsensor

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


Button button1(SW1_knop); //33 op de mega
Button button2(SW2_knop); //39
Button button3(SW3_knop); //47
Button button4(SW4_knop); //49


void setup() {
  Serial.begin(115200);
  Serial.println(versie) ;

  // read a byte from the current address of the EEPROM
  pwm_procent = EEPROM.read(1);
  gewensteTemp = EEPROM.read(2);
  alarmTemp = EEPROM.read(3);
  //   value = EEPROM.read(4);

  //bij een nieuwe arduino zit de eeprom mogelijk op 255 , en dat mag niet of we zitten block met onze userinputcheck
  if (userinput_check (EEPROM.read(8) , procent_lower_limit , procent_upper_limit))
  {
    pwm_procent = EEPROM.read(8);
  }
  else {
    EEPROM.update(8, 59);
  }


  if (userinput_check (EEPROM.read(6) , temp_lower_limit , temp_upper_limit))
  {
    gewensteTemp = EEPROM.read(6);
  }
  else {
    EEPROM.update(6, 58);
  }


  if (userinput_check (EEPROM.read(7) , alarm_lower_limit , alarm_upper_limit))
  {
    alarmTemp = EEPROM.read(7);
  }
  else {
    EEPROM.update(7, 57);
  }






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




}


//functie die pwm aanstuurt
void set_pwm_pomp (byte pwm)
{
  int dutycycle = map(pwm, 0, 100, 0, 255); // 90 met schaal conversie van 0-255 naar 0-100%
  analogWrite(Q3_mosfet, dutycycle) ; //pomp doen draaien op pwm constante
}

//debug  niet meer in gebruik
void controleerInputs()
{
  if ( button1.IsErFlank())
  {
    Serial.println("but1 flank+1");
    delay(500);
  }
  if ( button2.IsErFlank())
  {
    Serial.println("but2 flank+1");
    delay(500);
  }
  if ( button3.IsErFlank())
  {
    Serial.println("but3 flank+1");
   delay(500);
  }
  if ( button4.IsErFlank())
  {
    Serial.println("but4 flank+1");
    delay(500);
  }
}




// here is where you'd put code that needs to be running every second of some time.
void timer_elke_seconde() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you were here
    previousMillis = currentMillis;


    //we checken even de tempsensors , we doen dat elke seconde
    if (Sensor1.getTemperature(&temperature)) {
      //  Serial.print("uint_16: ");     Serial.println(temperature);
      Temperatur_C = Sensor1.calc_Celsius(&temperature);
      //    Serial.print("Temp: ");     Serial.print(Temperatur_C);    Serial.println(" °C");
    }

    if (Sensor2.getTemperature(&temperature2)) {
      //  Serial.print("uint_16: ");     Serial.println(temperature2);
      Temperatur_C2 = Sensor2.calc_Celsius(&temperature2);
      //    Serial.print("Temp2: ");     Serial.print(Temperatur_C2);    Serial.println(" °C");
    }




    // hier staan we elke seconde
    Serial.print("\n\ngewensteTemp \t\t"); Serial.print(gewensteTemp); Serial.println(" °C");
    Serial.print("pwm_procent \t\t"); Serial.print(pwm_procent); Serial.println(" procent");
    Serial.print("alarmTemp \t\t"); Serial.print(alarmTemp); Serial.println(" °C");
    Serial.print("Sleeptimer: \t\t"); Serial.print(Sleeptimer); Serial.println(" minuten");

    Serial.print("\nTemp1: \t\t\t"); Serial.print(Temperatur_C); Serial.println(" °C");
    Serial.print("Temp2: \t\t\t"); Serial.print(Temperatur_C2); Serial.println(" °C");

    Serial.print("vrijgave: \t\t"); Serial.print(menustructuur); Serial.println(" on/off");
    Serial.print("koel_1: \t\t"); Serial.print(menustructuur); Serial.println(" venti");
    Serial.print("koel_2: \t\t"); Serial.print(menustructuur); Serial.println(" venti");


    Serial.print("\nmenu: \t\t\t"); Serial.println(menustructuur);
    Serial.print("knop1: \t\t\t"); Serial.println(knopfunctie1);
    Serial.print("knop2: \t\t\t"); Serial.println(knopfunctie2);
    Serial.print("knop3: \t\t\t"); Serial.println(knopfunctie3);
    Serial.print("knop4: \t\t\t"); Serial.println(knopfunctie4);



  }
}



// menustructuur die 4parameters kan aanpassen binnen bepaalde limieten
void flowshartmenustructuurLCD() {
  switch (menustructuur) {
    case 0:    // startpositie
      knopfunctie1 = "on";
      knopfunctie2 = "off";
      knopfunctie3 = "settings";
      knopfunctie4 = "later opstarten";
      if ( button1.IsErFlank())
      {
        Serial.println(knopfunctie1 + selected );
        menustructuur = 1 ;
      }
      if ( button2.IsErFlank())
      {
        Serial.println(knopfunctie2 + selected );
        menustructuur = 2 ;
      }
      if ( button3.IsErFlank())
      {
        Serial.println(knopfunctie3 + selected );
        menustructuur = 3 ;
      }
      if ( button4.IsErFlank())
      {
        Serial.println(knopfunctie4 + selected );
        menustructuur = 4 ;
      }
      break;


    case 1:    // startpositie
      knopfunctie1 = "on";
      knopfunctie2 = "off";
      knopfunctie3 = "settings";
      knopfunctie4 = "exit";
      if ( button1.IsErFlank())
      {
        Serial.println(knopfunctie1 + selected );
        // menustructuur = 1 ;
      }
      if ( button2.IsErFlank())
      {
        Serial.println(knopfunctie2 + selected);
        menustructuur = 2 ;
      }
      if ( button3.IsErFlank())
      {
        Serial.println(knopfunctie3 + selected);
        menustructuur = 3 ;
      }
      if ( button4.IsErFlank())
      {
        Serial.println(knopfunctie4 + selected);
        menustructuur = 0 ;
      }
      break;



    case 2:    // off positie
      knopfunctie1 = "on";
      knopfunctie2 = "off";
      knopfunctie3 = "settings";
      knopfunctie4 = "exit";
      if ( button1.IsErFlank())
      {
        Serial.println(knopfunctie1 + selected);
        menustructuur = 1 ;
      }
      if ( button2.IsErFlank())
      {
        Serial.println(knopfunctie2 + selected);
        // menustructuur = 2 ;
      }
      if ( button3.IsErFlank())
      {
        Serial.println(knopfunctie3 + selected);
        menustructuur = 3 ;
      }
      if ( button4.IsErFlank())
      {
        Serial.println(knopfunctie4 + selected);
        menustructuur = 0 ;
      }
      break;



    case 3:    // setting positie
      knopfunctie1 = "timer";
      knopfunctie2 = "gewenstetemp";
      knopfunctie3 = "alarm";
      knopfunctie4 = "pwm";
      if ( button1.IsErFlank())
      {
        Serial.println(knopfunctie1 + selected);
        menustructuur = 5 ;
      }
      if ( button2.IsErFlank())
      {
        Serial.println(knopfunctie2 + selected);
        menustructuur = 6 ;
      }
      if ( button3.IsErFlank())
      {
        Serial.println(knopfunctie3 + selected);
        menustructuur = 7 ;
      }
      if ( button4.IsErFlank())
      {
        Serial.println(knopfunctie4 + selected);
        menustructuur = 8 ;
      }
      break;






    case 4:    // vertraagt starten
      knopfunctie1 = "on vertraagd";
      knopfunctie2 = "off";
      knopfunctie3 = "edit timer";
      knopfunctie4 = "exit";
      if ( button1.IsErFlank())
      {
        Serial.println(knopfunctie1 + selected);
        //menustructuur = 1 ;
      }
      if ( button2.IsErFlank())
      {
        Serial.println(knopfunctie2 + selected);
        //  menustructuur = 2 ;
      }
      if ( button3.IsErFlank())
      {
        Serial.println(knopfunctie3  + selected);
        menustructuur = 5 ;
      }
      if ( button4.IsErFlank())
      {
        Serial.println(knopfunctie4 + selected);
        menustructuur = 0 ;
      }
      break;



    case 5:    // timer edit
      knopfunctie1 = "+ " + String(stap);
      knopfunctie2 = "- " +  String(stap);
      knopfunctie3 = "per " +  String(nextstap);
      knopfunctie4 = "save timer";
      if ( button1.IsErFlank() && userinput_check (  pwm_procent , procent_lower_limit , procent_upper_limit - stap))
      {
        pwm_procent = pwm_procent + stap ;
      }
      if ( button2.IsErFlank() && userinput_check (  pwm_procent , procent_lower_limit + stap , procent_upper_limit ))
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
        Serial.println("save pwm"); //pwm_procent
        if ( userinput_check (  pwm_procent , procent_lower_limit , procent_upper_limit))
        {
          EEPROM.update(menustructuur, pwm_procent);
          menustructuur = 0 ; //0is home menu indien we geldige waarde kregen van gebruiker
        }
      }
      break;





    case 6:    // gewenste temp edit
      knopfunctie1 = "+ " + String(stap);
      knopfunctie2 = "- " +  String(stap);
      knopfunctie3 = "per " +  String(nextstap);
      knopfunctie4 = "save gewenste temp";
      if ( button1.IsErFlank() && userinput_check (  pwm_procent , procent_lower_limit , procent_upper_limit - stap))
      {
        pwm_procent = pwm_procent + stap ;
      }
      if ( button2.IsErFlank() && userinput_check (  pwm_procent , procent_lower_limit + stap , procent_upper_limit ))
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
        Serial.println("save pwm"); //pwm_procent
        if ( userinput_check (  pwm_procent , procent_lower_limit , procent_upper_limit))
        {
          EEPROM.update(menustructuur, pwm_procent);
          menustructuur = 0 ; //0is home menu indien we geldige waarde kregen van gebruiker
        }
      }
      break;





    case 7:    // alarm max
      knopfunctie1 = "+ " + String(stap);
      knopfunctie2 = "- " +  String(stap);
      knopfunctie3 = "per " +  String(nextstap);
      knopfunctie4 = "save alarm";
      if ( button1.IsErFlank() && userinput_check (  pwm_procent , procent_lower_limit , procent_upper_limit - stap))
      {
        pwm_procent = pwm_procent + stap ;
      }
      if ( button2.IsErFlank() && userinput_check (  pwm_procent , procent_lower_limit + stap , procent_upper_limit ))
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
        Serial.println("save pwm"); //pwm_procent
        if ( userinput_check (  pwm_procent , procent_lower_limit , procent_upper_limit))
        {
          EEPROM.update(menustructuur, pwm_procent);
          menustructuur = 0 ; //0is home menu indien we geldige waarde kregen van gebruiker
        }
      }
      break;












    case 8:    // pwm edit
      knopfunctie1 = "+ " + String(stap);
      knopfunctie2 = "- " +  String(stap);
      knopfunctie3 = "per " +  String(nextstap);
      knopfunctie4 = "save pwm";
      if ( button1.IsErFlank() && userinput_check (  pwm_procent , procent_lower_limit , procent_upper_limit - stap))
      {
        pwm_procent = pwm_procent + stap ;
      }
      if ( button2.IsErFlank() && userinput_check (  pwm_procent , procent_lower_limit + stap , procent_upper_limit ))
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
        Serial.println("save pwm"); //pwm_procent
        if ( userinput_check (  pwm_procent , procent_lower_limit , procent_upper_limit))
        {
          EEPROM.update(menustructuur, pwm_procent);
          menustructuur = 0 ; //0is home menu indien we geldige waarde kregen van gebruiker
        }
      }
      break;



  }
}



//controleer of de gebruiker parameters ingeeft die binnen de limiet zijn
bool userinput_check (byte waarde , byte onderlimiet , byte bovenlimiet)
{
  if ((waarde >= onderlimiet) && (waarde <= bovenlimiet))
  {
    Serial.println("geldige input");
    return true ;
  }
  else
  { Serial.println("ongeldige waarde");
    return false ;
  }
}
