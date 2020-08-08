// include the library code:
#include <LiquidCrystal.h> // LCD 4bit mode
#include "TSIC.h"       // include the library tsic voor de tempsensors uit te lezen
#include <EEPROM.h> //opslaan user settings
//variabelen :

String versie = "DEG V0.3 clasbuttonLCD-TEMP_onoff-MEGA-standenschak check_positie_draaiknop" ; //sketch versie

byte gewensteTemp  ; //hoe warm wil je het in de auto hebben?
byte alarmTemp   ; // als de webasto 60gr is , stop het geheel
byte pwm_procent   ; //hoeveel dutycyle moet de pomp draaien , instelbaar met knoppjes
int Sleeptimer = 1 ; //teller in minuten waneer de verwarming moet opstarten

String knopfunctie1 = "*" ;//lcd menu toetsfuncties
String knopfunctie2 = "*" ;
String knopfunctie3 = "*" ;
String knopfunctie4 = "*" ;

//menustructuur zie https://docs.google.com/spreadsheets/d/1Ux7BjQliBvcJo_uFXEetqfq39N1StjSkJp7nEkzE3UM/edit#gid=0
byte menustructuur = 0 ;//menustructuur lcd en rs232 die de parameters instelt 0=hoofdmenu 1=pwmmenu 2=gewenstetempmenu ...

bool  pompvrijgave = 0 ; //0=niet starten 1=mag wel starten
bool timervrijgave = 0 ; //is de sleeptimer afgelopen en mogen we de webasta opstarten met vertraging 0=nee 1=ja
bool startvrijgave = 0 ; // 0=mag niet opstarten 1=start webasto
bool stookvrijgave = 0 ; // 0=we hebben voldoende warm gestookt ,  1=we moeten bijstoken
bool alarmbit = 1 ; //als we niet gemeten hebben gaan we niet vrijgeven
//4standenschakelaar programma (vgl. wasmachine programma) zie https://docs.google.com/spreadsheets/d/1Ux7BjQliBvcJo_uFXEetqfq39N1StjSkJp7nEkzE3UM/edit#gid=0
byte meerstandenschakelaar[] = {B000,  B001, B001, B001 , B001, B001 , B011, B011, B011, B011 , B111 }; //bit-wise toestanden

byte stand = 0 ; //de stand van de meerstandenschakelaar
byte draairichtingMeerstandenschak = 2 ; // 0=zak  1=up 2=hold

//limieten
#define procent_lower_limit 0 // 0 tot 100 procent
#define procent_upper_limit 100

#define temp_lower_limit 0 //0 tot 60 graden voor de gewenste temp instelling in de wagen
#define temp_upper_limit 60

#define alarm_lower_limit 45 //0tot 100 graden voor het systeem ontploft en we tijdig alles kunnen stoppen
#define alarm_upper_limit 150

#define sleeptimer_lower_limit 0 //0 tot 720 minuten timer eer de webasto mag opstarten , 0 is direkt starten
#define sleeptimer_upper_limit 720

#define vertragingPWMpomp 6 //startseq pomp

String selected = " gekozen" ; //debug in rs232

byte stap = 1 ;//per1
byte nextstap = 5 ;//per5
// instantiate variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 1000;           // interval at which to check things (milliseconds)


// defines pins numbers
#define K1_ventilator   23  // relais ventilator
#define K2_gloeikaars   25  // relais gloeikaars
#define Q3_mosfet   45  //pomp pwm sproeier

#define SW1_knop  33   // A0  33 op de mega; //knopjes
#define SW2_knop  39   //A1   39op de mega;
#define SW3_knop  47   // A2  47op de mega;
#define SW4_knop  49   // A3   49op de mega;

#define J3_temp   36  //tempsensor  op de mega temp. in de auto
#define J4_temp   34  //34op de mega oververhitting veiligheid


byte aftelklok  ;

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
      // save the current state as the last state, for next time through the looop
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





void setup() {
  Serial.begin(115200);
  Serial.println(versie) ;

  // read a byte from the current address of the EEPROM
  pwm_procent = EEPROM.read(1);
  gewensteTemp = EEPROM.read(2);
  alarmTemp = EEPROM.read(3);
  //   value = EEPROM.read(4);

  //bij een nieuwe arduino zit de eeprom mogelijk op 255 , en dat mag niet of we zitten block met onze userinputcheck
  //een ongeldige waarde in een maagdeljk nieuwe eeprom , zetten we op 50% van het bereik tijdens 1e boot
  if (userinput_check (EEPROM.read(8) , procent_lower_limit , procent_upper_limit))
  {
    pwm_procent = EEPROM.read(8);
  }
  else {
    EEPROM.update(8, procent_upper_limit / 2);
  }


  if (userinput_check (EEPROM.read(6) , temp_lower_limit , temp_upper_limit))
  {
    gewensteTemp = EEPROM.read(6);
  }
  else {
    EEPROM.update(6, temp_upper_limit / 2);
  }


  if (userinput_check (EEPROM.read(7) , alarm_lower_limit , alarm_upper_limit))
  {
    alarmTemp = EEPROM.read(7);
  }
  else {
    EEPROM.update(7, alarm_upper_limit / 2);
  }



  // meerstandenschakelaar =  vertragingen[stand] ; // initieel stand 0 en zijn overeenkomstige vertraging



  //save power https://www.youtube.com/watch?v=urLSDi7SD8M&t=1276s
  for (int p = 0 ; p < 54 ; p++) //low power besparing //check hoeveel poorten de mega heeft....
  {
    pinMode(p, OUTPUT); //alles output , en ook de overtollige pinnen omwille van low power besparing
  }
  pinMode(J3_temp, INPUT); //temp sensor
  pinMode(J4_temp, INPUT); //

}



Button button1(SW1_knop); //33 op de mega
Button button2(SW2_knop); //39
Button button3(SW3_knop); //47
Button button4(SW4_knop); //49


void loop() {


  //  timervrijgave = 1 ; //debug
  // controleerInputs();//debug
  flowshartmenustructuurLCD ();
  timer_elke_seconde(); //output variabelen via rs232




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
    draairichtingMeerstandenschak = 1;//         0=zak  1=up 2=hold");

  }
  if ( button2.IsErFlank())
  {
    Serial.println("but2 flank+1");
    draairichtingMeerstandenschak = 0;//         0=zak  1=up 2=hold");
  }
  if ( button3.IsErFlank())
  {
    Serial.println("but3 flank+1");
    draairichtingMeerstandenschak = 2;//         0=zak  1=up 2=hold");;
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

    // hier staan we elke seconde vrijmaken rs232 scherm
    Serial.print("\n\n \n\n\n\n");

    //  klok = klok + 1 ;

    //checkPompvertraging() ;//eerst ventilator , dan pwm pomp
    //  checkInschakelvertraging() ; //als de sleeptimer op is , opstarten ( vb na 180minuten )



    check_stookvrijgave();//stoken we niet te veel of te weinig ?
    check_alarmbit();//tjernobyl of veilig?
    check_positie_draaiknop();//toon stand programmakknop visueel
    check_uitbollen_dan_shutdown() ; //we kunnen na de delay vertraging een stand verhogen


    //    #define K1_ventilator   23  // relais ventilator
    //#define K2_gloeikaars   25  // relais gloeikaars
    //#define Q3_mosfet   45  //pomp pwm sproeier
    bitRead(meerstandenschakelaar[stand] , 0) ; //venti
    bitRead(meerstandenschakelaar[stand] , 1) ; //gloeikaars
    bitRead(meerstandenschakelaar[stand] , 2) ; //pompvrijgave

    //byte wachttijd_next_stand = vertragingen [stand] ; //vertragingen , tijd tot volgende stap
    digitalWrite (K1_ventilator , bitRead(meerstandenschakelaar[stand] , 0)) ;
    digitalWrite (K2_gloeikaars , bitRead(meerstandenschakelaar[stand] , 1)) ;
    digitalWrite (Q3_mosfet , bitRead(meerstandenschakelaar[stand] , 2)) ;

    // en dan opt gemak printen
    // Serial.print("\n\ngewensteTemp \t\t"); Serial.print(gewensteTemp); Serial.println(" °C");
    Serial.print("pwm_procent  \t"); Serial.print(pwm_procent); Serial.println(" procent");
    // Serial.print("alarmTemp:\t"); Serial.print(alarmTemp); Serial.println(" °C");
    Serial.print("Sleeptimer:\t"); Serial.print(Sleeptimer); Serial.println(" minuten");

    Serial.print("\nhuidigetemp:\t"); Serial.print(Temperatur_C); Serial.print(" °C -gewenstetemp\t"); Serial.print(gewensteTemp); Serial.println("°C");
    Serial.print("veiligheid:\t"); Serial.print(Temperatur_C2); Serial.print(" °C -alarmTemp\t"); Serial.print(alarmTemp); Serial.println("°C");

    Serial.print("startvrijgave:\t"); Serial.print(startvrijgave); Serial.println(" // on/off");
    Serial.print("timervrijgave:\t"); Serial.print(timervrijgave); Serial.println(" // inschak vertr");
    Serial.print("stookvrijgave:\t"); Serial.print(stookvrijgave); Serial.println(" // 0=tewarm  1=bijstoken");
    Serial.print("alarmbit:\t"); Serial.print(alarmbit); Serial.println(" // 0=veilig  1=tjernobyl");


    //  Serial.print("\nstand:"); Serial.print (stand);// Serial.print(" vertraging: "); Serial.print(vertragingen [stand]);  Serial.print(" richtng: _"); Serial.print (draairichtingMeerstandenschak);  Serial.println("_        0=zak  1=up 2=hold");
    Serial.print("\nventilator:\t"); Serial.print(bitRead(meerstandenschakelaar[stand] , 0) ); Serial.println(" ");
    Serial.print("gloeikaars:\t"); Serial.print(bitRead(meerstandenschakelaar[stand] , 1)); Serial.println("  ");
    Serial.print("pompvrijgave:\t"); Serial.print(bitRead(meerstandenschakelaar[stand] , 2)); Serial.println(" ");

    // Serial.print("\ninterne klok: \t\t"); Serial.print(klok); Serial.println(" seconden");
    Serial.print("aftelklok:\t"); Serial.print(aftelklok); Serial.println(" uitbollen venti");


    Serial.print("\nmenu:\t"); Serial.println(menustructuur);
    Serial.print("knop1:\t"); Serial.println(knopfunctie1);
    Serial.print("knop2:\t"); Serial.println(knopfunctie2);
    Serial.print("knop3:\t"); Serial.println(knopfunctie3);
    Serial.print("knop4:\t"); Serial.println(knopfunctie4);






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
        startWebasto (); ;
      }
      if ( button2.IsErFlank())
      {
        Serial.println(knopfunctie2 + selected );
        stopWebasto ();;
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
        // startWebasto ();
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
        startvrijgave = true ; //nu zijn we afhankelijk van de inschakelvertraging
        //  klok = 0 ; //we maken het ons gemakkelijk
      }
      if ( button2.IsErFlank())
      {
        Serial.println(knopfunctie2 + selected);
        stopWebasto () ;
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

    case 5:    // sleeptimer edit
      knopfunctie1 = "+ " + String(stap);
      knopfunctie2 = "- " +  String(stap);
      knopfunctie3 = "per " +  String(nextstap);
      knopfunctie4 = "save timer doen we niet";
      if ( button1.IsErFlank() && userinput_check (  pwm_procent , sleeptimer_lower_limit , sleeptimer_upper_limit - stap))
      {
        Sleeptimer = Sleeptimer + stap ;
      }
      if ( button2.IsErFlank() && userinput_check (  pwm_procent , sleeptimer_lower_limit + stap , sleeptimer_upper_limit ))
      {
        Sleeptimer = Sleeptimer - stap ;
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
        Serial.println("save sleeptimer"); //pwm_procent
        if ( userinput_check (  Sleeptimer , sleeptimer_lower_limit , sleeptimer_upper_limit))
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
      if ( button1.IsErFlank() && userinput_check (  gewensteTemp , temp_lower_limit , temp_upper_limit - stap))
      {
        gewensteTemp = gewensteTemp + stap ;
      }
      if ( button2.IsErFlank() && userinput_check (  gewensteTemp , temp_lower_limit + stap , temp_upper_limit ))
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
        Serial.println("save gewensteTemp"); //gewensteTemp
        if ( userinput_check (  gewensteTemp , temp_lower_limit , temp_upper_limit))
        {
          EEPROM.update(menustructuur, gewensteTemp);
          menustructuur = 0 ; //0is home menu indien we geldige waarde kregen van gebruiker
        }
      }
      break;

    case 7:    // alarm max
      knopfunctie1 = "+ " + String(stap);
      knopfunctie2 = "- " +  String(stap);
      knopfunctie3 = "per " +  String(nextstap);
      knopfunctie4 = "save alarm";
      if ( button1.IsErFlank() && userinput_check (  alarmTemp , alarm_lower_limit , alarm_upper_limit - stap))
      {
        alarmTemp = alarmTemp + stap ;
      }
      if ( button2.IsErFlank() && userinput_check (  alarmTemp , alarm_lower_limit + stap , alarm_upper_limit ))
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
        Serial.println("save pwm"); //pwm_procent
        if ( userinput_check (  alarmTemp , alarm_lower_limit , alarm_upper_limit))
        {
          EEPROM.update(menustructuur, alarmTemp);
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

//startroutine en bewaking temp en veiligheden
void startWebasto ()
{
  //  bool  pompvrijgave = 0 ; //0=niet starten 1=mag wel starten
  //bool timervrijgave = 0 ; //is de sleeptimer afgelopen en mogen we de webasta opstarten met vertraging 0=nee 1=ja
  //bool startvrijgave = 0 ; // 0=mag niet opstarten 1=start webasto
  startvrijgave = true ;
  timervrijgave = true ; //forceer de timervrijgave want we willen nu direct opstarten

  draairichtingMeerstandenschak = 1 ; // 0=zak  1=up 2=hold

  // digitalWrite (K1_ventilator , HIGH) ;
}



//stoproutine en gecontroleerd afkoelen veiligheden
void stopWebasto ()
{
  //startvrijgave = false ;
  //timervrijgave = false ;

  if (!startvrijgave)
  {
    //als je niet op start ooit gedrukt hebt , moet je niet geforceert koelen
    Serial.println("\n\n\nwe drukten nooit op start");
  }
  else
  {
    stand = 1; //stand 1 is de altijd de ventilator  die aan is
    draairichtingMeerstandenschak = 2 ; // 0=zak  1=up 2=hold
    Serial.println("zet op HOLD we de koeling moet nog 10sec blijven");
    aftelklok = 10 ;
  }
  //set_pwm_pomp ( 0) ; //set_pwm_pomp (byte pwm) 0-100% als parameter meegeven , de pompvrijgave moet ook geset zijn
}





//controleer of we mogen aftellen ( mogen we de aftelklok doen lopen?)
void check_uitbollen_dan_shutdown()
{

  ////
  // stand = 1; //stand 1 is de altijd de ventilator  die aan is
  // draairichtingMeerstandenschak = 2 ; // 0=zak  1=up 2=hold
  if (stand ==  1  && draairichtingMeerstandenschak == 2 )
  {
    aftelklok = aftelklok - 1 ;


    if (aftelklok == 0) {
      startvrijgave = 0;
      timervrijgave = 0; //we mogen niet elke 8uur opstarten
      stand = 0 ;

    }

  }


  ////







}

//Sleeptimer we kunnen de timer op 180minuten zetten , en dan start de webasto autonoom op
void checkInschakelvertraging()
{
  //  bool  pompvrijgave = 0 ; //0=niet starten 1=mag wel starten
  //bool timervrijgave = 0 ; //is de sleeptimer afgelopen en mogen we de webasta opstarten met vertraging 0=nee 1=ja
  //bool startvrijgave = 0 ; // 0=mag niet opstarten 1=start webasto
  int sleeptimer_in_seconden = Sleeptimer * 60 ;
  sleeptimer_in_seconden = 16 ; //debug
  if (startvrijgave == true   )
  {
    timervrijgave = true ;
  }
  else
  {
    timervrijgave = false ;
  }
}

//moeten we stoken of moeten we ff wachten van stoken
void check_stookvrijgave()
{
  //we checken even de tempsensors ( we doen dat elke seconde)
  if (Sensor1.getTemperature(&temperature)) {
    //  Serial.print("uint_16: ");     Serial.println(temperature);
    Temperatur_C = Sensor1.calc_Celsius(&temperature);
    //    Serial.print("Temp: ");     Serial.print(Temperatur_C);    Serial.println(" °C");
  }


//debug 

Temperatur_C = 9;
  
  if (gewensteTemp > Temperatur_C)
  {
    // Serial.println("we mogen blijven stoken");
    stookvrijgave = true;
  }
  else
  {
    //  Serial.println("we moeten ff de brander afleggen");
    stookvrijgave = false;
  }
}





//Smelt de webasto niet en gaat dit niet ontploffen zoals in beirut ?
void check_alarmbit()
{
  //we checken even de tempsensors ( we doen dat elke seconde)
  if (Sensor2.getTemperature(&temperature2)) {
    //  Serial.print("uint_16: ");     Serial.println(temperature2);
    Temperatur_C2 = Sensor2.calc_Celsius(&temperature2);
    //    Serial.print("Temp2: ");     Serial.print(Temperatur_C2);    Serial.println(" °C");
  }

  if (alarmTemp >= Temperatur_C2)
  {
    // Serial.println("Geen tjernobyl");
    alarmbit = false;
  }
  else
  {
    // Serial.println("ALARM , tjernobyl is comming");
    alarmbit = true;
  }
}


void check_positie_draaiknop()
{
  Serial.println("");
  if (draairichtingMeerstandenschak == 0)
  {
    Serial.print ("down  - ");
    if (stand >= 1  ) {
      stand = stand - 1 ;
    }
  }
  if (draairichtingMeerstandenschak == 1   )
  {
    Serial.print ("up  - ");
    if (stand < sizeof(meerstandenschakelaar) - 1  ) {
    /////
if (stookvrijgave ) {   
    stand = stand + 1 ;
}
else {
   Serial.print ("geen stookvrijgave!! mmar we starten de ventilator wel zodat je hoort dat er iets aanligt");
   stand = 1 ; 
  }
    }
  }
  if (draairichtingMeerstandenschak == 2)
  {
    Serial.print ("hold  - ");
  }
  Serial.print("stand:"); Serial.print (stand); Serial.print("/"); Serial.println (sizeof(meerstandenschakelaar) - 1);

}
