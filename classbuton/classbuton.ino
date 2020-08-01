

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
          //  Serial.print("pin  ");
          //  Serial.print(pin );
          // Serial.println("in");
          flank = 1;
          // Serial.print("number of button pushes: ");
          // Serial.println(buttonPushCounter);
        } else {
          // if the current state is LOW then the button went from on to off:
          //   Serial.print("pin  ");
          // Serial.print(pin );
          // Serial.println("uit");
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
        // Serial.println("up+1");
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


Button button1(A0);
Button button2(A1);
Button button3(A2);
Button button4(A3);


void setup() {
  Serial.begin(115200);

}
void loop() {

  controleerInputs();



  //  button4.getState();

}






void controleerInputs()
{
  if ( button1.IsErFlank())
  {
    Serial.println("but1 flank+1");
  }
  if ( button2.IsErFlank())
  {
    Serial.println("but2 flank+1");
  }
  if ( button3.IsErFlank())
  {
    Serial.println("but3 flank+1");
  }
  if ( button4.IsErFlank())
  {
    Serial.println("but4 flank+1");
  }
}
