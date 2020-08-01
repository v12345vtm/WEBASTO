

class Button {
  private:
    byte pin;
    byte flag;
    byte lastflag;

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

      // compare the SW1flag to its previous state
      if (flag != lastflag) {
        // if the state has changed, increment the counter
        if (flag == HIGH) {
          // if the current state is HIGH then the button went from off to on:
          Serial.print("pin  ");
          Serial.print(pin );
          Serial.println("uit");
          // Serial.print("number of button pushes: ");
          // Serial.println(buttonPushCounter);
        } else {
          // if the current state is LOW then the button went from on to off:
          Serial.print("pin  ");
          Serial.print(pin );
          Serial.println("in");
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

  button1.getState();
  button2.getState();
  button3.getState();
  button4.getState();

}
