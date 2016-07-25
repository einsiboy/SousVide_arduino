/*
 * Sketch for learning about pullup resistors and reliably detecting button presses.
 */

// only two wires, one to ground the other to D2
// D2 used as input, with pullup resistor enabled
// uses the built in led on pin 13

int btn = 2;
int led = 13;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(btn, INPUT_PULLUP);
  pinMode(13, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int btnStatus = digitalRead(btn);
  Serial.println(btnStatus);
  //digitalWrite(led, !btnStatus);

  if(btnStatus == LOW){
    digitalWrite(led, !digitalRead(led)); 
   }
}
