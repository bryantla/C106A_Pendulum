// TODO: https://lastminuteengineers.com/rotary-encoder-arduino-tutorial/
// TODO: https://www.youtube.com/watch?v=QE4IQlwOgiA&ab_channel=MakerTutor

volatile unsigned int temp, counter = 0; //This variable will increase or decrease depending on the rotation of encoder
const int zeroPos = 0; // encoder reading when pendulum is vertical
const float res = 1200;  // number of encoder ticks per revolution
    
void setup() {
  Serial.begin(9600);

  pinMode(2, INPUT_PULLUP); // internal pullup input pin 2 
  
  pinMode(3, INPUT_PULLUP); // internal pullup input pin 3
  // Setting up interrupt
  // A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(0, ai0, RISING);
   
  // B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(1, ai1, RISING);
}
   
void loop() {
  // Send the value of counter
  if( counter != temp ){
    Serial.println(encoderToAngle(counter));
    temp = counter;
  }
//  Serial.println(encoderToAngle(counter));
}
   
void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(3) == LOW) {
    counter++;
  } else {
    counter--;
  }
}
   
void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if (digitalRead(2) == LOW) {
    counter--;
  } else {
    counter++;
  }
}

// converts encoder reading to angle (rad)
float encoderToAngle(int enc) {
  return fmod((enc-zeroPos)/res * 2*PI, 2*PI);
}
