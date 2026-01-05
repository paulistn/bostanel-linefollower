#include <Servo.h>

#define enA //L298N H bridge pins
#define in1 //Right motors backward pin (GND)
#define in2 //Right motors forward pin (VCC)
#define in3 //Left motors forward pin (VCC)
#define in4 //Left motors backward pin (GND)
#define enB

#define servoPin 8 //Servo pin
Servo servo; //

int distanceLeft, distanceFront, distanceRight; //distances

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // declare input and output pins:
  /* motor control pins(enA, in1, in2, in3, in4, enB) -> OUTPUT - digital for direction and speed control (using PWM), as they need HIGH/LOW signals 
     infrared sensors pins(LS, RS - maybe MS also) -> INPUT - digital?
     ultrasonic sensor pins(echo -> INPUT, trigger -> OUTPUT) - digital?
     servomotor pin (servo) -> OUTPUT - digital?
     buzzer - digital?
     leds -> OUTPUT - digital?
  */
  pinMode();
  pinMode();
  pinMode();
  pinMode(servoPin, OUTPUT);
  servo.attach(servoPin); //Attach to servo object
  for (int angle = 90; angle <= 180; angle += 1)  {
    servo.write(angle);
    delay(15);  }
  for (int angle = 180; angle >= 0; angle -= 1)  {
    servo.write(angle);
    delay(15);  }
  for (int angle = 0; angle <= 90; angle += 1)  {
    servo.write(angle); 
    delay(15); }
  servo.write(90);

  distanceFront=ultrasonicRead();
  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Show the distance
  distanceFront=ultrasonicRead();
  Serial.print("Distance Front: ");
  Serial.print(distanceFront);

  if(digitalRead(RS)==0 && digitalRead(LS)==0){
    //line is detected by both sensors
    if(distanceFront > minimalDistance){
      //if no obstacle is seen under the set minimal distance, continue going forward
      forward()
    }
    else{
      //otherwise, check the surroundings 
      checkSide();
    }
  }
  if(digitalRead(RS)==1 && digitalRead(LS)==0){
    turnLeft();
  }
  if(digitalRead(RS)==0 && digitalRead(LS)==1){
    turnRight();
  }
  delay(10);
}


//Find the distance from obstacle
long ultrasonicRead(){
  digitalWrite(trigger, LOW);
  delayMicroseconds(2); //Just make sure the pin is on LOW first
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10); //High for 10 microseconds -> sends out an 8 cycle sonic burst from the transmitter, which then bounces of an object and hits the receiver(connected to echo)
  digitalWrite(trigger, LOW);
  long duration=pulseIn(echo, HIGH); //When sound waves hit the reciever, they turn the echo HIGH for however long the waves were travelling for
  //plusIn function waits for the pin to hit the desired state, starts timing, and stops when it switches to the other state
  long distance=(duration*.0343)/2;
  return distance;
}

//Decides which way to continue based on the larger distance between both sides 
void compareDistance(){
  (if distanceLeft > distanceRight){
    turnLeft();
    delay(500);
    forword(600);
    delay(600);
    turnRight();
    delay(500);
    forward();
    delay(600);
    turnRight();
    delay(400);
  }
  else{
    turnRight();
    delay(500);
    forword(600);
    delay(600);
    turnLeft();
    delay(500);
    forward();
    delay(600);
    turnLeft();
    delay(400);
  }
}

//Move the servomotor with the ultrasound sensor and xompare the distances on left and right
void checkSide(){
  stop();
  delay(100);
  for (int angle = 90; angle <= 180; angle += 1)  {
    servo.write(angle);
  }
  delay(300);
  distanceRight = ultrasonicRead();
  Serial.print("Distance Right: ");
  Serial.print(distanceRight);
  delay(100);
  for (int angle = 180; angle >= 0; angle -= 1)  {
    servo.write(angle);
    delay(15);  
  }
  delay(500);
  distanceLeft = ultrasonicRead();
  Serial.print("Distance Left: ");
  Serial.print(distanceLeft);
  delay(100);
  for (int angle = 0; angle <= 90; angle += 1)  {
    servo.write(angle); 
    delay(15); 
  }
  servo.write(90);
  delay(300);
  compareDistance();
}


//Motor movement
void forward(){
  //goes forward - both sides go forward
  digitalWrite(in1, LOW); //Right backward LOW
  digitalWrite(in2, HIGH); //Right forward HIGH 
  digitalWrite(in3, HIGH); //Left forward HIGH
  digitalWrite(in4, LOW); //Left backward LOW
}

void backward(){
  //goes backward - both sides go backwards
  digitalWrite(in1, HIGH); //Right backward HIGH
  digitalWrite(in2, LOW); //Right forward LOW 
  digitalWrite(in3, LOW); //Left forward LOW
  digitalWrite(in4, HIGH); //Left backward HIGH
}

void turnLeft(){
  //left side forward, right side backward
  digitalWrite(in1, HIGH); //Right backward HIGH
  digitalWrite(in2, LOW); //Right forward LOW 
  digitalWrite(in3, HIGH); //Left forward HIGH
  digitalWrite(in4, LOW); //Left backward LOW
}

void turnRight(){
  //left side backward, right side forward
  digitalWrite(in1, LOW); //Right backward LOW
  digitalWrite(in2, HIGH); //Right forward HIGH 
  digitalWrite(in3, LOW); //Left forward LOW
  digitalWrite(in4, HIGH); //Left backward HIGH
}

void stop(){
  //everything on low
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW); 
}







