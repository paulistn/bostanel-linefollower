#include <Servo.h>

//L298N H bridge pins
#define enA 3 //For speed control, must be connected to PMW capable pins(with tilda)
#define in1 2 //Right motors backward pin (GND)
#define in2 4 //Right motors forward pin (VCC)
#define in3 5 //Left motors forward pin (VCC)
#define in4 7 //Left motors backward pin (GND)
#define enB 6 //For speed control, must be connected to PMW capable pins(with tilda)

//Left and Right IR sensors pins
#define LS A1 
#define RS A0

//Ultrasound pins
#define echo 10
#define trigger 9

//Servomotor pin
#define servoPin 8
Servo servo;

int distanceLeft, distanceFront, distanceRight; //distances
int minimalDistance = 15;
int baseSpeed=200; //Normal crusing speed
int turnSpeed=80; //Reduce speed for inner wheels druing turning

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
  //Initialization of motor pins
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);
  //Initialization of sensor pins
  pinMode(LS, INPUT);
  pinMode(RS, INPUT);
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
  //Set speed
  analogWrite(enA, baseSpeed);
  analogWrite(enB, baseSpeed);
  //Initalization of servomotor
  pinMode(servoPin, OUTPUT);
  servo.attach(servoPin); //Attach to servo object

  //Starting 
  Serial.println("Place the robot on the black line.");
  //The robot won't move until BOTH sensors see the line.
  while(digitalRead(LS)==1 || digitalRead(RS)==1) {
    //Both LEDs on the sensors should be ON when you are ready
    delay(100); 
  }
  Serial.println("Line detected! Starting in 2 seconds.");
  delay(2000);

  //Test servo movement
  servo.write(90);
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
  Serial.println(distanceFront);

  if((digitalRead(RS)==0) && (digitalRead(LS)==0)){
    if(distanceFront>minimalDistance){
      //on line
      forward(baseSpeed);
    }
    else{
      //stop and look around
      checkSide();
    }
  }
  else if (digitalRead(RS)==1 && digitalRead(LS)==0){
    //off track on right
    pivotLeft();
  }
  else if (digitalRead(RS)==0 && digitalRead(LS)==1){
    //off track on left
    pivotRight();
  }
}


//Find the distance from obstacle
long ultrasonicRead(){
  digitalWrite(trigger, LOW);
  delayMicroseconds(2); //Just make sure the pin is on LOW first
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10); //High for 10 microseconds -> sends out an 8 cycle sonic burst from the transmitter, which then bounces of an object and hits the receiver(connected to echo)
  digitalWrite(trigger, LOW);
  long duration=pulseIn(echo, HIGH, 30000); //When sound waves hit the reciever, they turn the echo HIGH for however long the waves were travelling for - if no echo in 30ms return 0
  //plusIn function waits for the pin to hit the desired state, starts timing, and stops when it switches to the other state
  if(duration==0){
    return 400;
  }
  long distance=(duration*0.0343)/2;
  return distance;
}

//Decides which way to continue based on the larger distance between both sides 
void compareDistance(){
  if (distanceLeft > distanceRight){
    pivotLeft();
      delay(500);
    forward(baseSpeed);
      delay(600);
    pivotRight();
      delay(500);
    forward(baseSpeed);
      delay(600);
    pivotRight();
      delay(400);
  }
  else{
    pivotRight();
    delay(500);
    forward(baseSpeed);
    delay(600);
    pivotLeft();
    delay(500);
    forward(baseSpeed);
    delay(600);
    pivotLeft();
    delay(400);
  }
}

//Move the servomotor with the ultrasound sensor and xompare the distances on left and right
void checkSide(){
  stop();
  delay(100);
  for (int angle = 90; angle <= 180; angle += 1)  {
    servo.write(angle);
    delay(15);
  }
  delay(300);
  distanceRight = ultrasonicRead();
  Serial.print("Distance Right: ");
  Serial.println(distanceRight);
  delay(100);
  for (int angle = 180; angle >= 0; angle -= 1)  {
    servo.write(angle);
    delay(15);  
  }
  delay(500);
  distanceLeft = ultrasonicRead();
  Serial.print("Distance Left: ");
  Serial.println(distanceLeft);
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
void forward(int speed){
  //goes forward - both sides go forward
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  digitalWrite(in1, LOW); //Right backward LOW
  digitalWrite(in2, HIGH); //Right forward HIGH 
  digitalWrite(in3, HIGH); //Left forward HIGH
  digitalWrite(in4, LOW); //Left backward LOW
}

void backward(){
  //goes backward - both sides go backwards
  analogWrite(enA, baseSpeed);
  analogWrite(enB, baseSpeed);
  digitalWrite(in1, HIGH); //Right backward HIGH
  digitalWrite(in2, LOW); //Right forward LOW 
  digitalWrite(in3, LOW); //Left forward LOW
  digitalWrite(in4, HIGH); //Left backward HIGH
}

void turnLeft(){
  //right side faster
  analogWrite(enA, baseSpeed); //Right side fast
  analogWrite(enB, turnSpeed);  //Left side slow
  digitalWrite(in1, LOW); //Right backward LOE
  digitalWrite(in2, HIGH); //Right forward HIGH 
  digitalWrite(in3, HIGH); //Left forward HIGH
  digitalWrite(in4, LOW); //Left backward LOW
}

void turnRight(){
  //left side faster
  analogWrite(enA, turnSpeed); //Right side slow
  analogWrite(enB, baseSpeed);  //Left side fast
  digitalWrite(in1, LOW); //Right backward LOW
  digitalWrite(in2, HIGH); //Right forward HIGH 
  digitalWrite(in3, HIGH); //Left forward HIGH
  digitalWrite(in4, LOW); //Left backward LOW
}

void pivotLeft(){
  //For obstacle avoidance - left forward, right backward
  analogWrite(enA, baseSpeed);
  analogWrite(enB, baseSpeed);
  digitalWrite(in1, HIGH); //Right backward HIGH
  digitalWrite(in2, LOW); //Right forward LOW 
  digitalWrite(in3, HIGH); //Left forward HIGH
  digitalWrite(in4, LOW); //Left backward LOW
}

void pivotRight(){
  analogWrite(enA, baseSpeed);
  analogWrite(enB, baseSpeed);
  digitalWrite(in1, LOW); //Right backward LOW
  digitalWrite(in2, HIGH); //Right forward HIGH 
  digitalWrite(in3, LOW); //Left forward LOW
  digitalWrite(in4, HIGH); //Left backward HIGH
}

void stop(){
  //everything on low
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW); 
}
