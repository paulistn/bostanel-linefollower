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

//Led diodes
#define blueLED 11
#define greenLED 12
#define redLED 13

//Buzzer module
#define buzzer A3

int distanceLeft, distanceFront, distanceRight; //distances
int minimalDistance=17;
int baseSpeed=200; //Normal crusing speed
int turnSpeed=80; //Reduce speed for inner wheels druing turning
int boostSpeed=220;
bool obstacleAlertPlayed = false; //checker for buzzer sound

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
  //Initialize led diodes
  pinMode(blueLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  //Initialize buzzer
  pinMode(buzzer, OUTPUT);

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
  for (int angle = 90; angle <= 150; angle += 2)  {
    servo.write(angle);
    delay(20);  }
  for (int angle = 150; angle >= 30; angle -= 2)  {
    servo.write(angle);
    delay(20);  }
  for (int angle = 30; angle <= 90; angle += 2)  {
    servo.write(angle); 
    delay(20); }
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
      obstacleAlertPlayed=false; //reset the value
      setLEDs(HIGH, LOW, LOW);
      forward(baseSpeed);
    }
    else{
      //stop and look around
      setLEDs(LOW, LOW, LOW);
      if (!obstacleAlertPlayed){
        obstacleJingle();
        obstacleAlertPlayed=true;
      }
      checkSide();
    }
  }
  else if (digitalRead(RS)==1 && digitalRead(LS)==0){
    //off track on right
    setLEDs(HIGH, LOW, LOW);
    pivotRight();
  }
  else if (digitalRead(RS)==0 && digitalRead(LS)==1){
    //off track on left
    setLEDs(HIGH, LOW, LOW);
    pivotLeft();
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
    setLEDs(LOW, HIGH, LOW);
    //Turn left off the line
    pivotLeft();
    delay(360); //Turn
    stop();
    delay(100);
    
    //Drive forward past obstacle
    forward(baseSpeed);
    delay(600); //Drive distance to clear obstacle
    stop();
    delay(100);
    
    //Turn rightm parallel to the original path
    pivotRight();
    delay(360); //Turn
    stop();
    delay(100);
    
    //Drive forward alongside obstacle
    forward(baseSpeed);
    delay(500); //Drive distance to pass obstacle
    stop();
    delay(100);
    
    //Turn right towards the line
    pivotRight();
    delay(360); //Turn
    stop();
    delay(100);
    
    //Drive forward until the sensors finds the line
    forward(baseSpeed);
    while(digitalRead(LS)==1 && digitalRead(RS)==1) {
      delay(10); //Keep going until one sensor hits the line
    }
    stop();
    delay(100);
    
    //Align with the line
    if(digitalRead(LS)==0 && digitalRead(RS)==1) {
      //If the left sensor is on the line, pivot left to center
      pivotLeft();
      delay(150);
    } else if(digitalRead(RS)==0 && digitalRead(LS)==1) {
      //If the right sensor on the line, pivot right to center
      pivotRight();
      delay(150);
    }
    //If both sensors on line, we're already centered
    //Delays might need calibration for turn angles and distance
    setLEDs(LOW, LOW, LOW);
  }
  else {
    //Turn right
    setLEDs(LOW, LOW, HIGH);
    pivotRight();
    delay(360);
    stop();
    delay(100);
    
    forward(baseSpeed);
    delay(600);
    stop();
    delay(100);
    
    pivotLeft();
    delay(360);
    stop();
    delay(100);
    
    forward(baseSpeed);
    delay(500);
    stop();
    delay(100);
    
    pivotLeft();
    delay(360);
    stop();
    delay(100);
    
    forward(baseSpeed);
    while(digitalRead(LS)==1 && digitalRead(RS)==1) {
      delay(10);
    }
    stop();
    delay(100);
    
    if(digitalRead(RS)==0 && digitalRead(LS)==1) {
      pivotRight();
      delay(150);
    } else if(digitalRead(LS)==0 && digitalRead(RS)==1) {
      pivotLeft();
      delay(150);
    }
    setLEDs(LOW, LOW, LOW);
  }
}

//Move the servomotor with the ultrasound sensor and compare the distances on left and right
void checkSide(){
  stop();
  delay(200);
  servo.write(150);
  delay(600);
  distanceLeft = ultrasonicRead();
  delay(100);
  distanceLeft = ultrasonicRead();
  Serial.print("Distance Left: ");
  Serial.println(distanceLeft);
  delay(100);

  servo.write(30);
  delay(600);
  distanceRight = ultrasonicRead();
  delay(100);
  distanceRight = ultrasonicRead();
  Serial.print("Distance Right: ");
  Serial.println(distanceRight);
  delay(100);

  servo.write(90);
  delay(400);
  compareDistance();
}

//Buzzer sound
void obstacleJingle(){
  tone(buzzer, 1000, 200); //1000 Hz for 200 ms
  delay(220);  
}


//LED set-up
void setLEDs(bool green, bool blue, bool red){
  digitalWrite(greenLED, green);
  digitalWrite(blueLED, blue);
  digitalWrite(redLED, red);
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
  //For obstacle avoidance - left backward, right forward
  analogWrite(enA, boostSpeed);
  analogWrite(enB, boostSpeed);
  digitalWrite(in1, LOW); //Right backward LOW
  digitalWrite(in2, HIGH); //Right forward HIGH 
  digitalWrite(in3, LOW); //Left forward LOW
  digitalWrite(in4, HIGH); //Left backward HIGH
}

void pivotRight(){
  analogWrite(enA, boostSpeed);
  analogWrite(enB, boostSpeed);
  digitalWrite(in1, HIGH); //Right backward HIGH
  digitalWrite(in2, LOW); //Right forward LOW 
  digitalWrite(in3, HIGH); //Left forward HIGH
  digitalWrite(in4, LOW); //Left backward LOW
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
