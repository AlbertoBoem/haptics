///////////////////////////////

////////////////////////////

//Motor 
int pinAIN1 = 12; // Direction 
int pinPWMA = 10; // PWM

//Standby
int pinSTBY = 4; 

//Sensors - Input
const int pinPOSITION = A0;
const int pinFORCE = A4;
const int pinPOT = A5;

int positionValue = 0;
int forceValue = 0;
int potValue = 0;

//int locationB = 0;
//int diff = 0;

int stiff = 1;  //1 very fast - 35 very slow come back
double forze = 0;

int ctrlValue = 90; //<---- this is for motor speed

//int reference = 0; //position goal on the slider. Where I want the slider to end up.
//int error = 0; //difference between reference and location. This will be computed later.
//int location = 0; //current location fo slider. This will be read later. 

int lastError = 0;

int basedSpeed = 150;

int maxSpeed = 200;

int dir = 0;

double Kp = 0.5;
double Kd = 1.5;

int posCopy = 0;

void setup() {
  
Serial.begin(9600); //initialize serial communication at 9600 bits/sec

//--------------- PWM Arduino uno
//for Pin 9-10
#ifdef TCCR1B
TCCR1B = TCCR1B & B11111000 | B00000001; // set timer 1 divisor to 1 for PWM frequency of 31372.55 Hz
#endif

pinMode(pinPOSITION, INPUT);
pinMode(pinFORCE, INPUT);
pinMode(pinPOT, INPUT);

pinMode(pinAIN1, OUTPUT); //initialize the #1h -bridge pin as an output
pinMode(pinPWMA, OUTPUT); //initialize the #2h -bridge pin as an output
pinMode(pinSTBY, OUTPUT);

} 

void loop() {

//Position
positionValue = analogRead(pinPOSITION);
positionValue = map(positionValue, 0, 1023, 0, 255);

//Force
forceValue = analogRead(pinFORCE);
forceValue = map(forceValue, 0, 1023, 0, 255);
forceValue = forceValue/10;

//int forceVal1 = map(forceValue, 0, 25, 20, 0); //1)
int forceVal1 = map(forceValue, 0, 25, 20, 0); //2)

//forceValue = map(forceValue, 0, 25, 20, 0);
//Serial.println(forceValue);


//Additional Input
potValue = analogRead(pinPOT);
potValue = map(potValue, 0, 1023, 0, 255);

//digitalWrite(pinAIN1, LOW);

int error = positionValue-127;//positionValue-127;

int motorSpeed = 0; //= Kp * error + Kd * (error - lastError);
//lastError = error;

int theMotorSpeed = 0; 

//Serial.println(forceValue);

//if(positionValue >= 120){
/*if(forceValue >= 5){   // >= 10 1)
  theMotorSpeed = 0;
} else {
  motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;
  //theMotorSpeed = 10*(100-motorSpeed); //error
  theMotorSpeed = 1000*(forceVal1-motorSpeed);
}*/

if(forceValue >= 5){   // >= 10 1)
  digitalWrite(pinAIN1, LOW); //LOW
} else {
  digitalWrite(pinAIN1, HIGH); //HIGH
}

int theMotorSpeedFinal;

motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;
  //theMotorSpeed = 10*(100-motorSpeed); //error
  theMotorSpeed = ctrlValue*(forceVal1-motorSpeed);
  theMotorSpeed = map(theMotorSpeed, 0, 1000, 255, 0);  
  theMotorSpeed = (abs(theMotorSpeed / 0.0125));
  if ( theMotorSpeed > 1){
    theMotorSpeedFinal = 0; //0
  } else {
    theMotorSpeedFinal = 1; //1
  }

theMotorSpeedFinal = theMotorSpeedFinal*255;
  
//Serial.println(theMotorSpeed);
/*
if(positionValue < 80) {         // error  < 30
      theMotorSpeed = 10*(10-error); // (10-error) = stiff , (80-error)= compliant, the value THIS*() can be increase, more stiff
      //digitalWrite(pinAIN1, LOW);

 } else {
      theMotorSpeed = 0;//(basedSpeed + motorSpeed)/2;
      //digitalWrite(pinAIN1, HIGH);
 }
*/

//theMotorSpeed = map(theMotorSpeed, 0, 1000, 255, 0);  //<----------
//Serial.println(theMotorSpeed);

posCopy = positionValue;

//Serial.println(theMotorSpeed);

if (theMotorSpeed > maxSpeed){
  theMotorSpeed = maxSpeed;
}
if (theMotorSpeed < 0){
  theMotorSpeed = 0;
}

if (positionValue < 250){
  digitalWrite(pinAIN1, LOW);
}
if (positionValue > 5){
  digitalWrite(pinAIN1, HIGH);
}

if(error == 0){
  theMotorSpeed = 0;
}

//analogWrite(pinPWMA, theMotorSpeed);
analogWrite(pinPWMA, theMotorSpeedFinal);


//Serial.println(theMotorSpeed);


send_data_1(positionValue, forceVal1);



/* 
location = analogRead(analogPin);

pinval = map(pinval,0,1023, 0.0, 0.999);

diff = locationB - location;
diff = map(diff, -6, 6, 0, 10);

//compute error
error = reference-location; //the difference between where you want to be and where you are

forze = -stiff *(diff-0.9);//-stiff *(diff-0.9);  //Hook's Law (?)

//compute output based on error
if(error>0){
motorDrive(motor2, turnCW, forze);
}
else {
motorDrive(motor2, turnCCW, forze);
}
if(abs(error)<10){
digitalWrite(pinBIN1, HIGH);
digitalWrite(pinBIN2, HIGH);
} 

locationB = location;
*/

}

inline void send_data_1(int val1, int val2){

    Serial.print("1 ");
    Serial.println(val1);
    Serial.print("2 ");
    Serial.println(val2);
  
}
