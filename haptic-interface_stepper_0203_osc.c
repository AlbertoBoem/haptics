// Program demonstrating how to control a powerSTEP01-based ST X-NUCLEO-IHM03A1
// stepper motor driver shield on an Arduino Uno-compatible board with touch input and haptic feedback
#include <Arduino.h>
#include <Ethernet.h>
#include <OSCMessage.h>
#include <powerSTEP01ArduinoLibrary.h>
#include <SPI.h>
#include "wiring_private.h" // pinPeripheral() function

//for ToF sensor
#include <Wire.h>
#include <VL53L0X.h>

//PID LIB
#include <PID_v1.h>

#define ledPin 13
byte mac[] = { 0xA8, 0x61, 0x0A, 0xAE, 0x29, 0x8A};//0x90, 0xA2, 0xDA, 0xD6, 0xA3, 23};//0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};//0x90, 0xA2, 0xDA, 0xD6, 0xA3, 23 };
IPAddress myIp(192,168,10,3); //router2
//myIp(192,168,2,2); //direct
//myIp(192,168,10,5); //router1
//(192,168,100,60);//(169,254,163,83);//(10,0,0,131); //131
IPAddress destIp(192,168,10,2); //router
//destIp(169,254,163,83); //direct?
unsigned int outPort = 8009;//20203; //udp receive in max
unsigned int inPort = 8008;//20000; //udpsend in max
EthernetUDP Udp;

SPIClass altSPI (&sercom1, 12, 13, 11, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);
// Pin definitions for the X-NUCLEO-IHM03A1 connected to an Uno-compatible board
#define nCS_PIN A2
#define STCK_PIN 9
#define nSTBY_nRESET_PIN 8
#define nBUSY_PIN 4

// powerSTEP library instance, parameters are distance from the end of a daisy-chain
// of drivers, !CS pin, !STBY/!Reset pin
powerSTEP driver(0, nCS_PIN, nSTBY_nRESET_PIN);

//for ToF sensor
VL53L0X sensor;
//#define LONG_RANGE
//#define HIGH_SPEED
#define HIGH_ACCURACY

//for switch
int readSwitch = 0;

//for FSR
int fsrPin = 0;     // the FSR and 10K pulldown are connected to a0
int fsrReading;     // the analog reading from the FSR resistor divider
int fsrVoltage;     // the analog reading converted to voltage
unsigned long fsrResistance;  // The voltage converted to resistance, can be very big so make "long"
unsigned long fsrConductance;
long fsrForce;
float  fsrMap;

//for PID (lib)
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);


//LIKE TAKIZAWA
double targetPos = 0;
double actualPos = 0;
int dig1 = 0;
int key1 = 0;
double r1_k = 0.5;
double actualPos1 = 0;


//AVERAGE SENSOR
const int numReadings = 100;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

//TEST PID
float targetPosi = 0;
int position_p = 0;
int position_i = 0;
int position_d = 0;
int pre_sensorRead = 0;
//double kp31_f = 0.2, kd31_f = 0.05, ki31_f = 0.00005; // double kp31_f = 0.3, kd31_f = 0.15, ki31_f = 0.00005; //original
//double kp31_f = 0.45, kd31_f = 0.2, ki31_f = 0.000005;
double kp31_f = 0.75, kd31_f = 0.15, ki31_f = 0.00005;


//double kp31_b = 0.05, kd31_b = 0.2, ki31_b = 0.00005; //kp31_b = 0.15, kd31_b = 0.3, ki31_b = 0.00005; //original
//double kp31_b = 0.2, kd31_b = 0.45, ki31_b = 0.000005;
double kp31_b = 0.15, kd31_b = 0.75, ki31_b = 0.00005;

int sensorReading = 0;
int motor31 = 0;

int sensorRead1 = 0;


int targetPosiB;

double sumPosi;

int stepsPerSec;

double forze;

double stiff;

int checKey1 = 0;
int stopDanger = 0;


//------ THIS
int actualPosition;
int motorSpeed = 0;
int position2reach = 0;
double mSpeed = 0;

#define INTERVAL_MESSAGE1 1000
#define INTERVAL_MESSAGE2 5000
#define INTERVAL_MESSAGE3 1800
#define INTERVAL_MESSAGE4 2000

unsigned long time_1 = 0;
unsigned long time_2 = 0;
unsigned long time_3 = 0;
unsigned long time_4 = 0;

int target2;

int fsrOnOff;
int targetPosiMax;

float posVelo;
float posAcc;


void setup()
{

  pinMode(ledPin, OUTPUT);

  // Start serial
  SerialUSB.begin(9600);
  //fo like TAKIZAWA
  /*
   delay(500);
   while(1){
   if(SerialUSB.available()>0)
     if(SerialUSB.read()=='a'){
      SerialUSB.print("ok");
      SerialUSB.println("\t");
       break;
     }
 }*/

  SerialUSB.println("powerSTEP01 Arduino control initialising...");

  // Prepare pins
  pinMode(nSTBY_nRESET_PIN, OUTPUT);
  pinMode(nCS_PIN, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, OUTPUT);
  pinMode(SCK, OUTPUT);

  // Reset powerSTEP and set CS
  digitalWrite(nSTBY_nRESET_PIN, HIGH);
  digitalWrite(nSTBY_nRESET_PIN, LOW);
  digitalWrite(nSTBY_nRESET_PIN, HIGH);
  digitalWrite(nCS_PIN, HIGH);

  // Start SPI
  altSPI.begin();
  pinPeripheral(11, PIO_SERCOM);
  pinPeripheral(12, PIO_SERCOM);
  pinPeripheral(13, PIO_SERCOM);
  altSPI.setDataMode(SPI_MODE3);
  // Configure powerSTEP
  driver.SPIPortConnect(&altSPI); // give library the SPI port (only the one on an Uno)

  driver.configSyncPin(BUSY_PIN, 0); // use SYNC/nBUSY pin as nBUSY,
  // thus syncSteps (2nd paramater) does nothing

  driver.configStepMode(STEP_FS_128); // 1/128 microstepping, full steps = STEP_FS,
  // options: 1, 1/2, 1/4, 1/8, 1/16, 1/32, 1/64, 1/128

  driver.setMaxSpeed(5000); // max speed in units of full steps/s
  driver.setFullSpeed(6000); // full steps/s threshold for disabling microstepping
  driver.setAcc(6000); // 200 full steps/s^2 acceleration
  driver.setDec(6000); // 200 full steps/s^2 deceleration

  driver.setSlewRate(SR_980V_us); // faster may give more torque (but also EM noise),
  // options are: 114, 220, 400, 520, 790, 980(V/us)

  driver.setOCThreshold(8); // over-current threshold for the 2.8A NEMA23 motor
  // used in testing. If your motor stops working for
  // no apparent reason, it's probably this. Start low
  // and increase until it doesn't trip, then maybe
  // add one to avoid misfires. Can prevent catastrophic
  // failures caused by shorts
  driver.setOCShutdown(OC_SD_ENABLE); // shutdown motor bridge on over-current event
  // to protect against permanant damage

  driver.setPWMFreq(PWM_DIV_1, PWM_MUL_0_75); // 16MHz*0.75/(512*1) = 23.4375kHz //0_75
  // power is supplied to stepper phases as a sin wave,
  // frequency is set by two PWM modulators,
  // Fpwm = Fosc*m/(512*N), N and m are set by DIV and MUL,
  // options: DIV: 1, 2, 3, 4, 5, 6, 7,
  // MUL: 0.625, 0.75, 0.875, 1, 1.25, 1.5, 1.75, 2

  driver.setVoltageComp(VS_COMP_DISABLE); // no compensation for variation in Vs as
  // ADC voltage divider is not populated

  driver.setSwitchMode(SW_USER); // switch doesn't trigger stop, status can be read.
  // SW_HARD_STOP: TP1 causes hard stop on connection
  // to GND, you get stuck on switch after homing

  driver.setOscMode(INT_16MHZ); // 16MHz internal oscillator as clock source

  // KVAL registers set the power to the motor by adjusting the PWM duty cycle,
  // use a value between 0-255 where 0 = no power, 255 = full power.
  // Start low and monitor the motor temperature until you find a safe balance
  // between power and temperature. Only use what you need
  driver.setRunKVAL(255); //64
  driver.setAccKVAL(255); //64
  driver.setDecKVAL(255); //64
  driver.setHoldKVAL(150); //32

  driver.setParam(ALARM_EN, 0x8F); // disable ADC UVLO (divider not populated),
  // disable stall detection (not configured),
  // disable switch (not using as hard stop)

  driver.getStatus(); // clears error flags

  SerialUSB.println(F("Initialisation complete"));

    Ethernet.begin(mac, myIp);
    Udp.begin(inPort);

   //for ToF sensor
   Wire.begin();
   sensor.init();
   sensor.setTimeout(500);

   //CONTINUOUS MODE

   // Start continuous back-to-back mode (take readings as
   // fast as possible).  To use continuous timed mode
   // instead, provide a desired inter-measurement period in
   // ms (e.g. sensor.startContinuous(100)).
   sensor.startContinuous(100);


   //SINGLE MODE
   /*
   #if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
#endif
   */

   //for switch
   pinMode(A1, INPUT);

   //for FSR
   pinMode(A0, INPUT);

   //for PID (lib)
   //Setpoint = 100;//100;
   //turn the PID on
   myPID.SetMode(AUTOMATIC);


   //Smoothing Average
   for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  //THIS
  //actualPosition = 0;

  //fsrOnOff = 0;

  //SerialUSB.print("ou");

  readSwitch = 0;

}
//------------ END START


void sendOneData(char *address, int32_t data) {
  OSCMessage newMes(address);
  newMes.add((int32_t)data);
  Udp.beginPacket(destIp, outPort);
  newMes.send(Udp);
  Udp.endPacket();
  newMes.empty();
}

void setDestIp(OSCMessage &msg ,int addrOffset) {
  destIp = Udp.remoteIP();
  sendOneData("/newDestIp", (int32_t)destIp[3]);
  digitalWrite(ledPin, !digitalRead(ledPin));
}

/*
void setKVAL(OSCMessage &msg ,int addrOffset) {
  int t = msg.getInt(0);
  t = constrain(t,0,255);
  driver.setHoldKVAL(t);
  t = msg.getInt(1);
  t = constrain(t,0,255);
  driver.setRunKVAL(t);
  t = msg.getInt(2);
  t = constrain(t,0,255);
  driver.setAccKVAL(t);
  t = msg.getInt(3);
  t = constrain(t,0,255);
  driver.setDecKVAL(t);
}

void setSpdProfile(OSCMessage &msg ,int addrOffset) {
  float t = msg.getFloat(0);
  driver.setAcc(t);
  t = msg.getFloat(1);
  driver.setDec(t);
  t = msg.getFloat(2);
  driver.setMaxSpeed(t);
}

void run(OSCMessage &msg ,int addrOffset) {
  float spd = msg.getFloat(0);
  boolean dir = spd>0;
  driver.run(dir,abs(spd));
}

void getPos(OSCMessage &msg ,int addrOffset) {
  sendOneData("/pos", driver.getPos());
}

void setPos(OSCMessage &msg ,int addrOffset) {
  long t = msg.getInt(0);
  driver.setPos(t);
}
*/

//MY OSC RECEIVE
void OSCMsgReceive(){
  OSCMessage msgIN;
  int size;
  if((size = Udp.parsePacket())>0){
    while(size--)
      msgIN.fill(Udp.read());
    if(!msgIN.hasError()){
      msgIN.route("/positionTarget",positionTarget);
      msgIN.route("/velocity",positionVelocity);
      msgIN.route("/acceleration",positionAccel);
      msgIN.route("/keyCheck",keyCheck);
      msgIN.route("/dangerStop",dangerStop);


      //msgIN.route("/inputSpeed",speedFromMax);
      //msgIN.route("/Fader/Value",funcValue);
    }
  }
}


void positionTarget(OSCMessage &msg, int addrOffset){
  targetPosiMax = msg.getInt(0);
  //OSCMessage msgOUT("/inputPosition");
}

void positionVelocity(OSCMessage &msg, int addrOffset){
  posVelo = msg.getFloat(0);
  //OSCMessage msgOUT("/inputPosition");
}

void positionAccel(OSCMessage &msg, int addrOffset){
  posAcc = msg.getFloat(0);
  //OSCMessage msgOUT("/inputPosition");
}

void keyCheck(OSCMessage &msg, int addrOffset){
  checKey1 = msg.getInt(0);
  //OSCMessage msgOUT("/inputPosition");
}

void dangerStop(OSCMessage &msg, int addrOffset){
  stopDanger = msg.getInt(0);
  //OSCMessage msgOUT("/inputPosition");
}


void loop() {

OSCMsgReceive();

key1 = checKey1;

//actual position
long sensorRead = driver.getPos();
//map
actualPosition = (int(sensorRead / 1000) *(-1));
actualPosition = map(actualPosition, 0, 335, 0, 128);


//For FSR
   fsrReading = analogRead(fsrPin);
  // analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
  fsrVoltage = map(fsrReading, 0, 1023, 0, 5000);

  if (fsrVoltage == 0) {
    //SerialUSB.println("No pressure");
  } else {
    // The voltage = Vcc * R / (R + FSR) where R = 10K and Vcc = 5V
    // so FSR = ((Vcc - V) * R) / V        yay math!
    fsrResistance = 5000 - fsrVoltage;     // fsrVoltage is in millivolts so 5V = 5000mV
    fsrResistance *= 10000;                // 10K resistor
    fsrResistance /= fsrVoltage;

    fsrConductance = 1000000;           // we measure in micromhos so
    fsrConductance /= fsrResistance;

    // Use the two FSR guide graphs to approximate the force
    if (fsrConductance <= 1000) {
      fsrForce = fsrConductance / 80;
      //SerialUSB.println(fsrForce);   //in Newtons
    } else {
      fsrForce = fsrConductance - 1000;
      fsrForce /= 30;
      //SerialUSB.println(fsrForce);      //in Newtons
    }
  }

fsrMap = map(abs(fsrReading), 0, 1023, 0, 50); //0, 20

int stepPos = 0;
stiff = 1.5; //0.8
int targetPosi1;



if(fsrMap < 10) {

   fsrOnOff = 0;

   targetPosi = targetPosiMax;
   key1 = 1;

/*
// TARGET POSITION
   if(SerialUSB.read()=='q'){

     targetPosi = 50; //5
     //stepPos = 50;
     SerialUSB.println(targetPosi);
     key1 = 1;


  }

   else if(SerialUSB.read()=='w'){

     targetPosi = 127;//200;//+=10; //1
     SerialUSB.println(targetPosi);
     key1 = 1;

  }

  else if(SerialUSB.read()=='e'){

     targetPosi = 100;//-=10; //1
     SerialUSB.println(targetPosi);
     key1 = 1;

  }

   else if(SerialUSB.read()=='r'){

     targetPosi = 10;//+=25; //5
     SerialUSB.println(targetPosi);
     key1 = 1;

  }

   else if(SerialUSB.read()=='t'){

     targetPosi = 2; //5
     SerialUSB.println(targetPosi);
     key1 = 1;

  }

  else if(SerialUSB.read()=='y'){

     targetPosi = 100 ; //5
     SerialUSB.println(targetPosi);
     key1 = 1;

  }
*/

   //test posVelo  or   posAcc
   mSpeed = 950;//950; //500 previous
   target2 = targetPosi;

   targetPosi1 = 0;

} else {

  fsrOnOff = 1;
  //targetPosi = targetPosi1;
  //sumPosi -= ((targetPosi + fsrMap) - sensorReading);
  forze = (-stiff) * (fsrConductance/1000);  //Hook's Law (?)
  //targetPosi = (target2 - fsrMap); //+ fsrMap //THIS

  targetPosi = (target2 - fsrMap);
  sendOneData("/positionFSR", targetPosi1); //OSC

  mSpeed += forze*1.5;//fsrConductance/10; *5
  //bordeo();
  //mSpeed = 100;
  //key1 = 1;

  //position2reach = targetPosi;

}

/*
if(millis() > time_1 + INTERVAL_MESSAGE1){
        time_1 = millis();
        targetPosi = 50;
    }

    if(millis() > time_2 + INTERVAL_MESSAGE2){
      time_2 = millis();
        targetPosi = 200;
    }

    if(millis() > time_3 + INTERVAL_MESSAGE3){
      time_3 = millis();
        targetPosi = 50;
    }

    if(millis() > time_4 + INTERVAL_MESSAGE4){
      time_4 = millis();
        targetPosi = 30;
    }
  */

  //mSpeed = 500;

  sendOneData("/fsr_value", int(fsrMap)); //OSC




  //Target Position
  position2reach = targetPosi;

  sumPosi = (position2reach - actualPosition);

  //MOVE
  //moveStepLeft();
  //moveStepRight();
  goToPosition();
  //bordeo();
  //checkSwitch();
  directStop();

  //driver.run(FWD, 100);
  sendOneData("/touchOnOff", int(fsrOnOff)); //OSC

  sendOneData("/actualPosition", int(actualPosition)); //OSC

  readSwitch = digitalRead(A1);
  sendOneData("/switchOnOff", int(readSwitch)); //OSC

  SerialUSB.print(actualPosition); //step counter
  SerialUSB.print("\t");
  SerialUSB.print(position2reach); //input position
  SerialUSB.print("\t");
  SerialUSB.print((actualPosition - position2reach));
  SerialUSB.print("\t");
  SerialUSB.print((position2reach - actualPosition));
  SerialUSB.print("\t");
  SerialUSB.print(target2);
  SerialUSB.print("\t");
  SerialUSB.print(fsrMap);
  SerialUSB.print("\t");
  SerialUSB.println(targetPosiMax);
  //SerialUSB.print("\t");
  //SerialUSB.println(sensorRead1);



}




void moveStepLeft() {

  position2reach = actualPosition - 1000;

}

void moveStepRight() {

  position2reach = actualPosition + 1000;

}


void goToPosition() {

if(key1 == 1){

  if(actualPosition < position2reach) {

    moveMotorLeft();
    //digitalWrite(nSTBY_nRESET_PIN, HIGH);
    //digitalWrite(pinSTBY, HIGH);

  }

  if(actualPosition > position2reach) {

    moveMotorRight();
    //digitalWrite(nSTBY_nRESET_PIN, HIGH);

  }

  if(actualPosition == position2reach) {

    //motorStop(motor1);
    //digitalWrite(nSTBY_nRESET_PIN, LOW);
    driver.softStop();

  }
}

}


void moveMotorLeft() {

  //if((position2reach - actualPosition) > 1) { //> 100
  if(sumPosi > 1){

    motorSpeed = mSpeed; //diff; //= 100;

  } else {

    motorSpeed = 0;

  }

  driver.run(REV, motorSpeed);
  //motorDrive(motor1, turnCCW, motorSpeed);

}

void moveMotorRight() {

 // if((actualPosition - position2reach) < -1) { //< -100
    //if((position2reach - actualPosition) < -1){
    if(sumPosi < -1){
    motorSpeed = mSpeed;//diff;//= 100;

  } else {

    motorSpeed = 0;

  }

  driver.run(FWD, motorSpeed);
  //motorDrive(motor1, turnCW, motorSpeed);

}


void bordeo() {

  driver.setAcc(posAcc); // 200 full steps/s^2 acceleration
  driver.setDec(posAcc); // 200 full steps/s^2 deceleration


}

void checkSwitch(){

  if(readSwitch = 1){

    actualPosition = 0;
    //position2reach = actualPosition - position2reach)

  }

}

void directStop(){
  if(stopDanger == 1){
    driver.hardStop();
  }
}
