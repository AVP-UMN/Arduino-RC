
#include <Servo.h>

// Pin: Channel in
#define MOTOR_IN 2
#define STEER_IN 3
#define OVERRIDE_IN 4

//Pins: Channel Out
#define MOTOR_OUT 5
#define STEER_OUT 6
#define OVERRIDE_OUT 7

#define SOP 0
#define EOP 511


Servo servoMotor;
Servo servoSteer;
Servo servoOverride;

#define MOTOR_FLAG 1
#define STEER_FLAG 2
#define OVERRIDE_FLAG 4

volatile uint8_t updateFlagsShared;

volatile uint16_t motorPulseWidthShared;
volatile uint16_t steerPulseWidthShared;
volatile uint16_t overridePulseWidthShared;


volatile uint16_t test=0;;

uint32_t interruptStart;
uint32_t motorStart;
uint32_t steerStart;
uint32_t overrideStart;

void setup()
{
  Serial.begin(115200);
  pinMode(OVERRIDE_IN,INPUT);
  pinMode(OVERRIDE_OUT,OUTPUT);
  
  attachInterrupt(MOTOR_IN - 2,motorPWM,CHANGE); 
  attachInterrupt(STEER_IN - 2,steerPWM,CHANGE); 
}

void loop()
{
  static uint16_t motorPulseWidthLocal;
  static uint16_t steerPulseWidthLocal;
  static uint16_t overridePulseWidth;
  static uint8_t overrideState = LOW;
  
  static uint8_t updateFlags = 7;
  
  Serial.println("NextSet");
  //Serial.write(SOP);
  motorPulseWidthLocal = motorPulseWidthShared;
  //Serial.write(motorPulseWidthLocal);
  Serial.println(motorPulseWidthLocal);
  steerPulseWidthLocal = steerPulseWidthShared;
  //Serial.write(steerPulseWidthLocal);
  Serial.println(steerPulseWidthLocal);
  overridePulseWidth = pulseIn(4,HIGH,20000);
  if( overridePulseWidth > 1600 )
  { 
    overrideState = HIGH;
  }
  if( overridePulseWidth < 1200 )
  {
    overrideState = LOW;
  }
  //digitalWrite(OVERRIDE_OUT,overrideState);
  Serial.println(overridePulseWidth);
  Serial.println(overrideState);
  //Serial.write(EOP);
  
  
  //delay (1000);
  
  
  
}

//Upon an interrupt, this either records the time at the rising edge or 
//calculates the total 


void motorPWM()
{
  interruptStart = micros();
  test=1;
  if(digitalRead(MOTOR_IN) == HIGH)
  { 
    motorStart = interruptStart;
  }
  else
  {
    motorPulseWidthShared = (uint16_t)(interruptStart - motorStart);
    updateFlagsShared =  updateFlagsShared | MOTOR_FLAG;
  }
}

void steerPWM()
{
  interruptStart = micros();
  test=1;
  if(digitalRead(STEER_IN) == HIGH)
  { 
    steerStart = interruptStart;
  }
  else
  {
    steerPulseWidthShared = (uint16_t)(interruptStart - steerStart);
    updateFlagsShared =  updateFlagsShared | STEER_FLAG;
  }
}
