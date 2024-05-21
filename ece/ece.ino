#include <ECE3.h>
#include <stdio.h>

#define RAMP_SPEED (0.00001)
#define NUM_PREVIOUS_ERRORS (3)
const int left_nslp_pin=31; // nslp HIGH ==> awake & ready for PWM
const int right_nslp_pin=11; // nslp HIGH ==> awake & ready for PWM
const int left_dir_pin=29;
const int right_dir_pin=30;
const int left_pwm_pin=40;
const int right_pwm_pin=39;

const int LED_RF = 75;
int wheelSpd = 80;
int distance = 300;

uint16_t sensorCalibrationMins[8] = {827, 617, 663, 640, 640, 594, 686, 709};
uint16_t sensorCalibrationMaxs[8] = {2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500};

const float Kp = 4.0;
const float Ki = 0.5;
const float Kd = 2.0;

typedef struct {
  float currentForwardSpeed = 0.0;
  float currentTurnSpeed = 0.0;
  float currentError = 0.0;
  float previousErrors[NUM_PREVIOUS_ERRORS] = {0.0};
  uint16_t sensorValues[8] = {0};
} CarState;

CarState * const state = new CarState();


void setup()
{
  ECE3_Init();
  
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

//  pinMode(13,INPUT);

  pinMode(LED_RF, OUTPUT);

  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_nslp_pin,HIGH);
//  digitalWrite(left_nslp_pin,LOW);
//  digitalWrite(right_nslp_pin,LOW);
  
// set the data rate in bits/second for serial data transmission
  Serial.begin(9600); 

  resetEncoderCount_left();
  resetEncoderCount_right();

  delay(2000); //Wait 2 seconds before starting 
  
}

// speed is -1.0 to 1.0
// turn clockwise is positive
void move() {
  float straightSpeed = state->currentForwardSpeed;
  float turnSpeed = state->currentTurnSpeed;

  straightSpeed = max(-1.0, min(1.0, straightSpeed));
  turnSpeed = max(-1.0, min(1.0, turnSpeed));

  float rightWheelSpeed = straightSpeed - 0.5 * turnSpeed;
  float leftWheelSpeed = straightSpeed + 0.5 * turnSpeed;

  if(leftWheelSpeed < 0.0) {
    digitalWrite(left_dir_pin, HIGH);
  } else {
    digitalWrite(left_dir_pin, LOW);
  }
  leftWheelSpeed = abs(leftWheelSpeed);

  if(rightWheelSpeed < 0.0) {
    digitalWrite(right_dir_pin, HIGH);
  } else {
    digitalWrite(right_dir_pin, LOW);
  }
  rightWheelSpeed = abs(rightWheelSpeed);
  
  analogWrite(left_pwm_pin, (int)(leftWheelSpeed * 255.0));    
  analogWrite(right_pwm_pin, (int)(rightWheelSpeed * 255.0)); 
  
}


// getEncoderCount_right();


// increment is always positive
void rampSpeedTo(float &currentSpeed, float toSpeed, float increment) {
  if(toSpeed >= 0.0) {
    currentSpeed += increment;
    currentSpeed = min(currentSpeed, toSpeed);
  } else {
    currentSpeed -= increment;
    currentSpeed = max(currentSpeed, toSpeed);
  }
}

float currentForwardSpeed = 0.0;
float currentTurnSpeed = 0.0;

float pid() {
  return 0.0;
}

void updateErrors(){
  int fusion[8] = {-4000, -3000, -2000, -1000, 1000, 2000, 3000, 4000};
  int weightedsum = 0;
  int totalsum = 0;
  for(int i=0; i<8; i++){
    weightedsum+=fusion[i]*state->sensorValues[i];
    totalsum += state->sensorValues[i];
  }
  float previousError = state->currentError;
  if (totalsum == 0){
    state->currentError = 0;
  }
  state->currentError = weightedsum/totalsum;
  for (int i = 0; i < NUM_PREVIOUS_ERRORS - 2; i++){
    state->previousErrors[i+1] = state->previousErrors[i]; 
  }
  state->previousErrors[0] = previousError;
}

void normalize(){
  for(int i=0; i<8; i++){
    if (state->sensorValues[i] > sensorCalibrationMins[i]){
      state->sensorValues[i] = (((state->sensorValues[i])-sensorCalibrationMins[i])*1000)/sensorCalibrationMaxs[i];
    } else {
      state->sensorValues[i] = 0;
    }
  }
}


void loop() {
  //ECE3_read_IR(state->sensorValues);
  //normalize();
  //updateErrors();
  rampSpeedTo(state->currentForwardSpeed, 0.5, RAMP_SPEED);
  move();
}
