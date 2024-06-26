#include <ECE3.h>
#include <stdio.h>

#define RAMP_SPEED (0.0005)
#define NUM_PREVIOUS_ERRORS (3)
#define NUM_PREVIOUS_SUMS (4)


const int left_nslp_pin = 31; // nslp HIGH ==> awake & ready for PWM
const int right_nslp_pin = 11; // nslp HIGH ==> awake & ready for PWM
const int left_dir_pin = 29;
const int right_dir_pin = 30;
const int left_pwm_pin = 40;
const int right_pwm_pin = 39;

const int LED_RF = 75;
int wheelSpd = 80;
int distance = 300;

uint16_t sensorCalibrationMins[8] = {827, 617, 663, 640, 640, 594, 686, 709};
uint16_t sensorCalibrationMaxs[8] = {2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500};

const float Kp = 0.4;
const float Ki = 0.0;
const float Kd = 7.0;

enum MovementState { LINE_FOLLOWING_FORWARD, TURNING_AROUND, LINE_FOLLOWING_BACK, FINISHED };

typedef struct {
  float currentForwardSpeed = 0.0;
  float currentTurnSpeed = 0.0;
  float currentError = 0.0;
  float previousErrors[NUM_PREVIOUS_ERRORS] = {0.0};
  uint16_t sensorValues[8] = {0};
  int previousSums[NUM_PREVIOUS_SUMS] = {0};
  MovementState movementState = LINE_FOLLOWING_FORWARD;
} CarState;

CarState state;


void setup()
{
  ECE3_Init();

  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);
  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  //  pinMode(13,INPUT);

  pinMode(LED_RF, OUTPUT);

  digitalWrite(left_nslp_pin, HIGH);
  digitalWrite(right_nslp_pin, HIGH);
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
  float straightSpeed = state.currentForwardSpeed;
  float turnSpeed = state.currentTurnSpeed;
//  Serial.print("straight speed before:\t ");
//  Serial.println(straightSpeed);
  straightSpeed = max(-1.0, min(1.0, straightSpeed));
  turnSpeed = max(-1.0, min(1.0, turnSpeed));
//  Serial.print("straight speed after:\t ");
//  Serial.println(straightSpeed);

  float rightWheelSpeed = straightSpeed;// - (0.5 * turnSpeed);
  float leftWheelSpeed = straightSpeed;// + (0.5 * turnSpeed);
//  Serial.print("turn calc: ");
//  Serial.print(0.5f * (float)turnSpeed);
//  Serial.print(" right wheel before: ");
//  Serial.print(rightWheelSpeed);
  if(fabs(turnSpeed) > 0.001) {
    rightWheelSpeed -= (0.5f * turnSpeed);
    leftWheelSpeed += (0.5f * turnSpeed);
  }

//  Serial.print(" right wheel after: ");
//  Serial.println(rightWheelSpeed);
  if (leftWheelSpeed < 0.0) {
    digitalWrite(left_dir_pin, HIGH);
  } else {
    digitalWrite(left_dir_pin, LOW);
  }
  leftWheelSpeed = fabs(leftWheelSpeed);

  if (rightWheelSpeed < 0.0) {
    digitalWrite(right_dir_pin, HIGH);
  } else {
    digitalWrite(right_dir_pin, LOW);
  }
  rightWheelSpeed = fabs(rightWheelSpeed);
//  Serial.println((int)(leftWheelSpeed * 255.0));
  analogWrite(left_pwm_pin, (int)(leftWheelSpeed * 255.0));
  analogWrite(right_pwm_pin, (int)(rightWheelSpeed * 255.0));

}


// getEncoderCount_right();


// increment is always positive
void rampSpeedTo(float &currentSpeed, float toSpeed, float increment) {
  if (toSpeed >= 0.0) {
    currentSpeed += increment;
    currentSpeed = min(currentSpeed, toSpeed);
  } else {
    currentSpeed -= increment;
    currentSpeed = max(currentSpeed, toSpeed);
  }
}

float currentForwardSpeed = 0.0;
float currentTurnSpeed = 0.0;

void calcTurnSpeedPD() {
  float p = state.currentError;
  float d = 0.0;
  d += state.currentError - state.previousErrors[0];
  for (int i = 0; i < NUM_PREVIOUS_ERRORS - 1; i++) {
    d += state.previousErrors[i] - state.previousErrors[i + 1];
  }
  d /= NUM_PREVIOUS_ERRORS;

  state.currentTurnSpeed = p * Kp + d * Kd;
}

void updateSensors() {
  float fusion[8] = {0.25, 0.75, 0.5, 0.25, -0.25, -0.5, -0.75, -0.25};
  float weightedsum = 0.0;
  int sum = 0;
  float totalsum = 0.0;
  for (int i = 0; i < 8; i++) {
    sum += state.sensorValues[i];
    weightedsum += fusion[i] * state.sensorValues[i];
    totalsum += state.sensorValues[i];
  }

  for (int i = 0; i < NUM_PREVIOUS_SUMS - 1; i++) {
    state.previousSums[i+1] = state.previousSums[i];
  }
  state.previousSums[0] = sum;
  
  float previousError = state.currentError;
  if (totalsum == 0) {
    state.currentError = 0;
  }
  state.currentError = weightedsum / totalsum;
  for (int i = 0; i < NUM_PREVIOUS_ERRORS - 1; i++) {
    state.previousErrors[i + 1] = state.previousErrors[i];
  }
  state.previousErrors[0] = previousError;
}

void normalize() {
  for (int i = 0; i < 8; i++) {
    if (state.sensorValues[i] > sensorCalibrationMins[i]) {
      state.sensorValues[i] = (((state.sensorValues[i]) - sensorCalibrationMins[i]) * 1000) / sensorCalibrationMaxs[i];
    } else {
      state.sensorValues[i] = 0;
    }
  }
}

bool onFinish() {
  int avg = 0;
  for (int i = 0; i < NUM_PREVIOUS_SUMS; i++) {
     if(state.previousSums[i] < 3500) return false;
  }
  return true;
}

void stopMove() {
  state.currentForwardSpeed = 0.0;
  state.currentTurnSpeed = 0.0;
  move();
}

void printError() {
  Serial.print(state.currentError);
  Serial.print('\n');
}

void printErrorChanges() {
  Serial.print(state.currentError);
  Serial.print('\t');
  for (int i = 0; i < NUM_PREVIOUS_ERRORS; i++) {
    Serial.print(state.previousErrors[i]);
    Serial.print('\t');
  }
  Serial.print('\n');
}

void printTurnSpeed() {
  Serial.print(state.currentTurnSpeed);
  Serial.print('\n');
}

void loop() {
  ECE3_read_IR(state.sensorValues);
  normalize();
  updateSensors();

  switch (state.movementState) {
    case LINE_FOLLOWING_FORWARD:
      if (onFinish()) {
//        Serial.print("yes\n");
        state.movementState = TURNING_AROUND;
        stopMove();
        resetEncoderCount_left();
        resetEncoderCount_right();
        return;
      }

      calcTurnSpeedPD();
      rampSpeedTo(state.currentForwardSpeed, 0.06, RAMP_SPEED);

      break;
    case TURNING_AROUND:
//      Serial.print("turning\n");
      if(getEncoderCount_left() > 350) {
        state.movementState = LINE_FOLLOWING_BACK;
        stopMove();
        return;
      }
      rampSpeedTo(state.currentTurnSpeed, 0.2, RAMP_SPEED);
      break;
    case LINE_FOLLOWING_BACK:
        //  Serial.print(state.currentError);
      //Serial.print('\n');
//          Serial.print("going back\n");

      if (onFinish()) {
//        Serial.print("yes\n");
        state.movementState = FINISHED;
        stopMove();
        return;
      }
      calcTurnSpeedPD();
      rampSpeedTo(state.currentForwardSpeed, 0.06, RAMP_SPEED);
      //Serial.println(state.currentForwardSpeed);
      break;
    case FINISHED:
//          Serial.print("finished\n");

      stopMove();
      return;
  }

  move();
}
