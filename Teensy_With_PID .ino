#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

const int pwm1L = 2, pwm1R = 3;
const int pwm2L = 4, pwm2R = 5;
const int pwm3L = 6, pwm3R = 7;
const int pwm4L = 8, pwm4R = 9;

#define SLAVE_ADDRESS 0x08
int8_t ps4data[3];  // Vx, Vy, w
float V1, V2, V3, V4;
float r = 0.37338;  // Wheel radius factor

// PID Variables
float kP = 3, kI = 0.0055, kD = 300;  // PID gains kP = 3, kI = 0.0055,
float targetAngle = 0;  // Desired angle (setpoint)
float currentAngle = 0, error = 0, errorOld = 0, errorSum = 0, errorRate = 0 , errorSlop = 0;
float pidOutput = 0;

// BNO055 setup
Adafruit_BNO055 bno = Adafruit_BNO055(55);

int milliold;
int millinew;
int dt;

// Function to normalize angles to the range -180 to 180
float normalizeAngle(float angle) {
  while (angle > 180) angle -= 360;
  while (angle <= -180) angle += 360;
  return angle;
}

void setup() {
  Wire1.begin();         // Initialize as I2C master
  Serial.begin(115200);  // Serial communication

  // BNO055 Initialization
  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check wiring!");
    while (1);
  }
  bno.setExtCrystalUse(true);
  millinew = millis();
  // Motor control pin setup
  setupMotors();
  Serial.println("Teensy Master Ready");
  
}

void loop() {
  // Read BNO055 Euler angle
  sensors_event_t event;
  bno.getEvent(&event);
  currentAngle = normalizeAngle(event.orientation.x);

  // Calculate error and normalize P
  
  error = normalizeAngle(targetAngle - currentAngle);
  
  if (abs(error) <= 1){ 
    error = 0;
    errorSum = 0;  
    errorRate = 0;
    pidOutput = 0;
  }

  // PID calculations
  milliold = millinew;
  millinew = millis();
  dt = millinew - milliold;

  errorSum += error * dt;  // I
  
  errorRate = (error - errorOld)/dt;// D

  
  pidOutput = kP * error + kI * errorSum + kD * errorRate;  // PID formula
  errorOld = error;  // Update old error
  

  pidOutput = constrain(pidOutput, -100, 100);

  Wire1.requestFrom(SLAVE_ADDRESS, sizeof(ps4data));
  int i = 0;
  while (Wire1.available()) {
    ps4data[i++] = Wire1.read();
  }

  int8_t Vx = ps4data[0];
  int8_t Vy = ps4data[1];
  int8_t w = ps4data[2]; 

  Vx = map(Vx, -127, 127, -100, 100);
  Vy = map(Vy, -127, 127, -100, 100);
  w = map(w, -127, 127, -100, 100);

  if (w !=  0) {
    targetAngle = normalizeAngle(currentAngle); // Update only when joystick gives rotation input
} else {
    w += pidOutput; // Apply PID correction when no joystick rotation
}


  // Calculate wheel velocities
  V1 = w * r + (Vy / 1.41421356237) + (Vx / 1.41421356237);
  V2 = w * r + (Vx / 1.41421356237) - (Vy / 1.41421356237);
  V3 = w * r - (Vx / 1.41421356237) - (Vy / 1.41421356237);
  V4 = w * r + (Vy / 1.41421356237) - (Vx / 1.41421356237);

  // Drive motors
  driveMotor(V1, pwm1L, pwm1R);
  driveMotor(V2, pwm2L, pwm2R);
  driveMotor(V3, pwm3L, pwm3R);
  driveMotor(V4, pwm4L, pwm4R);

  //Print debug info
  Serial.print(V1); Serial.print("\t");
  Serial.print(V2); Serial.print("\t");
  Serial.print(V3); Serial.print("\t");
  Serial.print(V4); Serial.print("\t");
  Serial.print(targetAngle); Serial.print("\t");
  Serial.println(currentAngle); Serial.print("\t");
  Serial.print(error); Serial.print("\t");
  Serial.print(errorRate); Serial.print("\t");
  Serial.print(pidOutput); Serial.println();

  delay(5);
}

void setupMotors() {
  pinMode(pwm1L, OUTPUT); pinMode(pwm1R, OUTPUT);
  pinMode(pwm2L, OUTPUT); pinMode(pwm2R, OUTPUT);
  pinMode(pwm3L, OUTPUT); pinMode(pwm3R, OUTPUT); 
  pinMode(pwm4L, OUTPUT); pinMode(pwm4R, OUTPUT); 
}

void driveMotor(float speed, int pwmL, int pwmR) {
  int pwmValue = abs(speed);
  pwmValue = constrain(pwmValue, 0, 100);

  if (speed > 0) {  // Forward
    analogWrite(pwmL, pwmValue); analogWrite(pwmR, 0);
  } else if (speed < 0) {  // Backward
    analogWrite(pwmL, 0); analogWrite(pwmR, pwmValue);
  } else {  // Stop
    analogWrite(pwmL, 0); analogWrite(pwmR, 0);
  }
}
