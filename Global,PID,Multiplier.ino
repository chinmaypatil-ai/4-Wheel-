// Feb 10 Some updates in Constrain and Maping

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
float radian;
float theta;
float Vgx , Vgy;
float r = 0.37338;  // Wheel radius factor

// PID Variables
float kP = 3, kI = 0.0055, kD = 300;  // PID gains
float targetAngle = 0;  // Desired angle (setpoint)
float currentAngle = 0, error = 0, errorOld = 0, errorSum = 0, errorRate = 0;
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
  theta = normalizeAngle(event.orientation.x);
  currentAngle = normalizeAngle(event.orientation.x);

  // Calculate error and normalize PID
  error = normalizeAngle(targetAngle - currentAngle);
  
  if (abs(error) <= 1) { 
    error = 0;
    errorSum = 0;  
    errorRate = 0;
    pidOutput = 0;
  }

  // PID calculations
  milliold = millinew;
  millinew = millis();
  dt = millinew - milliold;

  errorSum += error * dt;  // Integral
  errorRate = (error - errorOld) / dt;  // Derivative
  
  pidOutput = kP * error + kI * errorSum + kD * errorRate;  // PID formula
  errorOld = error;  // Update old error

  pidOutput = constrain(pidOutput, -120, 120);

  // Read joystick data from slave
  Wire1.requestFrom(SLAVE_ADDRESS, sizeof(ps4data));
  int i = 0;
  while (Wire1.available()) {
    ps4data[i++] = Wire1.read();
  }

  int8_t Vx = ps4data[0];
  int8_t Vy = ps4data[1];
  int8_t w = ps4data[2];

  // Map joystick values to -100 to 100 range
  Vx = map(Vx, -127, 127, -90, 90);
  Vy = map(Vy, -127, 127, -90, 90);
  w = map(w, -127, 127, -90, 90);

  if (w !=  0) {
    targetAngle = normalizeAngle(currentAngle);  // Update only when joystick gives rotation input
  } else {
    w += pidOutput;  // Apply PID correction when no joystick rotation
  }
  
  // Global to local transformation
  radian = theta * (3.141592 / 180);  
  Vgx = (cos(radian) * Vx) - (sin(radian) * Vy);  
  Vgy = (sin(radian) * Vx) + (cos(radian) * Vy);  

  // Calculate wheel velocities
  float velo_const = 1.2;
  V1 = velo_const*(w * r + (Vgy / 1.41421356237) + (Vgx / 1.41421356237));
  V2 = velo_const*(w * r + (Vgx / 1.41421356237) - (Vgy / 1.41421356237));
  V3 = velo_const*(w * r - (Vgx / 1.41421356237) - (Vgy / 1.41421356237));
  V4 = velo_const*(w * r + (Vgy / 1.41421356237) - (Vgx / 1.41421356237));

  // Drive motors
  driveMotor(V1, pwm1L, pwm1R);
  driveMotor(V2, pwm2L, pwm2R);
  driveMotor(V3, pwm3L, pwm3R);
  driveMotor(V4, pwm4L, pwm4R);

  // Print debug info
  Serial.print(V1); Serial.print("\t");
  Serial.print(V2); Serial.print("\t");
  Serial.print(V3); Serial.print("\t");
  Serial.print(V4); Serial.print("\t");
  Serial.print(targetAngle); Serial.print("\t");
  Serial.print(currentAngle); Serial.print("\t");
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
  pwmValue = constrain(pwmValue, 0, 120);

  if (speed > 0) {  // Forward
    analogWrite(pwmL, pwmValue); analogWrite(pwmR, 0);
  } else if (speed < 0) {  // Backward
    analogWrite(pwmL, 0); analogWrite(pwmR, pwmValue);
  } else {  // Stop
    analogWrite(pwmL, 0); analogWrite(pwmR, 0); 
  }
  // Serial.println(pwmValue); //Serial.print("\t");

  // Serial.print(V2); Serial.print("\t");
  // Serial.print(V3); Serial.print("\t");
  // Serial.print(V4); Serial.print("\t");
}
