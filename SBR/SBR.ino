#include "MPU9250.h"
#define rightMotorDirPin 4
#define rightMotorPWMPin 5
#define leftMotorPWMPin 6
#define leftMotorDirPin 7
#define interruptPin 3
int motor_power = 0;
volatile double curr_angle = 0;
volatile double prev_angle = 0;
volatile double Ax = 0 , Ay = 0 , Az = 0;
volatile double Gy = 0;
volatile double elapsedTime = 0, currentTime = 0, previousTime = 0;
double error = 0, error_sum = 0;
double Kp = 20, Ki = 50, Kd = 10;
double set_angle = 0;
volatile bool dataReady = false;
//I2C Object Declaration
MPU9250 IMU(Wire, 0x68);
// gets data from mpu-9250 and calculates current angle
void getData()
{
  // read the sensor
  IMU.readSensor();
  Ax = (IMU.getAccelX_mss() / 9.81);
  Ay = (IMU.getAccelY_mss() / 9.81);
  Az = (IMU.getAccelZ_mss() / 9.81);
  Gy = IMU.getGyroY_rads() * (180 / PI);

  //calculates angle
  curr_angle = 0.98 * (prev_angle + Gy * elapsedTime) + 0.02 * (atan(-1 * Ax / sqrt(pow(Ay, 2) + pow(Az, 2) ) ) * (180 / PI));
  dataReady = true;
}
//sets motor power
void setMotors() {
  if (motor_power >= 0) {
    analogWrite(leftMotorPWMPin, motor_power);
    digitalWrite(leftMotorDirPin, LOW);
    analogWrite(rightMotorPWMPin, motor_power);
    digitalWrite(rightMotorDirPin, LOW);
  }
  else {
    analogWrite(leftMotorPWMPin, 255 + motor_power);
    digitalWrite(leftMotorDirPin, HIGH);
    analogWrite(rightMotorPWMPin, 255 + motor_power);
    digitalWrite(rightMotorDirPin, HIGH);
  }
}
//PID function that calculates motor power
void pid()
{
  error = curr_angle - set_angle;
  error_sum += error;
  motor_power = Kp * error + Ki * error_sum * elapsedTime + Kd * ( curr_angle - prev_angle ) / elapsedTime;
  prev_angle = curr_angle;
}
void setup() {
  pinMode(rightMotorDirPin, OUTPUT);
  pinMode(rightMotorPWMPin, OUTPUT);
  pinMode(leftMotorPWMPin, OUTPUT);
  pinMode(leftMotorDirPin, OUTPUT);
  pinMode(interruptPin, INPUT);
  attachInterrupt( digitalPinToInterrupt( interruptPin ), getData, RISING );
  Serial.begin(115200);
  while (!Serial) {}
  // initialising communication with the MPU-9250
  IMU.begin();
  if (IMU.begin() < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println( IMU.begin() );
    while (1) {}
  }
  // enables interrupt
  IMU.enableDataReadyInterrupt();
  if (IMU.enableDataReadyInterrupt() < 0) {
    Serial.println("Interrupt initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println( IMU.enableDataReadyInterrupt() );
    while (1) {}
  }
  /*
    IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
    if (IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G) < 0) {
    Serial.println("Accel. Range change unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println( IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G) );
    while (1) {}
    }

    IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
    if (IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS) < 0) {
    Serial.println("Gyro. Range unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println( IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS) );
    while (1) {}
    }
  */
  Serial.print("Starting balance...");
}

void loop() {
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000;

  while (!dataReady)
  {
    Serial.print(curr_angle);
    Serial.print("/");
    Serial.print(motor_power);
    Serial.print( "\n" );
    pid();
    motor_power = constrain( motor_power, -255, 255 );
    setMotors();
  }
  
  dataReady = false;
  Serial.print(curr_angle);
  Serial.print("/");
  Serial.print(motor_power);
  Serial.print( "\n" );

}
