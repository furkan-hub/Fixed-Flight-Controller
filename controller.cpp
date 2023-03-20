#include <Wire.h>
#include <PID_v1.h>
#include <MPU6050.h>

// Define the MPU6050 address
const int MPU_addr = 0x68;

// Define the roll PID constants
double Kp = 1;
double Ki = 0.1;
double Kd = 0.1;

// Define the PID setpoint and input variables
double setPoint = 0;
double input, output;

// Initialize the PID controller
PID rollPID(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);

// Initialize the MPU6050 sensor
MPU6050 mpu;

void setup() {
  // Initialize the serial communication
  Serial.begin(9600);
  
  // Initialize the MPU6050 sensor
  Wire.begin();
  mpu.initialize();
  
  // Set the roll PID tuning parameters
  rollPID.SetMode(AUTOMATIC);
  rollPID.SetSampleTime(10);
  rollPID.SetOutputLimits(-255, 255);
}

void loop() {
  // Read the MPU6050 sensor data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Calculate the roll angle from the MPU6050 data
  float roll = atan2(-ay, -az) * (180.0 / PI);
  
  // Compute the PID output
  input = roll;
  rollPID.Compute();

  // Send the PID output to the motor controller
  // Use the "output" variable to control the motor
  Serial.println(output);
  
  // Delay to allow the motor to respond
  delay(10);
}
