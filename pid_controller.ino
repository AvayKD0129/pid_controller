#include <Wire.h>
#include <Servo.h>

#define MPU 0x68 // MPU6050 I2C address
#define MA_LENGTH 50 // Moving average length
int values[MA_LENGTH], lastValue, cur_index = 0, remove_index, cur_value;
unsigned long up_time = 0, value;

double AccX, AccY, AccZ;
float GyroX;
double accAngleX,totalAngleX;
float elapsedTime, currentTime, previousTime;

// PID Variables
double dt, last_time;
double integral, previous;
int output = 0;
double kp, ki, kd;
double setpoint = 0.00;

Servo myservo;
 
void setup() {
  Serial.begin(19200);
  setup_pid();
  setup_mpu();
  setup_servo();
}

void setup_servo() {
  myservo.attach(3);
  myservo.write(0);

  Serial.println("Test staarted!!!");
  myservo.writeMicroseconds(1000);  // Arming the Morot
}

void setup_mpu() {
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
}

void setup_pid() {
  kp = 2.2;
  ki = 0.0013;
  kd = 0.32;
  last_time = 0;
}

double pid_now = 0, pid_dt;

void loop() {
  pid_now = millis();
  pid_dt = pid_now - last_time;
  if (pid_dt >=5) {
    last_time = pid_now;
    pid_loop(pid_dt);
  }
}

void pid_loop(double pid_dt) {
  dt = (pid_dt)/1000.00;  // In seconds
  float xAngle = calculate_xAngle();
  float error = setpoint - xAngle;
  float pid_output = pid(error);
  int output = 1015 + int(pid_output);
  myservo.write(constrain(output, 1015, 1150));
}

void servo_test() {
  int val = 5;
  myservo.write(val);    
  while(1) {
    if (Serial.available() > 0) {
      val = Serial.parseInt();
      myservo.writeMicroseconds(val);

      Serial.print("Value: ");
      Serial.println(val);
    }
    delay(100);
  }
  Serial.println("Test completed!!!");
}

float calculate_xAngle() {
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers

  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value

  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 2.84;

  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 2, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  totalAngleX = 0.98 * (totalAngleX + GyroX * elapsedTime) + 0.02 * accAngleX;
  return totalAngleX;
}

double pid(double error) {
  double proportional = error;
  integral += error * dt;
  double derivative = (error - previous) / dt;
  previous = error;
  double output = (kp * proportional) + (ki * integral) + (kd * derivative);
  return output;
}
