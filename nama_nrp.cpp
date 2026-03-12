#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "ESP32Servo.h"

MPU6050 mpu;
Servo servo;

bool dmpReady = false;
uint8_t devStatus;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

const uint8_t servoPin = 13;

typedef struct struct_msg {
  float yaw;
  float pitch;
  float roll;
} struct_msg;

struct_msg angle;

float targetPitch = 0;
float error = 0, prevError = 0, output = 0;
float integral = 0, derivative = 0;
unsigned long lastTime = 0;
float dt = 0;

const float kp = 0.0;
const float ki = 0.0;
const float kd = 0.0;


void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  servo.attach(servoPin, 500, 2400);
  servo.write(90);

  if (devStatus == 0) {
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);

    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    dmpReady = true;
    Serial.println("MPU6050 IS READY");
  }
  else {
    Serial.println("MPU6050 GOT WRONG");
  }

  lastTime = millis();
}


void loop() {
  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }

  angle.yaw   = ypr[0] * 180 / M_PI;
  angle.roll  = ypr[1] * 180 / M_PI;
  angle.pitch = ypr[2] * 180 / M_PI;

  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  lastTime = now;

  error = targetPitch - angle.pitch;
  integral += error * dt;
  derivative = (error - prevError) / dt;

  output = (kp * error) + (ki * integral) + (kd * derivative);
  prevError = error;

  float servoAngle = constrain(90.0 + output, 45, 135);
  servo.write(servoAngle);

  Serial.print("Yaw (°): ");    Serial.println(angle.yaw);
  Serial.print("Roll (°): ");   Serial.println(angle.roll);
  Serial.print("Pitch (°): ");  Serial.println(angle.pitch);
  Serial.print("Servo (°): ");  Serial.println(servoAngle);
  Serial.print("PID Output: "); Serial.println(output);
  Serial.print("Error: ");      Serial.println(error);

  Serial.print(targetPitch);  Serial.print("\t");
  Serial.print(angle.yaw);    Serial.print("\t");
  Serial.print(angle.roll);   Serial.print("\t");
  Serial.print(angle.pitch);  Serial.print("\t");
  Serial.print(output);       Serial.print("\t");
  Serial.println(servoAngle);
}
