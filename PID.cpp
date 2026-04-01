#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "Servo.h"

MPU6050 mpu;
Servo servo1, servo2, servo3, servo4, servo5;

bool dmpReady = false;
uint8_t devStatus;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

float targetPitch = 0;
float targetRoll = 0;
float targetYaw = 0;

float errorPitch, errorRoll, errorYaw;
float prevErrorPitch = 0, prevErrorRoll = 0, prevErrorYaw = 0;
float integralPitch = 0, integralRoll = 0, integralYaw = 0;

float kp = 1.0, ki = 1.0, kd = 1.0;

float dt;
unsigned long lastTime;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  servo1.attach(3);
  servo2.attach(5);
  servo3.attach(6);
  servo4.attach(9);
  servo5.attach(10);

  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
  servo5.write(90);

  if (devStatus == 0) {
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    dmpReady = true;
    Serial.println("MPU6050 READY");
  } else {
    Serial.println("MPU6050 ERROR");
  }

  lastTime = millis();
}

void loop() {

  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float yaw   = ypr[0] * 180 / M_PI;
    float pitch = ypr[1] * 180 / M_PI;
    float roll  = ypr[2] * 180 / M_PI;

    unsigned long now = millis();
    dt = (now - lastTime) / 1000.0;
    lastTime = now;

    errorPitch = targetPitch - pitch;
    errorRoll  = targetRoll  - roll;
    errorYaw   = targetYaw   - yaw;

    integralPitch += errorPitch * dt;
    integralRoll  += errorRoll * dt;
    integralYaw   += errorYaw * dt;

    float dPitch = (errorPitch - prevErrorPitch) / dt;
    float dRoll  = (errorRoll  - prevErrorRoll)  / dt;
    float dYaw   = (errorYaw   - prevErrorYaw)   / dt;

    float outPitch = kp * errorPitch + ki * integralPitch + kd * dPitch;
    float outRoll  = kp * errorRoll  + ki * integralRoll  + kd * dRoll;
    float outYaw   = kp * errorYaw   + ki * integralYaw   + kd * dYaw;

    prevErrorPitch = errorPitch;
    prevErrorRoll  = errorRoll;
    prevErrorYaw   = errorYaw;

    float s1 = 90 + outPitch + outRoll;
    float s2 = 90 + outPitch - outRoll;
    float s3 = 90 + outYaw;
    float s4 = 90 + outPitch;
    float s5 = 90 + outRoll;

    s1 = constrain(s1, 45, 135);
    s2 = constrain(s2, 45, 135);
    s3 = constrain(s3, 45, 135);
    s4 = constrain(s4, 45, 135);
    s5 = constrain(s5, 45, 135);

    servo1.write(s1);
    servo2.write(s2);
    servo3.write(s3);
    servo4.write(s4);
    servo5.write(s5);

    Serial.print("Pitch (°): "); Serial.print(pitch);
    Serial.print(" Roll (°): "); Serial.print(roll);
    Serial.print(" Yaw (°): "); Serial.print(yaw);

    Serial.print(" S1: "); Serial.print(s1);
    Serial.print(" S2: "); Serial.print(s2);
    Serial.print(" S3: "); Serial.print(s3);
    Serial.print(" S4: "); Serial.print(s4);
    Serial.print(" S5: "); Serial.println(s5);
  }
}
