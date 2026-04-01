#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "Servo.h"

MPU6050 mpu;
Servo servo1, servo2, servo3, servo4, servo5;

// ── DMP ──────────────────────────────────────────────────────────────────────
bool      dmpReady  = false;
uint8_t   devStatus;
uint16_t  packetSize;          
uint16_t  fifoCount;           
uint8_t   fifoBuffer[64];

Quaternion  q;
VectorFloat gravity;
float       ypr[3];            

// ── PID target ───────────────────────────────────────────────────────────────
float targetPitch = 0;
float targetRoll  = 0;
float targetYaw   = 0;

// ── PID state ────────────────────────────────────────────────────────────────
float errorPitch,     errorRoll,     errorYaw;
float prevErrorPitch = 0, prevErrorRoll = 0, prevErrorYaw = 0;
float integralPitch  = 0, integralRoll  = 0, integralYaw  = 0;

// ── PID gains (sesuaikan untuk sistem Anda) ──────────────────────────────────
float kp = 1.0, ki = 0.05, kd = 0.1;   

// ── Integral windup limit ────────────────────────────────────────────────────
const float INTEGRAL_LIMIT = 50.0;      

// ── Timing ───────────────────────────────────────────────────────────────────
float         dt;
unsigned long lastTime;
const float   DT_MIN = 0.001;           

// =============================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);              

  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 tidak terdeteksi! Cek kabel.");
    while (true);
  }

  devStatus = mpu.dmpInitialize();

 
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  // Pasang semua servo
  servo1.attach(3);
  servo2.attach(5);
  servo3.attach(6);
  servo4.attach(9);
  servo5.attach(10);

  // Posisi netral
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
  servo5.write(90);

  if (devStatus == 0) {
    
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();         

    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();  

    dmpReady = true;
    Serial.println("MPU6050 DMP SIAP");
  } else {
    Serial.print("Inisialisasi DMP GAGAL, kode: ");
    Serial.println(devStatus);
    
  }

  lastTime = millis();
}

// =============================================================================
void loop() {
  if (!dmpReady) return;

  
  bool dataReady = false;

#ifdef _MPU6050_6AXIS_MOTIONAPPS20_H_
  
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    dataReady = true;
  }
#else
  
  fifoCount = mpu.getFIFOCount();
  if (fifoCount >= packetSize) {
    
    if (fifoCount == 1024) {
      mpu.resetFIFO();
      Serial.println("FIFO overflow! Reset.");
      return;
    }
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.resetFIFO();   
    dataReady = true;
  }
#endif

  if (!dataReady) return;

  // ── Hitung sudut dari DMP ─────────────────────────────────────────────────
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  float yaw   = ypr[0] * 180.0 / M_PI;
  float pitch = ypr[1] * 180.0 / M_PI;
  float roll  = ypr[2] * 180.0 / M_PI;

  // ── dt ───────────────────────────────────────────────────────────────────
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  lastTime = now;

  
  if (dt < DT_MIN) dt = DT_MIN;

  // ── Hitung error ──────────────────────────────────────────────────────────
  errorPitch = targetPitch - pitch;
  errorRoll  = targetRoll  - roll;
  errorYaw   = targetYaw   - yaw;

  // ── Integral dengan anti-windup ───────────────────────────────────────────
  integralPitch += errorPitch * dt;
  integralRoll  += errorRoll  * dt;
  integralYaw   += errorYaw   * dt;

  
  integralPitch = constrain(integralPitch, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  integralRoll  = constrain(integralRoll,  -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  integralYaw   = constrain(integralYaw,   -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  // ── Derivative ────────────────────────────────────────────────────────────
  float dPitch = (errorPitch - prevErrorPitch) / dt;
  float dRoll  = (errorRoll  - prevErrorRoll)  / dt;
  float dYaw   = (errorYaw   - prevErrorYaw)   / dt;

  // ── Output PID ────────────────────────────────────────────────────────────
  float outPitch = kp * errorPitch + ki * integralPitch + kd * dPitch;
  float outRoll  = kp * errorRoll  + ki * integralRoll  + kd * dRoll;
  float outYaw   = kp * errorYaw   + ki * integralYaw   + kd * dYaw;

  // ── Simpan error sebelumnya ───────────────────────────────────────────────
  prevErrorPitch = errorPitch;
  prevErrorRoll  = errorRoll;
  prevErrorYaw   = errorYaw;

  // ── Hitung posisi servo ───────────────────────────────────────────────────
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

  servo1.write((int)s1);
  servo2.write((int)s2);
  servo3.write((int)s3);
  servo4.write((int)s4);
  servo5.write((int)s5);

  
  Serial.print("Pitch: "); Serial.print(pitch, 1);
  Serial.print("  Roll: ");  Serial.print(roll,  1);
  Serial.print("  Yaw: ");   Serial.print(yaw,   1);
  Serial.print("  | S1:"); Serial.print((int)s1);
  Serial.print(" S2:"); Serial.print((int)s2);
  Serial.print(" S3:"); Serial.print((int)s3);
  Serial.print(" S4:"); Serial.print((int)s4);
  Serial.print(" S5:"); Serial.println((int)s5);
}
