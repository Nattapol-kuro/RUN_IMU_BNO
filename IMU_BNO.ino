
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <math.h>

#define BNO08X_RESET -1

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// =====================================
// SERVO
// =====================================

Servo myservo1;
Servo myservo2;
Servo myservo3;

// =====================================
// GYRO SETTINGS
// =====================================

const int YAW_DIR = -1;

const float KP_STRAIGHT = 3.0;
const float KP_TURN = 3.8;

const int TURN_MIN = 15;
const int TURN_MAX = 250;

const float TOLERANCE = 0.4;

const float KP_SPIN = 2.0;
const int SPIN_MIN = 22;
const int SPIN_MAX = 160;
const float SPIN_TOLERANCE = 1.0;

// Soft Start
const unsigned long START_RAMP_MS = 150;

float lastYaw = 0;
float startYaw = 0;

// =====================================
// MOTOR PIN
// =====================================

#define PwmA 2
#define Ain1 22
#define Ain2 23

#define PwmC 5
#define Cin1 27
#define Cin2 26

// =====================================
// IMU SETTINGS
// =====================================

void setReports() {

  if (!bno08x.enableReport(
        SH2_GAME_ROTATION_VECTOR, 10000)) {

    Serial.println("Error: Could not enable game vector");
  }
}

// =====================================
// READ YAW
// =====================================

float getYaw() {

  if (bno08x.wasReset()) {
    setReports();
  }

  if (bno08x.getSensorEvent(&sensorValue)) {

    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {

      float qr = sensorValue.un.gameRotationVector.real;
      float qi = sensorValue.un.gameRotationVector.i;
      float qj = sensorValue.un.gameRotationVector.j;
      float qk = sensorValue.un.gameRotationVector.k;

      float siny_cosp = 2.0 * (qr * qk + qi * qj);

      float cosy_cosp =
        1.0 - 2.0 * (qj * qj + qk * qk);

      float yaw = atan2(siny_cosp, cosy_cosp)
                  * 180.0 / PI;

      if (yaw < 0) yaw += 360.0;

      lastYaw = yaw;

      return yaw;
    }
  }

  return -1.0;
}

// =====================================
// UPDATE YAW
// =====================================

float updateYaw() {

  float y = getYaw();

  if (y >= 0) {
    lastYaw = y;
  }

  return lastYaw;
}

// =====================================
// INITIALIZE IMU
// =====================================

void getIMU() {

  float y = -1.0;

  for (int i = 0; i < 7; i++) {

    float v = getYaw();

    if (v >= 0) y = v;

    delay(5);
  }

  while (y < 0) {

    y = getYaw();

    delay(5);
  }

  startYaw = y;
  lastYaw = y;
}

// =====================================
// MOTOR CONTROL
// =====================================

void MC(int speedL, int speedR) {

  speedL = constrain(speedL, -255, 255);
  speedR = constrain(speedR, -255, 255);

  if (speedL >= 0) {

    digitalWrite(Ain1, HIGH);
    digitalWrite(Ain2, LOW);

    analogWrite(PwmA, speedL);

  } else {

    digitalWrite(Ain1, LOW);
    digitalWrite(Ain2, HIGH);

    analogWrite(PwmA, -speedL);
  }

  if (speedR >= 0) {

    digitalWrite(Cin1, LOW);
    digitalWrite(Cin2, HIGH);

    analogWrite(PwmC, speedR);

  } else {

    digitalWrite(Cin1, HIGH);
    digitalWrite(Cin2, LOW);

    analogWrite(PwmC, -speedR);
  }
}

// =====================================
// MOTOR MIX
// รักษาสัดส่วนความเร็วมอเตอร์
// =====================================

void MCmix(float L, float R) {

  float maxSpeed = max(fabs(L), fabs(R));

  if (maxSpeed > 255.0) {

    float scale = 255.0 / maxSpeed;

    L *= scale;
    R *= scale;
  }

  MC((int)L, (int)R);
}

// =====================================
// STOP
// =====================================

void STOP() {

  digitalWrite(Ain1, HIGH);
  digitalWrite(Ain2, HIGH);
  analogWrite(PwmA, 255);

  digitalWrite(Cin1, HIGH);
  digitalWrite(Cin2, HIGH);
  analogWrite(PwmC, 255);
}

void yt(int s) {

  STOP();
  delay(s);
}

void mct(int L, int R, int time) {

  unsigned long looptime = millis();

  while (millis() - looptime <= time) {

    MC(L, R);
  }
}

// =====================================
// ANGLE ERROR
// =====================================

float getAngleError(float target, float current) {

  float error = target - current;

  if (error > 180.0) error -= 360.0;
  if (error < -180.0) error += 360.0;

  return error;
}

// =====================================
// GYRO DRIVE + SOFT START
// =====================================

void gyroDrive(int speed, unsigned long ms) {

  float target = startYaw;

  unsigned long start = millis();

  while (millis() - start < ms) {

    // อ่าน IMU ตลอดระหว่างวิ่ง
    float y = updateYaw();

    // คำนวณเวลาเร่ง
    unsigned long elapsed = millis() - start;

    float ramp = (float)elapsed / START_RAMP_MS;

    ramp = constrain(ramp, 0.0, 1.0);

    // ค่อย ๆ เพิ่มความเร็ว
    float baseSpeed = speed * ramp;

    // Gyro Correction
    float err = getAngleError(target, y) * YAW_DIR;

    float corr = err * KP_STRAIGHT * ramp;

    // ส่งความเร็วทั้งสองข้างพร้อมกัน
    MCmix(
      baseSpeed + corr,
      baseSpeed - corr
    );

    delay(5);
  }

  // ไม่ STOP
}

// =====================================
// GYRO PIVOT
// =====================================

void gyroPivot(float angle, bool goForward) {

  float delta = goForward ? -angle : angle;

  bool useRight = angle > 0;

  float prevYaw = lastYaw;

  float rotated = 0;

  unsigned long start = millis();

  int stable = 0;

  while (stable < 5 && millis() - start < 8000) {

    float y = getYaw();

    // ประมวลผลเฉพาะเมื่อได้ค่าใหม่
    if (y < 0) {
      delay(5);
      continue;
    }

    rotated += getAngleError(y, prevYaw);

    prevYaw = y;
    lastYaw = y;

    float err = delta - rotated * YAW_DIR;

    if (fabs(err) <= TOLERANCE) {

      stable++;

      delay(5);

      continue;
    }

    stable = 0;

    int u = (int)(fabs(err) * KP_TURN);

    u = constrain(u, TURN_MIN, TURN_MAX);

    if (err < 0) u = -u;

    if (useRight) {

      MC(0, -u);

    } else {

      MC(u, 0);
    }

    delay(5);
  }

  // ตั้ง Heading ใหม่หลังเลี้ยว
  startYaw = lastYaw;
}

// =====================================
// GYRO SPIN
// =====================================

void gyroSpin(float angle) {

  float delta = -angle;

  float prevYaw = lastYaw;

  float rotated = 0;

  unsigned long start = millis();

  int stable = 0;

  while (stable < 5 && millis() - start < 8000) {

    float y = getYaw();

    if (y < 0) {
      delay(5);
      continue;
    }

    rotated += getAngleError(y, prevYaw);

    prevYaw = y;
    lastYaw = y;

    float err = delta - rotated * YAW_DIR;

    if (fabs(err) <= SPIN_TOLERANCE) {

      stable++;

      delay(5);

      continue;
    }

    stable = 0;

    int u = (int)(fabs(err) * KP_SPIN);

    u = constrain(u, SPIN_MIN, SPIN_MAX);

    if (err < 0) u = -u;

    MC(u, -u);

    delay(5);
  }

  startYaw = lastYaw;
}

// =====================================
// COMMAND
// =====================================

void GF(int speed, unsigned long ms) {

  gyroDrive(abs(speed), ms);
}

void GB(int speed, unsigned long ms) {

  gyroDrive(-abs(speed), ms);
}

void GTF(float angle) {

  gyroPivot(angle, true);
}

void GTB(float angle) {

  gyroPivot(angle, false);
}

// =====================================
// SETUP
// =====================================

void setup() {

  Serial.begin(9600);

  pinMode(PwmA, OUTPUT);
  pinMode(Ain1, OUTPUT);
  pinMode(Ain2, OUTPUT);

  pinMode(PwmC, OUTPUT);
  pinMode(Cin1, OUTPUT);
  pinMode(Cin2, OUTPUT);

  myservo1.attach(9);
  myservo2.attach(10);
  myservo3.attach(30);

  myservo1.write(110);
  myservo2.write(62);
  myservo3.write(130);

  Wire.begin();

  if (!bno08x.begin_I2C()) {

    Serial.println("BNO08x not found");

    while (1) delay(10);
  }

  setReports();

  tone(4, 1200, 100);

  // อ่าน Heading เริ่มต้นครั้งเดียว
  getIMU();
}

// =====================================
// BLUETOOTH / START
// =====================================

void bt() {

  while (digitalRead(7) == 0) {

    STOP();
  }

  delay(300);
}

// =====================================
// LOOP
// =====================================

void loop() {

  bt();

  GF(255, 500);

  GTF(90);

  GF(255, 500);

  GTF(90);

  GF(255, 500);

  GTF(90);

  GF(255, 500);

  GTF(90);

  STOP();
}
