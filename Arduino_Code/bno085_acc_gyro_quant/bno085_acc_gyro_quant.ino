#include <Wire.h>
#include <Adafruit_BNO08x.h>

#define SDA_PIN 8
#define SCL_PIN 9
#define SEMG_PIN 0   // ADC pin for sEMG

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

// IMU storage
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float qw = 0, qx = 0, qy = 0, qz = 0;
float roll = 0, pitch = 0, yaw = 0;

// Timing
#define IMU_INTERVAL_US 10000    // 100 Hz
//#define SEMG_INTERVAL_US 1250    // 800 Hz
#define SEMG_INTERVAL_US 10000    // 100 Hz

unsigned long lastImuMicros = 0;
unsigned long lastSemgMicros = 0;

// Flag for new IMU data
bool imuUpdated = false;

void setup() {
  Serial.begin(1000000);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!bno08x.begin_I2C(0x4A, &Wire)) {
    Serial.println("BNO08x not detected ... Check wiring!");
    while (1);
  }

  bno08x.enableReport(SH2_ACCELEROMETER, IMU_INTERVAL_US);
  bno08x.enableReport(SH2_RAW_GYROSCOPE, IMU_INTERVAL_US);
  bno08x.enableReport(SH2_ROTATION_VECTOR, IMU_INTERVAL_US);

  // CSV header
  Serial.println("imu_time_us,semg_time_us,roll,pitch,yaw,qw,qx,qy,qz,ax,ay,az,gx,gy,gz,semg_raw,semg_voltage");
}

void loop() {
  unsigned long now = micros();

  // --- Read IMU at 50 Hz ---
  if (now - lastImuMicros >= IMU_INTERVAL_US) {
    lastImuMicros = now;

    if (bno08x.getSensorEvent(&sensorValue)) {
      imuUpdated = true;

      switch (sensorValue.sensorId) {
        case SH2_ACCELEROMETER:
          ax = sensorValue.un.accelerometer.x;
          ay = sensorValue.un.accelerometer.y;
          az = sensorValue.un.accelerometer.z;
          break;


          case SH2_RAW_GYROSCOPE:
          gx = sensorValue.un.rawGyroscope.x;
          gy = sensorValue.un.rawGyroscope.y;
          gz = sensorValue.un.rawGyroscope.z;
          break;


//        case SH2_RAW_GYROSCOPE:
//          gx = sensorValue.un.gyroscope.x;
//          gy = sensorValue.un.gyroscope.y;
//          gz = sensorValue.un.gyroscope.z;
//          break;

        case SH2_ROTATION_VECTOR:
          qw = sensorValue.un.rotationVector.real;
          qx = sensorValue.un.rotationVector.i;
          qy = sensorValue.un.rotationVector.j;
          qz = sensorValue.un.rotationVector.k;

          // Quaternion → Euler
          {
            float ysqr = qy * qy;
            float t0 = +2.0 * (qw * qx + qy * qz);
            float t1 = +1.0 - 2.0 * (qx * qx + ysqr);
            roll = atan2(t0, t1) * 180.0 / PI;

            float t2 = +2.0 * (qw * qy - qz * qx);
            t2 = constrain(t2, -1.0, 1.0);
            pitch = asin(t2) * 180.0 / PI;

            float t3 = +2.0 * (qw * qz + qx * qy);
            float t4 = +1.0 - 2.0 * (ysqr + qz * qz);
            yaw = atan2(t3, t4) * 180.0 / PI;
          }
          break;
      }
    }
  }

  // --- Read sEMG at 1 kHz and publish ---
  if (now - lastSemgMicros >= SEMG_INTERVAL_US) {
    lastSemgMicros = now;

    int semgRaw = analogRead(SEMG_PIN);
    float semgVolt = semgRaw * 3.3 / 4095.0;

    // imu_time_us
    if (imuUpdated) {
      Serial.print(lastImuMicros);
    }
    Serial.write(',');

    // semg_time_us
    Serial.print(lastSemgMicros);
    Serial.write(',');

    if (imuUpdated) {
      // IMU values with reduced precision
      Serial.print(roll, 2); Serial.write(',');
      Serial.print(pitch, 2); Serial.write(',');
      Serial.print(yaw, 2); Serial.write(',');
      Serial.print(qw, 4); Serial.write(',');
      Serial.print(qx, 4); Serial.write(',');
      Serial.print(qy, 4); Serial.write(',');
      Serial.print(qz, 4); Serial.write(',');
      Serial.print(ax, 2); Serial.write(',');
      Serial.print(ay, 2); Serial.write(',');
      Serial.print(az, 2); Serial.write(',');
      Serial.print(gx, 2); Serial.write(',');
      Serial.print(gy, 2); Serial.write(',');
      Serial.print(gz, 2); Serial.write(',');
      imuUpdated = false;
    } else {
      // Just output commas for empty IMU slots
      for (int i = 0; i < 13; i++) Serial.write(',');
    }

    // Always output sEMG
    Serial.print(semgRaw);
    Serial.write(',');
    Serial.println(semgVolt, 3);
  }
}
