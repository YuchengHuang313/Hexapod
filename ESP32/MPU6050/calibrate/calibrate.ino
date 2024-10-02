#include <Wire.h>
#include <MPU6050.h>
#include <Kalman.h>  // Include the Kalman library

#define MPU6050_ADDRESS_1 0x68 // AD0 = 0
#define MPU6050_ADDRESS_2 0x69 // AD0 = 1

MPU6050 mpu1(MPU6050_ADDRESS_1);
MPU6050 mpu2(MPU6050_ADDRESS_2);

Kalman kalmanX1, kalmanY1, kalmanZ1;
Kalman kalmanX2, kalmanY2, kalmanZ2;

void setup() {
  Serial.begin(115200);
  Wire.begin(11, 12);

  // Initialize and test connection for MPU6050 sensor 1
  mpu1.initialize();
  if (!mpu1.testConnection()) {
    Serial.println("MPU6050 (0x68) connection failed");
    while (1);
  } else {
    Serial.println("MPU6050 (0x68) connected");
  }

  // Initialize and test connection for MPU6050 sensor 2
  mpu2.initialize();
  if (!mpu2.testConnection()) {
    Serial.println("MPU6050 (0x69) connection failed");
    while (1);
  } else {
    Serial.println("MPU6050 (0x69) connected");
  }

  Serial.println("Calibrating MPU6050 (0x68)...");
  mpu1.CalibrateAccel();
  mpu1.CalibrateGyro();
  Serial.println("\nCalibration complete for MPU6050 (0x68).");

  Serial.println("Calibrating MPU6050 (0x69)...");
  mpu2.CalibrateAccel();
  mpu2.CalibrateGyro();
  Serial.println("\nCalibration complete for MPU6050 (0x69).");
}

void loop() {
  int16_t ax1, ay1, az1, gx1, gy1, gz1;
  int16_t ax2, ay2, az2, gx2, gy2, gz2;

  // Read data from MPU6050 sensor 1
  mpu1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
  float ax1_corrected = ax1 / 2.0 * mpu1.get_acce_resolution();
  float ay1_corrected = ay1 / 2.0 * mpu1.get_acce_resolution();
  float az1_corrected = az1 / 2.0 * mpu1.get_acce_resolution();
  float gx1_corrected = gx1 / 2.0 * mpu1.get_gyro_resolution();
  float gy1_corrected = gy1 / 2.0 * mpu1.get_gyro_resolution();
  float gz1_corrected = gz1 / 2.0 * mpu1.get_gyro_resolution();

  // Apply Kalman filter to sensor 1 data
  float ax1_filtered = kalmanX1.getAngle(ax1_corrected, gx1_corrected, 0.01);
  float ay1_filtered = kalmanY1.getAngle(ay1_corrected, gy1_corrected, 0.01);
  float az1_filtered = kalmanZ1.getAngle(az1_corrected, gz1_corrected, 0.01);

  // Read data from MPU6050 sensor 2
  mpu2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);
  float ax2_corrected = ax2 / 2.0 * mpu2.get_acce_resolution();
  float ay2_corrected = ay2 / 2.0 * mpu2.get_acce_resolution();
  float az2_corrected = az2 / 2.0 * mpu2.get_acce_resolution();
  float gx2_corrected = gx2 / 2.0 * mpu2.get_gyro_resolution();
  float gy2_corrected = gy2 / 2.0 * mpu2.get_gyro_resolution();
  float gz2_corrected = gz2 / 2.0 * mpu2.get_gyro_resolution();

  // Apply Kalman filter to sensor 2 data
  float ax2_filtered = kalmanX2.getAngle(ax2_corrected, gx2_corrected, 0.01);
  float ay2_filtered = kalmanY2.getAngle(ay2_corrected, gy2_corrected, 0.01);
  float az2_filtered = kalmanZ2.getAngle(az2_corrected, gz2_corrected, 0.01);

  // Averaging filtered data from both sensors
  float ax_avg = (ax1_filtered + ax2_filtered) / 2.0;
  float ay_avg = (ay1_filtered + ay2_filtered) / 2.0;
  float az_avg = (az1_filtered + az2_filtered) / 2.0;
  float gx_avg = (gx1_corrected + gx2_corrected) / 2.0;
  float gy_avg = (gy1_corrected + gy2_corrected) / 2.0;
  float gz_avg = (gz1_corrected + gz2_corrected) / 2.0;

  // Display averaged filtered data
  Serial.print("Accel: ");
  Serial.print(ax_avg, 1);
  Serial.print(" ");
  Serial.print(ay_avg, 1);
  Serial.print(" ");
  Serial.print(az_avg, 1);
  Serial.print(" | Gyro: ");
  Serial.print(gx_avg, 1);
  Serial.print(" ");
  Serial.print(gy_avg, 1);
  Serial.print(" ");
  Serial.print(gz_avg, 1);
  Serial.println();

  delay(100);
}
