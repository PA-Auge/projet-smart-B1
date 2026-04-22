#include "ICM_20948.h"

// ─── Configuration ────────────────────────────────────────────────────────────
#define AD0_VAL  0       // 0x68 → AD0_VAL = 0
#define INT_PIN  2

ICM_20948_I2C myICM;


// ─── Helpers : calcul des angles ──────────────────────────────────────────────
void quaternionToEuler(float qw, float qx, float qy, float qz,
                       float &roll, float &pitch, float &yaw) {
  float sinr = 2.0f * (qw * qx + qy * qz);
  float cosr = 1.0f - 2.0f * (qx * qx + qy * qy);
  roll = atan2(sinr, cosr) * RAD_TO_DEG;

  float sinp = 2.0f * (qw * qy - qz * qx);
  pitch = (fabs(sinp) >= 1.0f) ? copysign(90.0f, sinp) : asin(sinp) * RAD_TO_DEG;

  float siny = 2.0f * (qw * qz + qx * qy);
  float cosy = 1.0f - 2.0f * (qy * qy + qz * qz);
  yaw = atan2(siny, cosy) * RAD_TO_DEG;
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial);
  

  Wire.begin();
  Wire.setClock(400000);

  // 1. Connexion
  bool initialized = false;
  while (!initialized) {
    myICM.begin(Wire, AD0_VAL);
    if (myICM.status == ICM_20948_Stat_Ok) {
      initialized = true;
    } else {
      delay(500);
    }
  }

  myICM.startupDefault();
  // 2. Firmware DMP
  if (myICM.loadDMPFirmware() != ICM_20948_Stat_Ok) {
    while (1);
  }

  // 3. Capteurs DMP
  if (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) != ICM_20948_Stat_Ok) {
    while (1);
  }
  if (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) != ICM_20948_Stat_Ok) {
    while (1);
  }

  // 4. ODR
  if (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) != ICM_20948_Stat_Ok) {
    while (1);
  }
  if (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) != ICM_20948_Stat_Ok) {
    while (1);
  }

  // 5. FIFO + DMP + reset — ordre impératif
  if (myICM.enableFIFO() != ICM_20948_Stat_Ok) {
    while (1);
  }
  if (myICM.enableDMP() != ICM_20948_Stat_Ok) {
    while (1);
  }
  if (myICM.resetDMP() != ICM_20948_Stat_Ok) {
    while (1);
  }
  if (myICM.resetFIFO() != ICM_20948_Stat_Ok) {
    while (1);
  }
  delay(100);

  Serial.println(F("ok"));
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  icm_20948_DMP_data_t data;
  ICM_20948_Status_e status;

  do {
    status = myICM.readDMPdataFromFIFO(&data);

    if (status == ICM_20948_Stat_Ok || 
        status == ICM_20948_Stat_FIFOMoreDataAvail) {

      if (data.header & DMP_header_bitmap_Quat6) {
        float qx = (float)data.Quat6.Data.Q1 / 1073741824.0f;
        float qy = (float)data.Quat6.Data.Q2 / 1073741824.0f;
        float qz = (float)data.Quat6.Data.Q3 / 1073741824.0f;
        float qw = sqrt(max(0.0f, 1.0f - qx*qx - qy*qy - qz*qz));

        float roll, pitch, yaw;
        quaternionToEuler(qw, qx, qy, qz, roll, pitch, yaw);

        //Serial.print(roll,  2); Serial.print(F(" "));
        //Serial.print(pitch, 2); Serial.print(F(" "));
        //Serial.print(yaw,   2); Serial.print(F(" "));
      }
      /**/
      if (data.header & DMP_header_bitmap_Accel) {
        float ax = (float)data.Raw_Accel.Data.X / 16384.0f;
        float ay = (float)data.Raw_Accel.Data.Y / 16384.0f;
        float az = (float)data.Raw_Accel.Data.Z / 16384.0f;

        Serial.print(ax, 3); Serial.print(F("\t"));
        //Serial.print(ay, 3); Serial.print(F("\t"));
        //Serial.print(az, 3);
        //Serial.println("acc");
      }

      if (data.header & DMP_header_bitmap_Quat6 || 
          data.header & DMP_header_bitmap_Accel) {
        //Serial.println();
      }
    }

  } while (status == ICM_20948_Stat_FIFOMoreDataAvail);
}