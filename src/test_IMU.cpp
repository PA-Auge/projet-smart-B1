#include "ICM_20948.h"

#define AD0_VAL 0
ICM_20948_I2C myICM;

// ─── Paramètres Mahony ────────────────────────────────────────────────────────
#define Kp 2.0f
#define Ki 0.005f

float integralX = 0, integralY = 0, integralZ = 0;
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;

unsigned long lastTime = 0;

// ─── Mahony 9 axes ────────────────────────────────────────────────────────────
void MahonyUpdate(float gx, float gy, float gz,
                  float ax, float ay, float az,
                  float mx, float my, float mz,
                  float dt) {

  // Normalise accél
  float norm = sqrt(ax*ax + ay*ay + az*az);
  if (norm == 0) return;
  ax /= norm; ay /= norm; az /= norm;

  // Normalise magnéto
  norm = sqrt(mx*mx + my*my + mz*mz);
  if (norm == 0) return;
  mx /= norm; my /= norm; mz /= norm;

  // Vecteur référence dans le repère monde → repère capteur
  float hx = 2*(mx*(0.5f - q2*q2 - q3*q3) + my*(q1*q2 - q0*q3) + mz*(q1*q3 + q0*q2));
  float hy = 2*(mx*(q1*q2 + q0*q3) + my*(0.5f - q1*q1 - q3*q3) + mz*(q2*q3 - q0*q1));
  float bx = sqrt(hx*hx + hy*hy);
  float bz = 2*(mx*(q1*q3 - q0*q2) + my*(q2*q3 + q0*q1) + mz*(0.5f - q1*q1 - q2*q2));

  // Vecteur gravité et champ mag estimés
  float vx = 2*(q1*q3 - q0*q2);
  float vy = 2*(q0*q1 + q2*q3);
  float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  float wx = 2*bx*(0.5f - q2*q2 - q3*q3) + 2*bz*(q1*q3 - q0*q2);
  float wy = 2*bx*(q1*q2 - q0*q3)        + 2*bz*(q0*q1 + q2*q3);
  float wz = 2*bx*(q0*q2 + q1*q3)        + 2*bz*(0.5f - q1*q1 - q2*q2);

  // Erreur
  float ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  float ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  float ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

  // Intégrale
  integralX += Ki * ex * dt;
  integralY += Ki * ey * dt;
  integralZ += Ki * ez * dt;

  // Correction gyro
  gx += Kp*ex + integralX;
  gy += Kp*ey + integralY;
  gz += Kp*ez + integralZ;

  // Intégration quaternion
  float dq0 = 0.5f*(-q1*gx - q2*gy - q3*gz)*dt;
  float dq1 = 0.5f*( q0*gx + q2*gz - q3*gy)*dt;
  float dq2 = 0.5f*( q0*gy - q1*gz + q3*gx)*dt;
  float dq3 = 0.5f*( q0*gz + q1*gy - q2*gx)*dt;

  q0 += dq0; q1 += dq1; q2 += dq2; q3 += dq3;

  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  Wire.setClock(400000);

  while (myICM.begin(Wire, AD0_VAL) != ICM_20948_Stat_Ok) {
    Serial.println(F("Tentative..."));
    delay(500);
  }
  Serial.println(F("ICM connecte"));

  lastTime = micros();
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  if (myICM.dataReady()) {
    myICM.getAGMT();

    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    lastTime = now;

    // Accéléromètre en g
    float ax = myICM.accX() / 1000.0f;
    float ay = myICM.accY() / 1000.0f;
    float az = myICM.accZ() / 1000.0f;

    // Gyroscope en rad/s
    float gx = myICM.gyrX() * DEG_TO_RAD;
    float gy = myICM.gyrY() * DEG_TO_RAD;
    float gz = myICM.gyrZ() * DEG_TO_RAD;

    // Magnétomètre en uT
    float mx = myICM.magX();
    float my = myICM.magY();
    float mz = myICM.magZ();

    MahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, dt);

    // Quaternion → Euler
    float roll  = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * RAD_TO_DEG;
    float pitch = asin( 2*(q0*q2 - q3*q1)) * RAD_TO_DEG;
    float yaw   = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)) * RAD_TO_DEG;

    Serial.print(roll,  2); Serial.print(F("\t"));
    Serial.print(pitch, 2); Serial.print(F("\t"));
    Serial.println(yaw, 2);
  }
}