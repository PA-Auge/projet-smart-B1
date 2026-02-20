#include <Arduino.h>
#include <Wire.h>
#include <NewPing.h>
#include <ICM20948_WE.h>
#include <SparkFunISL29125.h>




// ---------- classes du projet ----------

class Motor {
    public : 
        int PIN_1;
        int PIN_2;
        int PIN_SPEED;

        Motor(int pin_1, int pin_2, int pin_speed) {
            PIN_1 = pin_1;
            PIN_2 = pin_2;
            PIN_SPEED = pin_speed;
            pinMode(PIN_1, OUTPUT);
            pinMode(PIN_2, OUTPUT);
            pinMode(PIN_SPEED, OUTPUT);
        };

        int update_speed(int speed);
};

int Motor::update_speed(int speed) {
  if (speed < 0) {
    digitalWrite(this->PIN_1, HIGH);
    digitalWrite(this->PIN_2, LOW);
  }
  else {
    digitalWrite(this->PIN_1, LOW);
    digitalWrite(this->PIN_2, HIGH);
  }
  analogWrite(this->PIN_SPEED, abs(speed));
};


// ---------- composants et variables/constantes associées ----------


Motor motor_left = Motor(0, 1, 2);
Motor motor_right = Motor(3, 4, 5);

const int SONAR_MAX_DISTANCE = 100;
NewPing sonar_l = NewPing(6,7, SONAR_MAX_DISTANCE);
NewPing sonar_r = NewPing(8,9, SONAR_MAX_DISTANCE);

#define ICM20948_ADDR 0x68
ICM20948_WE imu = ICM20948_WE(ICM20948_ADDR);

SFE_ISL29125 RGB_sensor;

// ---------- variables et constantes globales ----------

// puissance du moteur
const float SPEED = 128; // entre 0 et 255 inclus

float angle_reference = 0;
float stop_distance = 0;
bool move = true;


// ---------- fonctions ----------
/*
speed : declare la vitesse de base des moteurs
motor_difference : ecart de vitesse des moteurs par rapport a la vitesse de base (s'ajoute a la puissance du moteur gauche et se soustrait a la puissance du moteur droit)
*/ 
void set_speed(int speed,int motors_difference);

/*
negatif : plus proche de la gauche
positif : plus proche de la droite
*/
int get_middle_offset();

/*
0 : non-identifié
1 : rouge
2 : vert
*/
int get_color();

void setup() {
  Wire.begin();
  Serial.begin(115200);
  
  while (!Serial) {}

  // ----- verifications du capteur IMU -----
  if (!imu.init()) {
    Serial.println("L'IMU ne repond pas !");
  }
  else {
    Serial.println("IMU connecté");
  }

  if (!imu.initMagnetometer()) {
    Serial.println("Magnetometre non connecté");
  }
  else {
    Serial.println("Magnetometre connecté");
  }
  
  Serial.println("Calibration de l'IMU, posez a plat et ne plus le bougez plus");
  delay(1000);
  imu.autoOffsets();
  Serial.println("Calibration finie");
  imu.setAccRange(ICM20948_ACC_RANGE_2G);
  imu.setAccDLPF(ICM20948_DLPF_6);
  imu.setAccSampleRateDivider(10);

  // ----- verification capteur RGB -----

  if (!RGB_sensor.init()) {
    Serial.println("Capteur RVB non connecté");
  }
  else {
    Serial.println("Capteur RVB connecté");
  }
}

void loop() {
  if (!move) {
    if ( get_color() == 2 ) {
      move = true;
    }
  }
  else {
    if ( get_color() == 1 ) {
      move = false;
      stop_distance = 0;
    }
  }

  delay(100);
}


void set_speed(int speed, int motors_difference = 0)
{ 
  motor_left.update_speed(speed-motors_difference);
  motor_right.update_speed(speed+motors_difference);
}

int get_middle_offset()
{

}

int get_color()
{
  int r = RGB_sensor.readRed();
  int g = RGB_sensor.readGreen();
  int b = RGB_sensor.readBlue();

  if ( r > 200 && g < 50 && b < 50 ) {
    return 1;
  }
  if ( r < 50 && g > 200 && b < 50 ) {
    return 2;
  }
  return 0;
}