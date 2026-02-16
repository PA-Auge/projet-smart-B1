#include <Arduino.h>
#include <NewPing.h>


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
1 : vert
2 : rouge
*/
int get_color();

// met a jour le capteur IMU
void update_imu();

void setup() {
  Serial.begin(115200);
}

void loop() {
  if (!move) {
    if (get_color() == 1){
      move = true;
    }
  }
  else {
    if (get_color() == 2) {
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
  
}

void update_imu()
{
  
}