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

        void update_speed(int);
};

void Motor::update_speed(int speed) {
  if (speed < 0) {
    digitalWrite(this->PIN_1, HIGH);
    digitalWrite(this->PIN_2, LOW);
  }
  else {
    digitalWrite(this->PIN_1, LOW);
    digitalWrite(this->PIN_2, HIGH);
  }
  analogWrite(this->PIN_SPEED, abs(speed));
}


// ---------- composants et variables/constantes associées ----------


Motor motor_left = Motor(0, 1, 2);
Motor motor_right = Motor(3, 4, 5);

const int SONAR_MAX_DISTANCE = 100;
NewPing sonar_l = NewPing(6,7, SONAR_MAX_DISTANCE);
NewPing sonar_r = NewPing(8,9, SONAR_MAX_DISTANCE);

#define ICM20948_ADDR 0x68
ICM20948_WE imu = ICM20948_WE(ICM20948_ADDR);

SFE_ISL29125 RGB_sensor;

// ---------- enumerateurs ----------

enum class Color 
{
  Green,
  Red,
  Any
};

enum class Direction 
{
  Stop,
  Forward,
  Backward
};

enum class Orientation
{
  Neutral,
  Left,
  Right
};

enum class Intensity 
{
  Low,
  Medium,
  High
};

// ---------- structs ----------

struct BoatMovement
{
  struct DirectionParameters
  {
    Direction direction = Direction::Stop;
    Intensity intensity = Intensity::Medium;
  } direction;
  struct OrientationParameters
  {
    Orientation rotation = Orientation::Neutral;
    Intensity intensity = Intensity::Medium;
  } rotation;
} movements;



// ---------- variables et constantes globales ----------

// puissance du moteur
const float SPEED = 128; // entre 0 et 255 inclus
const float ALLOWED_ANGLE_OFFSET = 10;
const float CRITICAL_ANGLE_OFFSET = 30;
const float ALLOWED_MIDDLE_OFFSET = 10;
const float CRITICAL_MIDDLE_OFFSET = 20;
const float STOP_OFFSET = 5;
const float SPEED_REF = 1;

float angle_reference = 0;
float angle_offset = 0;
float current_speed = 0;
float stop_distance = 0;
bool move = false;


// ---------- fonctions ----------
/*
speed : declare la vitesse de base des moteurs
motor_difference : ecart de vitesse des moteurs par rapport a la vitesse de base (s'ajoute a la puissance du moteur gauche et se soustrait a la puissance du moteur droit)
*/ 
void set_speed(int speed,int motors_difference = 0);

/*
negatif : plus proche de la gauche
positif : plus proche de la droite
*/
float get_middle_offset();

/*
met a jour les données liées au capteur IMU
*/
void update_imu_data();

/*
donne la couleur captée par le capteur
*/
Color get_color();

void update_movements();

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
  
  Serial.println("Calibration de l'IMU, posez a plat et ne le bougez plus");
  delay(1000);
  imu.autoOffsets();
  Serial.println("Calibration finie");
  imu.setAccRange(ICM20948_ACC_RANGE_2G);
  imu.setAccDLPF(ICM20948_DLPF_6);
  imu.setAccSampleRateDivider(10);

  // ----- verification capteur RVB -----

  if (!RGB_sensor.init()) {
    Serial.println("Capteur RVB non connecté");
  }
  else {
    Serial.println("Capteur RVB connecté");
  }
}

void loop() {
  update_imu_data();
  update_movements();

  int speed = 0;
  int motot_diff = 0;

  switch (movements.direction.direction) 
  {
    case Direction::Stop:
      speed = 0;
      break;
    
    case Direction::Forward:
      switch (movements.direction.intensity)
      {
        case Intensity::Low:
          speed = SPEED/3;
          break;
        
        case Intensity::Medium:
          speed = SPEED/2;
          break;

        case Intensity::High:
          speed = SPEED;
          break;
      }

    case Direction::Backward:
      switch (movements.direction.intensity)
      {
        case Intensity::Low:
          speed = -SPEED/3;
          break;
        
        case Intensity::Medium:
          speed = -SPEED/2;
          break;

        case Intensity::High:
          speed = -SPEED;
          break;
      }
  }

  switch (movements.rotation.rotation)
  {
    case Orientation::Neutral:
      break;
    
    case Orientation::Left:
      switch (movements.rotation.intensity)
      {
        case Intensity::Low:
          motot_diff = -speed/3;
          break;
        
        case Intensity::Medium:
          motot_diff = -speed/2;
          break;

        case Intensity::High:
          motot_diff = -SPEED;
          speed = 0;
          break;
      }

      case Orientation::Right:
      switch (movements.rotation.intensity)
      {
        case Intensity::Low:
          motot_diff = speed/3;
          break;
        
        case Intensity::Medium:
          motot_diff = speed/2;
          break;

        case Intensity::High:
          motot_diff = SPEED;
          speed = 0;
          break;
      } 
      break;
  }

  set_speed(speed, motot_diff);
  
  delay(100);
}

void update_movements()
{
  int offset = get_middle_offset();
  int rotation_intensity = 0;
  if (offset < -ALLOWED_MIDDLE_OFFSET) {
    rotation_intensity ++;
  }
  else if (offset > ALLOWED_MIDDLE_OFFSET) {
    rotation_intensity --;
  }
  if (offset < -CRITICAL_MIDDLE_OFFSET) {
    rotation_intensity ++;
  }
  else if (offset > CRITICAL_MIDDLE_OFFSET) {
    rotation_intensity --;
  }

  if (angle_offset < -ALLOWED_ANGLE_OFFSET) {
    rotation_intensity ++;
  }
  else if (angle_offset > ALLOWED_ANGLE_OFFSET) {
    rotation_intensity --;
  }
  if (angle_offset < -CRITICAL_ANGLE_OFFSET) {
    rotation_intensity ++;
  }
  else if (angle_offset > CRITICAL_ANGLE_OFFSET) {
    rotation_intensity --;
  }

  int direction_intensity = 0;
  if (!move) {
    
    float d = abs(stop_distance);
    if (d > STOP_OFFSET) {
      direction_intensity++;
    }
    if (d > 2*STOP_OFFSET) {
      direction_intensity++;
    }
    if (d > 3*STOP_OFFSET) {
      direction_intensity++;
    }
    if (current_speed > SPEED_REF) {
      direction_intensity--;
    }
    if (current_speed > 2*SPEED_REF) {
      direction_intensity*=-1;
    }

    if (stop_distance < d) {
      direction_intensity = 0;
    }
    else if (stop_distance > STOP_OFFSET) {
      direction_intensity*=-1;
    }
  }
  else {
    direction_intensity = 2;
    rotation_intensity = 0;
  }

  if (direction_intensity > 0) {
    movements.direction.direction = Direction::Forward;
  }
  else if (direction_intensity < 0) {
    movements.direction.direction = Direction::Backward;
    direction_intensity*=-1;
  }
  else {
    movements.direction.direction = Direction::Stop;
  }

  if (direction_intensity == 1) {
    movements.direction.intensity = Intensity::Low;
  }
  if (direction_intensity == 2) {
    movements.direction.intensity = Intensity::Medium;
  }
  if (direction_intensity >= 3) {
    movements.direction.intensity = Intensity::High;
  }

  if (rotation_intensity > 0) {
    movements.rotation.rotation = Orientation::Right;
  }
  else if (rotation_intensity < 0) {
    movements.rotation.rotation = Orientation::Left;
    rotation_intensity*=-1;
  }
  else {
    movements.rotation.rotation = Orientation::Neutral;
  }

  if (rotation_intensity == 1) {
    movements.rotation.intensity = Intensity::Low;
  }
  if (rotation_intensity == 2) {
    movements.rotation.intensity = Intensity::Medium;
  }
  if (rotation_intensity >= 3) {
    movements.rotation.intensity = Intensity::High;
  }
}


void set_speed(int speed, int motors_difference = 0)
{ 
  motor_left.update_speed(speed-motors_difference);
  motor_right.update_speed(speed+motors_difference);
}

float get_middle_offset()
{
  return sonar_l.ping_cm() - sonar_r.ping_cm();
}

void update_imu_data() {
  xyzFloat acceleration;
  imu.getCorrectedAccRawValues(&acceleration);
  current_speed += acceleration.x * 0.1;
  stop_distance += current_speed * 0.1;

  xyzFloat angle;
  imu.getAngles(&angle);
  angle_offset += angle.z;
}

Color get_color()
{
  int r = RGB_sensor.readRed();
  int g = RGB_sensor.readGreen();
  int b = RGB_sensor.readBlue();

  if ( r > 15000 && g < 5000 && b < 5000 ) {
    return Color::Red;
  }
  if ( r < 5000 && g > 25000 && b < 5000 ) {
    return Color::Green;
  }
  return Color::Any;
}