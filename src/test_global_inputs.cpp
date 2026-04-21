#include <Arduino.h>

#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <ICM20948_WE.h>
#include <NewPing.h>

// ----- enums -----
enum class Color 
{
  Green,
  Red,
  Any
};

// ----- RGB variables -----
Adafruit_TCS34725 RGB_sensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
// ----- end RGB variables -----

// ----- IMU variables -----
#define ICM20948_ADDR 0x68
ICM20948_WE imu = ICM20948_WE(ICM20948_ADDR);

long prev_t = 0;
float dt = 1;

float current_speed = 0;
float stop_distance = 0;

float angle_speed = 0;
float angle_offset = 0;
// ----- end IMU variables -----

// ----- sonar variables -----
#define SONAR_MAX_DISTANCE 100

NewPing sonar_l = NewPing(9,10, SONAR_MAX_DISTANCE);
NewPing sonar_r = NewPing(11,12, SONAR_MAX_DISTANCE);
// ----- end sonar variables -----

// retourne la couleur capte par le capteur `(rouge, vert, autre)`
Color get_color();

void update_imu_data();

void setup() 
{
    Serial.begin(115200);
    if (!RGB_sensor.begin()) {
        Serial.println("Capteur RVB non connecté");
        while (true) {}
    }
    else {
        Serial.println("Capteur RVB connecté");
    }

    if (!imu.init()) {
        Serial.println("L'IMU ne repond pas !");
    }
    else {
        Serial.println("IMU connecté");
    }
}

void loop() {
    dt = millis() - prev_t;
    prev_t = millis();

    update_imu_data();
    Serial.print("curent speed    | ");
    Serial.println(current_speed);
    Serial.print("curent distance | ");
    Serial.println(stop_distance);

    Serial.print("angular speed   | ");
    Serial.println(angle_speed);
    Serial.print("angle offset    | ");
    Serial.println(angle_offset);

    Serial.println("----------");
    Serial.print("Color : ");
    switch (get_color())
    {
    case Color::Red:
        Serial.println("Rouge");
        break;
    
    case Color::Green:
        Serial.println("Vert");
        break;

    default:
        Serial.println("Non défini");
        break;
    }
    Serial.println("----------");

    Serial.print("left :  ");
    Serial.println(sonar_l.ping_cm());

    Serial.print("right : ");
    Serial.println(sonar_r.ping_cm());

    Serial.println("//////////");
}


Color get_color()
{
  uint16_t r, g, b, c, colorTemp, lux;
  RGB_sensor.getRawData(&r, &g, &b, &c);

  if ( r > 15000 && g < 5000 && b < 5000 ) {
    return Color::Red;
  }
  if ( r < 5000 && g > 25000 && b < 5000 ) {
    return Color::Green;
  }
  return Color::Any;
}

void update_imu_data() 
{
    xyzFloat acceleration;
    imu.getCorrectedAccRawValues(&acceleration);
    current_speed += acceleration.x * dt/1000;
    stop_distance += current_speed * dt/1000;

    xyzFloat angle;
    imu.getAngles(&angle);

    angle_speed   += angle.z * dt/1000;
    angle_offset  += angle_speed * dt/1000;
}