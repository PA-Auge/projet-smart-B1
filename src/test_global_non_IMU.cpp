#include "Adafruit_TCS34725.h"
#include <NewPing.h>

class Motor {
    public : 
        int PIN_1, PIN_2, PIN_SPEED;

        Motor(int p1, int p2, int ps) : PIN_1(p1), PIN_2(p2), PIN_SPEED(ps) {}

        void init() {
            pinMode(PIN_1, OUTPUT);
            pinMode(PIN_2, OUTPUT);
            pinMode(PIN_SPEED, OUTPUT);
        }

        void update_speed(int speed) {
            speed = constrain(speed, -255, 255); // Sécurité
            if (speed < 0) {
                digitalWrite(PIN_1, HIGH);
                digitalWrite(PIN_2, LOW);
            } else if (speed > 0) {
                digitalWrite(PIN_1, LOW);
                digitalWrite(PIN_2, HIGH);
            } else {
                digitalWrite(PIN_1, LOW);
                digitalWrite(PIN_2, LOW);
            }

            analogWrite(PIN_SPEED, abs(speed));
        }
};


enum class Color 
{
  Green,
  Red,
  Any
};

// retourne la couleur capte par le capteur `(rouge, vert, autre)`
Color get_color();
void move_forward();
void move_backward();
void stop();

Adafruit_TCS34725 RGB_sensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

#define SONAR_MAX_DISTANCE 100

NewPing sonar_l = NewPing(9,10, SONAR_MAX_DISTANCE);
NewPing sonar_r = NewPing(12,11, SONAR_MAX_DISTANCE);

Motor motor_l = Motor(2, 4, 3);
Motor motor_r = Motor(7, 8, 5);

const int SPEED = 128;

bool move = true;
// true : forward | flase : backward
bool last_direction = true;

int offset;
Color color;

Color prev_color = Color::Any;

void setup() 
{
    if (!RGB_sensor.begin()) {
        while(1);
    }
    motor_l.init();
    motor_r.init();
}

void loop() 
{
    offset = sonar_l.ping_cm() - sonar_r.ping_cm();
    color = get_color();
    switch (color)
    {
    case Color::Red:
        move = false;
        break;
    
    case Color::Green:
        move = true;
        break;
    }

    if (move) {
        move_forward();
        last_direction = true;
    }
    else {
        if (color == Color::Red) {
            stop();
            return;
        }
        if (color == Color::Any && prev_color == Color::Red) {
            if (last_direction) {
                move_backward();
                last_direction = !last_direction;
            }
            else {
                move_forward();
                last_direction = !last_direction;
            }
        }
    }
    prev_color = color;
    delay(100);
}


Color get_color()
{
  uint16_t r, g, b, c, colorTemp, lux;
  RGB_sensor.getRawData(&r, &g, &b, &c);

  if ( r > g*10/3 && r > b*10/3) {
    return Color::Red;
  }
  if ( g > r*8/3 && g > b*8/5) {
    return Color::Green;
  }
  return Color::Any;
}

void move_forward() {
    if ( offset > 3 ) {
        motor_l.update_speed(SPEED+50);
        motor_r.update_speed(SPEED-50);
    } else if ( offset < -3 ) {
        motor_l.update_speed(SPEED-50);
        motor_r.update_speed(SPEED+50);
    } else {
        motor_l.update_speed(SPEED);
        motor_r.update_speed(SPEED);
    }
}

void move_backward() {
    if ( offset > 3 ) {
        motor_l.update_speed(-SPEED-50);
        motor_r.update_speed(-SPEED+50);
    } else if ( offset < -3 ) {
        motor_l.update_speed(-SPEED+50);
        motor_r.update_speed(-SPEED-50);
    } else {
        motor_l.update_speed(-SPEED);
        motor_r.update_speed(-SPEED);
    }
}

void stop() {
    Serial.println("stop");
    motor_l.update_speed(0);
    motor_r.update_speed(0);
}
