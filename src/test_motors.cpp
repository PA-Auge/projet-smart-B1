#include <Arduino.h>

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

Motor motor_l = Motor(2, 4, 3);
Motor motor_r = Motor(7, 8, 5);


void setup()
{
    motor_l.init();
    motor_r.init();
}

void loop()
{   
    motor_l.update_speed(255);
    motor_r.update_speed(0);
    delay(1000);

    motor_l.update_speed(-255);
    motor_r.update_speed(255);
    delay(1000);

    motor_l.update_speed(100);
    motor_r.update_speed(100);
    delay(1000);

    motor_l.update_speed(-100);
    motor_r.update_speed(-100);
    delay(1000);
}