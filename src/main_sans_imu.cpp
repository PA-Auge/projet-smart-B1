#include "Adafruit_TCS34725.h"
#include <NewPing.h>

// ─── Classes ────────────────────────────────────────────────────────────────

class Motor {
public:
    int PIN_1, PIN_2, PIN_SPEED;
    Motor(int p1, int p2, int ps) : PIN_1(p1), PIN_2(p2), PIN_SPEED(ps) {}

    void init() {
        pinMode(PIN_1, OUTPUT);
        pinMode(PIN_2, OUTPUT);
        pinMode(PIN_SPEED, OUTPUT);
    }

    void update_speed(int speed) {
        speed = constrain(speed, -255, 255);
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

// ─── Enum ────────────────────────────────────────────────────────────────────

enum class Color {
    Green,
    Red,
    Any
};

// ─── Prototypes ──────────────────────────────────────────────────────────────

Color get_color();
void move_forward();
void move_backward();
void stop();

// ─── Objets matériels ────────────────────────────────────────────────────────

Adafruit_TCS34725 RGB_sensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

#define SONAR_MAX_DISTANCE 100
NewPing sonar_l = NewPing(9, 10, SONAR_MAX_DISTANCE);
NewPing sonar_r = NewPing(12, 11, SONAR_MAX_DISTANCE);

Motor motor_l = Motor(2, 4, 3);
Motor motor_r = Motor(7, 8, 5);

// ─── Constantes chenal ───────────────────────────────────────────────────────

const int CHENAL_WIDTH    = 48;  // cm
const int BOAT_WIDTH      = 20;  // cm
const int SIDE_CLEARANCE  = (CHENAL_WIDTH - BOAT_WIDTH) / 2;  // 14 cm par côté
const int TARGET_DISTANCE = SIDE_CLEARANCE;                   // 14 cm = distance idéale sonar
const int ALLOWED_OFFSET  = 2;   // tolérance de ±2 cm autour du centre

// ─── Constantes moteurs ──────────────────────────────────────────────────────

const int SPEED            = 255;
const int CORRECTION_SPEED = 150;
const int REVERSE_DURATION = 10;   // ~1 seconde (10 × 100ms)
const float PREDICTION_FACTOR = 1.5;

// ─── Variables globales ──────────────────────────────────────────────────────

bool is_moving       = true;
bool last_direction  = true;  // true = avant | false = arrière

bool reverse_decided = false;
int  reverse_counter = 0;

int offset           = 0;
int prev_offset      = 0;

Color color;
Color prev_color = Color::Any;

// ─── Setup ───────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(9600);
    pinMode(LED_BUILTIN, OUTPUT);

    if (!RGB_sensor.begin()) {
        digitalWrite(LED_BUILTIN, HIGH);
    }

    motor_l.init();
    motor_r.init();

    Serial.print("Chenal: "); Serial.print(CHENAL_WIDTH);   Serial.println(" cm");
    Serial.print("Bateau: "); Serial.print(BOAT_WIDTH);     Serial.println(" cm");
    Serial.print("Espace par cote: "); Serial.print(SIDE_CLEARANCE); Serial.println(" cm");
    Serial.print("Distance cible sonar: "); Serial.print(TARGET_DISTANCE); Serial.println(" cm");
}

// ─── Loop ────────────────────────────────────────────────────────────────────

void loop() {
    int d_l = sonar_l.ping_cm();
    int d_r = sonar_r.ping_cm();

    if (d_l > 0 && d_r > 0) {
        // offset par rapport au centre : positif = trop à droite, négatif = trop à gauche
        int raw_offset = d_l - d_r;
        int drift      = raw_offset - prev_offset;
        offset         = raw_offset + (int)(drift * PREDICTION_FACTOR);
        prev_offset    = raw_offset;

        Serial.print("L: "); Serial.print(d_l);
        Serial.print(" cm | R: "); Serial.print(d_r);
        Serial.print(" cm | offset predit: "); Serial.println(offset);
    }

    color = get_color();

    switch (color) {
        case Color::Red:   is_moving = false; break;
        case Color::Green: is_moving = true;  break;
        default: break;
    }

    // ── Logique de déplacement ────────────────────────────────────────────

    if (is_moving) {
        move_forward();
        last_direction  = true;
        reverse_decided = false;
        reverse_counter = 0;
    } else {
        if (color == Color::Red) {
            stop();
            prev_color = color;
            return;
        }

        if (color == Color::Any && prev_color == Color::Red) {
            if (!reverse_decided) {
                reverse_decided = true;
                reverse_counter = REVERSE_DURATION;
            }
        }

        if (reverse_decided && reverse_counter > 0) {
            if (last_direction) {
                move_backward();
            } else {
                move_forward();
            }
            reverse_counter--;

            if (reverse_counter == 0) {
                last_direction  = !last_direction;
                reverse_decided = false;
                is_moving       = true;
            }
        }
    }

    prev_color = color;
    delay(100);
}

// ─── Fonctions ───────────────────────────────────────────────────────────────

Color get_color() {
    uint16_t r, g, b, c;
    RGB_sensor.getRawData(&r, &g, &b, &c);

    if (c == 0) return Color::Any;

    if (r > g * 10 / 3 && r > b * 10 / 3) return Color::Red;
    if (g > r * 8  / 3 && g > b * 8  / 5) return Color::Green;

    return Color::Any;
}

void move_forward() {
    if (offset > ALLOWED_OFFSET) {
        // Trop près de la paroi gauche → correction droite
        motor_l.update_speed(-CORRECTION_SPEED);
        motor_r.update_speed(SPEED);
    } else if (offset < -ALLOWED_OFFSET) {
        // Trop près de la paroi droite → correction gauche
        motor_l.update_speed(SPEED);
        motor_r.update_speed(-CORRECTION_SPEED);
    } else {
        motor_l.update_speed(SPEED);
        motor_r.update_speed(SPEED);
    }
}

void move_backward() {
    if (offset > ALLOWED_OFFSET) {
        motor_l.update_speed(CORRECTION_SPEED);
        motor_r.update_speed(-SPEED);
    } else if (offset < -ALLOWED_OFFSET) {
        motor_l.update_speed(-SPEED);
        motor_r.update_speed(CORRECTION_SPEED);
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