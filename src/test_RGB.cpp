#include "Adafruit_TCS34725.h"

enum class Color 
{
  Green,
  Red,
  Any
};

Adafruit_TCS34725 RGB_sensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

// retourne la couleur capte par le capteur `(rouge, vert, autre)`
Color get_color();

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
}

void loop() {
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
