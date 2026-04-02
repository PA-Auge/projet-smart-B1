#include <NewPing.h>

#define SONAR_MAX_DISTANCE 100

NewPing sonar_l = NewPing(9,10, SONAR_MAX_DISTANCE);
NewPing sonar_r = NewPing(11,12, SONAR_MAX_DISTANCE);

void setup() 
{
    Serial.begin(115200);
}

void loop() {
    Serial.print("left :  ");
    Serial.println(sonar_l.ping_cm());

    Serial.print("right : ");
    Serial.println(sonar_r.ping_cm());

    Serial.println("----------");
}
