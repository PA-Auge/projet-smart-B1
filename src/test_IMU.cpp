#include <ICM20948_WE.h>

#define ICM20948_ADDR 0x68
ICM20948_WE imu = ICM20948_WE(ICM20948_ADDR);

long prev_t = 0;
float dt = 1;

float current_speed = 0;
float stop_distance = 0;

float angle_speed = 0;
float angle_offset = 0;

void update_imu_data();

void setup() 
{
    Wire.begin();
    Serial.begin(115200);

    if (!imu.init()) {
        Serial.println("L'IMU ne repond pas !");
    }
    else {
        Serial.println("IMU connecté");
    }
}

void loop()
{
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