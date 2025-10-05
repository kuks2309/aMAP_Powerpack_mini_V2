

#define MOTOR1_EN1 2
#define MOTOR1_EN2 3

#define MOTOR2_EN1 4
#define MOTOR2_EN2 5

#define SERVO1_PIN 6
#define SERVO2_PIN 7

#define SERVO1_NEUTRAL 87
#define SERVO2_NEUTRAL 92

#include <Servo.h>

Servo servo1;
Servo servo2;

// DRV8711 motor driver-1
void motor1_speed_control(int speed)
{
    if (speed > 0)
    {
        analogWrite(MOTOR1_EN1, speed);
        analogWrite(MOTOR1_EN2, 0);
    }
    else if (speed < 0)
    {
        analogWrite(MOTOR1_EN1, 0);
        analogWrite(MOTOR1_EN2, abs(speed));
    }
    else
    {
        analogWrite(MOTOR1_EN1, 0);
        analogWrite(MOTOR1_EN2, 0);
    }
}

// DRV8711 motor driver-2
void motor2_speed_control(int speed)
{
    if (speed > 0)
    {
        analogWrite(MOTOR2_EN1, speed);
        analogWrite(MOTOR2_EN2, 0);
    }
    else if (speed < 0)
    {
        analogWrite(MOTOR2_EN1, 0);
        analogWrite(MOTOR2_EN2, abs(speed));
    }
    else
    {
        analogWrite(MOTOR2_EN1, 0);
        analogWrite(MOTOR2_EN2, 0);
    }
}

void servo1_angle_control(int angle)
{
    angle = constrain(angle, 0, 180);
    servo1.write(angle);
}

void servo2_angle_control(int angle)
{
    angle = constrain(angle, 0, 180);
    servo2.write(angle);
}

void setup()
{
    Serial.begin(9600);

    pinMode(MOTOR1_EN1, OUTPUT);
    pinMode(MOTOR1_EN2, OUTPUT);

    pinMode(MOTOR2_EN1, OUTPUT);
    pinMode(MOTOR2_EN2, OUTPUT);

    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);

    servo1.write(90);
    servo2.write(90);

    Serial.println("aMAP Powerpack mini V2 initialized");
}

void loop()
{
    // Motor test: forward -> stop -> backward -> stop
    motor1_speed_control(150);
    motor2_speed_control(150);
    delay(2000);

    motor1_speed_control(0);
    motor2_speed_control(0);
    delay(1000);

    motor1_speed_control(-150);
    motor2_speed_control(-150);
    delay(2000);

    motor1_speed_control(0);
    motor2_speed_control(0);
    delay(1000);

    // Servo test: 0 -> 90 -> 180 -> 90
    servo1_angle_control(0);
    servo2_angle_control(0);
    delay(1000);

    servo1_angle_control(90);
    servo2_angle_control(90);
    delay(1000);

    servo1_angle_control(180);
    servo2_angle_control(180);
    delay(1000);

    servo1_angle_control(90);
    servo2_angle_control(90);
    delay(1000);
}