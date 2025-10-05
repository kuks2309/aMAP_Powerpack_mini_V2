

#define MOTOR1_EN1 2
#define MOTOR1_EN2 3

#define MOTOR2_EN1 4
#define MOTOR2_EN2 5

#define SERVO1_PIN 6
#define SERVO2_PIN 7

#define SERVO1_NEUTRAL 87
#define SERVO2_NEUTRAL 92

#define LS7166_CS1 8
#define LS7166_CS2 9

// LS7166 Register addresses
#define MDR0 0x08
#define MDR1 0x10
#define DTR  0x18
#define CNTR 0x20
#define OTR  0x28
#define STR  0x30
#define CMD  0x38

// LS7166 Commands
#define CLR_CNTR 0x20
#define RLD_CNTR 0x40
#define RLD_OTR  0x50
#define RESET_BP 0x01
#define RESET_E  0x02
#define LATCH_CNTR 0x03
#define SET_DTR_TO_CNTR 0x04
#define MCR_MDR0 0x88
#define MCR_MDR1 0x90

#include <Servo.h>
#include <SPI.h>

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

// LS7166 Encoder

void write_LS7166(uint8_t cs_pin, uint8_t reg, uint8_t data)
{
    digitalWrite(cs_pin, LOW);
    SPI.transfer(reg | 0x80);
    SPI.transfer(data);
    digitalWrite(cs_pin, HIGH);
}

uint8_t read_LS7166(uint8_t cs_pin, uint8_t reg)
{
    uint8_t data;
    digitalWrite(cs_pin, LOW);
    SPI.transfer(reg);
    data = SPI.transfer(0x00);
    digitalWrite(cs_pin, HIGH);
    return data;
}

void init_encoder(uint8_t cs_pin)
{
    // Configure MDR0: 4X quadrature, free running, index disabled
    write_LS7166(cs_pin, MCR_MDR0, 0x00);
    write_LS7166(cs_pin, MDR0, 0x03);

    // Configure MDR1: 32-bit counter, enable counting
    write_LS7166(cs_pin, MCR_MDR1, 0x00);
    write_LS7166(cs_pin, MDR1, 0x00);

    // Clear counter
    write_LS7166(cs_pin, CMD, CLR_CNTR);
}

int32_t read_encoder1(void)
{
    int32_t count = 0;

    // Latch counter to output register
    write_LS7166(LS7166_CS1, CMD, LATCH_CNTR);

    // Read 32-bit counter value
    digitalWrite(LS7166_CS1, LOW);
    SPI.transfer(OTR);
    count = SPI.transfer(0x00);
    count |= ((int32_t)SPI.transfer(0x00) << 8);
    count |= ((int32_t)SPI.transfer(0x00) << 16);
    count |= ((int32_t)SPI.transfer(0x00) << 24);
    digitalWrite(LS7166_CS1, HIGH);

    return count;
}

int32_t read_encoder2(void)
{
    int32_t count = 0;

    // Latch counter to output register
    write_LS7166(LS7166_CS2, CMD, LATCH_CNTR);

    // Read 32-bit counter value
    digitalWrite(LS7166_CS2, LOW);
    SPI.transfer(OTR);
    count = SPI.transfer(0x00);
    count |= ((int32_t)SPI.transfer(0x00) << 8);
    count |= ((int32_t)SPI.transfer(0x00) << 16);
    count |= ((int32_t)SPI.transfer(0x00) << 24);
    digitalWrite(LS7166_CS2, HIGH);

    return count;
}

void reset_encoder1(void)
{
    write_LS7166(LS7166_CS1, CMD, CLR_CNTR);
}

void reset_encoder2(void)
{
    write_LS7166(LS7166_CS2, CMD, CLR_CNTR);
}

void servo1_angle_control(int angle)
{
    angle = constrain(angle, -35, 35);
    servo1.write(SERVO1_NEUTRAL + angle);
}

void servo2_angle_control(int angle)
{
    angle = constrain(angle, -35, 35);
    servo2.write(SERVO2_NEUTRAL + angle);
}

void setup()
{
    Serial.begin(9600);

    // Initialize SPI for LS7166
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV8);
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);

    // Configure CS pins
    pinMode(LS7166_CS1, OUTPUT);
    pinMode(LS7166_CS2, OUTPUT);
    digitalWrite(LS7166_CS1, HIGH);
    digitalWrite(LS7166_CS2, HIGH);

    // Initialize encoders
    init_encoder(LS7166_CS1);
    init_encoder(LS7166_CS2);

    pinMode(MOTOR1_EN1, OUTPUT);
    pinMode(MOTOR1_EN2, OUTPUT);

    pinMode(MOTOR2_EN1, OUTPUT);
    pinMode(MOTOR2_EN2, OUTPUT);

    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);

    servo1.write(SERVO1_NEUTRAL);
    servo2.write(SERVO2_NEUTRAL);

    Serial.println("aMAP Powerpack mini V2 initialized");
}

void loop()
{
}