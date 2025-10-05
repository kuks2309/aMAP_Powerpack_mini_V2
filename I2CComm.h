#ifndef I2C_COMM_H
#define I2C_COMM_H

#include <Arduino.h>
#include <Wire.h>

// I2C Configuration
#define I2C_SLAVE_ADDRESS 0x08

// I2C Command definitions
#define CMD_SET_MOTOR1_SPEED 0x01
#define CMD_SET_MOTOR2_SPEED 0x02
#define CMD_SET_MOTOR1_POSITION 0x03
#define CMD_SET_MOTOR2_POSITION 0x04
#define CMD_SET_MOTOR1_IDLE 0x05
#define CMD_SET_MOTOR2_IDLE 0x06
#define CMD_RESET_ENCODERS 0x07
#define CMD_GET_STATUS 0x10
#define CMD_GET_MOTOR1_STATUS 0x11
#define CMD_GET_MOTOR2_STATUS 0x12

class I2CComm
{
  public:
    I2CComm();
    void begin(uint8_t address = I2C_SLAVE_ADDRESS);

    // Callback setters
    void setMotor1SpeedCallback(void (*callback)(int16_t));
    void setMotor2SpeedCallback(void (*callback)(int16_t));
    void setMotor1PositionCallback(void (*callback)(int32_t));
    void setMotor2PositionCallback(void (*callback)(int32_t));
    void setMotor1IdleCallback(void (*callback)());
    void setMotor2IdleCallback(void (*callback)());
    void setResetEncodersCallback(void (*callback)());

    // Status setters (called by main program)
    void setMotor1Status(int32_t position, int16_t speed, uint8_t mode);
    void setMotor2Status(int32_t position, int16_t speed, uint8_t mode);

  private:
    // Static handlers for Wire library
    static void onReceiveHandler(int numBytes);
    static void onRequestHandler();

    // Instance handlers
    void onReceive(int numBytes);
    void onRequest();

    // Callbacks
    void (*_motor1SpeedCallback)(int16_t);
    void (*_motor2SpeedCallback)(int16_t);
    void (*_motor1PositionCallback)(int32_t);
    void (*_motor2PositionCallback)(int32_t);
    void (*_motor1IdleCallback)();
    void (*_motor2IdleCallback)();
    void (*_resetEncodersCallback)();

    // Status data
    struct MotorStatus {
        int32_t position;
        int16_t speed;
        uint8_t mode;
    };

    MotorStatus _motor1Status;
    MotorStatus _motor2Status;
    uint8_t _lastCommand;

    // Singleton instance for static callbacks
    static I2CComm* _instance;
};

#endif // I2C_COMM_H
