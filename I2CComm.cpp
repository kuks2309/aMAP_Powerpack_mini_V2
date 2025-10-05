#include "I2CComm.h"

// Initialize static instance pointer
I2CComm* I2CComm::_instance = nullptr;

I2CComm::I2CComm()
{
    _motor1SpeedCallback = nullptr;
    _motor2SpeedCallback = nullptr;
    _motor1PositionCallback = nullptr;
    _motor2PositionCallback = nullptr;
    _motor1IdleCallback = nullptr;
    _motor2IdleCallback = nullptr;
    _resetEncodersCallback = nullptr;

    _motor1Status = {0, 0, 0};
    _motor2Status = {0, 0, 0};
    _lastCommand = 0;

    _instance = this;
}

void I2CComm::begin(uint8_t address)
{
    Wire.begin(address);
    Wire.onReceive(onReceiveHandler);
    Wire.onRequest(onRequestHandler);
}

// Callback setters
void I2CComm::setMotor1SpeedCallback(void (*callback)(int16_t))
{
    _motor1SpeedCallback = callback;
}

void I2CComm::setMotor2SpeedCallback(void (*callback)(int16_t))
{
    _motor2SpeedCallback = callback;
}

void I2CComm::setMotor1PositionCallback(void (*callback)(int32_t))
{
    _motor1PositionCallback = callback;
}

void I2CComm::setMotor2PositionCallback(void (*callback)(int32_t))
{
    _motor2PositionCallback = callback;
}

void I2CComm::setMotor1IdleCallback(void (*callback)())
{
    _motor1IdleCallback = callback;
}

void I2CComm::setMotor2IdleCallback(void (*callback)())
{
    _motor2IdleCallback = callback;
}

void I2CComm::setResetEncodersCallback(void (*callback)())
{
    _resetEncodersCallback = callback;
}

// Status setters
void I2CComm::setMotor1Status(int32_t position, int16_t speed, uint8_t mode)
{
    _motor1Status.position = position;
    _motor1Status.speed = speed;
    _motor1Status.mode = mode;
}

void I2CComm::setMotor2Status(int32_t position, int16_t speed, uint8_t mode)
{
    _motor2Status.position = position;
    _motor2Status.speed = speed;
    _motor2Status.mode = mode;
}

// Static handlers
void I2CComm::onReceiveHandler(int numBytes)
{
    if (_instance != nullptr)
    {
        _instance->onReceive(numBytes);
    }
}

void I2CComm::onRequestHandler()
{
    if (_instance != nullptr)
    {
        _instance->onRequest();
    }
}

// Instance receive handler
void I2CComm::onReceive(int numBytes)
{
    if (numBytes < 1)
        return;

    uint8_t command = Wire.read();
    _lastCommand = command;

    switch (command)
    {
    case CMD_SET_MOTOR1_SPEED:
        if (numBytes >= 3)
        {
            int16_t speed = (Wire.read() << 8) | Wire.read();
            if (_motor1SpeedCallback != nullptr)
            {
                _motor1SpeedCallback(speed);
            }
        }
        break;

    case CMD_SET_MOTOR2_SPEED:
        if (numBytes >= 3)
        {
            int16_t speed = (Wire.read() << 8) | Wire.read();
            if (_motor2SpeedCallback != nullptr)
            {
                _motor2SpeedCallback(speed);
            }
        }
        break;

    case CMD_SET_MOTOR1_POSITION:
        if (numBytes >= 5)
        {
            int32_t position = ((int32_t)Wire.read() << 24) | ((int32_t)Wire.read() << 16) | ((int32_t)Wire.read() << 8) |
                               Wire.read();
            if (_motor1PositionCallback != nullptr)
            {
                _motor1PositionCallback(position);
            }
        }
        break;

    case CMD_SET_MOTOR2_POSITION:
        if (numBytes >= 5)
        {
            int32_t position = ((int32_t)Wire.read() << 24) | ((int32_t)Wire.read() << 16) | ((int32_t)Wire.read() << 8) |
                               Wire.read();
            if (_motor2PositionCallback != nullptr)
            {
                _motor2PositionCallback(position);
            }
        }
        break;

    case CMD_SET_MOTOR1_IDLE:
        if (_motor1IdleCallback != nullptr)
        {
            _motor1IdleCallback();
        }
        break;

    case CMD_SET_MOTOR2_IDLE:
        if (_motor2IdleCallback != nullptr)
        {
            _motor2IdleCallback();
        }
        break;

    case CMD_RESET_ENCODERS:
        if (_resetEncodersCallback != nullptr)
        {
            _resetEncodersCallback();
        }
        break;

    case CMD_GET_STATUS:
    case CMD_GET_MOTOR1_STATUS:
    case CMD_GET_MOTOR2_STATUS:
        // Command stored in _lastCommand, will be handled in onRequest
        break;

    default:
        // Unknown command, flush remaining bytes
        while (Wire.available())
        {
            Wire.read();
        }
        break;
    }
}

// Instance request handler
void I2CComm::onRequest()
{
    switch (_lastCommand)
    {
    case CMD_GET_STATUS:
    {
        // Send both motor statuses (14 bytes total)
        // Motor 1: position(4) + speed(2) + mode(1) = 7 bytes
        // Motor 2: position(4) + speed(2) + mode(1) = 7 bytes
        uint8_t data[14];

        // Motor 1
        data[0] = (_motor1Status.position >> 24) & 0xFF;
        data[1] = (_motor1Status.position >> 16) & 0xFF;
        data[2] = (_motor1Status.position >> 8) & 0xFF;
        data[3] = _motor1Status.position & 0xFF;
        data[4] = (_motor1Status.speed >> 8) & 0xFF;
        data[5] = _motor1Status.speed & 0xFF;
        data[6] = _motor1Status.mode;

        // Motor 2
        data[7] = (_motor2Status.position >> 24) & 0xFF;
        data[8] = (_motor2Status.position >> 16) & 0xFF;
        data[9] = (_motor2Status.position >> 8) & 0xFF;
        data[10] = _motor2Status.position & 0xFF;
        data[11] = (_motor2Status.speed >> 8) & 0xFF;
        data[12] = _motor2Status.speed & 0xFF;
        data[13] = _motor2Status.mode;

        Wire.write(data, 14);
        break;
    }

    case CMD_GET_MOTOR1_STATUS:
    {
        // Send motor 1 status (7 bytes)
        uint8_t data[7];
        data[0] = (_motor1Status.position >> 24) & 0xFF;
        data[1] = (_motor1Status.position >> 16) & 0xFF;
        data[2] = (_motor1Status.position >> 8) & 0xFF;
        data[3] = _motor1Status.position & 0xFF;
        data[4] = (_motor1Status.speed >> 8) & 0xFF;
        data[5] = _motor1Status.speed & 0xFF;
        data[6] = _motor1Status.mode;

        Wire.write(data, 7);
        break;
    }

    case CMD_GET_MOTOR2_STATUS:
    {
        // Send motor 2 status (7 bytes)
        uint8_t data[7];
        data[0] = (_motor2Status.position >> 24) & 0xFF;
        data[1] = (_motor2Status.position >> 16) & 0xFF;
        data[2] = (_motor2Status.position >> 8) & 0xFF;
        data[3] = _motor2Status.position & 0xFF;
        data[4] = (_motor2Status.speed >> 8) & 0xFF;
        data[5] = _motor2Status.speed & 0xFF;
        data[6] = _motor2Status.mode;

        Wire.write(data, 7);
        break;
    }

    default:
        // Send error byte
        Wire.write(0xFF);
        break;
    }
}
