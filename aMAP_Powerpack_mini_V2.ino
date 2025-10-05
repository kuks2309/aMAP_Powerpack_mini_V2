#include "LS7166.h"
#include <Arduino_FreeRTOS.h>
#include <Servo.h>
#include <semphr.h>

// Pin definitions
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

// Control modes
enum ControlMode
{
    MODE_IDLE = 0,
    MODE_SPEED_CONTROL,
    MODE_POSITION_CONTROL
};

// Motor control structure
typedef struct
{
    int target_speed;
    int current_speed;
    int32_t target_position;
    int32_t current_position;
    ControlMode mode;
} MotorControl;

// Global objects
Servo servo1;
Servo servo2;
LS7166 encoder(LS7166_CS1, LS7166_CS2);

// Control structures
MotorControl motor1_ctrl = {0, 0, 0, 0, MODE_IDLE};
MotorControl motor2_ctrl = {0, 0, 0, 0, MODE_IDLE};

// Semaphores for data protection
SemaphoreHandle_t xMotor1Semaphore;
SemaphoreHandle_t xMotor2Semaphore;
SemaphoreHandle_t xSerialSemaphore;

// PID constants
const float KP_SPEED = 0.5;
const float KI_SPEED = 0.1;
const float KD_SPEED = 0.05;

const float KP_POSITION = 1.0;
const float KI_POSITION = 0.01;
const float KD_POSITION = 0.1;

// PID variables for motor 1
float motor1_speed_error_sum = 0;
float motor1_speed_last_error = 0;
float motor1_pos_error_sum = 0;
float motor1_pos_last_error = 0;

// PID variables for motor 2
float motor2_speed_error_sum = 0;
float motor2_speed_last_error = 0;
float motor2_pos_error_sum = 0;
float motor2_pos_last_error = 0;

// Function prototypes
void TaskMotorSpeedControl(void *pvParameters);
void TaskMotorPositionControl(void *pvParameters);
void TaskSerialCommunication(void *pvParameters);

void motor1_speed_control(int speed);
void motor2_speed_control(int speed);
int calculate_speed_pid(float target, float current, float *error_sum, float *last_error);
int calculate_position_pid(int32_t target, int32_t current, float *error_sum, float *last_error);

// Motor control functions
void motor1_speed_control(int speed)
{
    speed = constrain(speed, -255, 255);
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

void motor2_speed_control(int speed)
{
    speed = constrain(speed, -255, 255);
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

// PID calculation for speed control
int calculate_speed_pid(float target, float current, float *error_sum, float *last_error)
{
    float error = target - current;
    *error_sum += error;
    *error_sum = constrain(*error_sum, -1000, 1000); // Anti-windup

    float p_term = KP_SPEED * error;
    float i_term = KI_SPEED * (*error_sum);
    float d_term = KD_SPEED * (error - *last_error);

    *last_error = error;

    return constrain((int)(p_term + i_term + d_term), -255, 255);
}

// PID calculation for position control
int calculate_position_pid(int32_t target, int32_t current, float *error_sum, float *last_error)
{
    float error = target - current;
    *error_sum += error;
    *error_sum = constrain(*error_sum, -10000, 10000); // Anti-windup

    float p_term = KP_POSITION * error;
    float i_term = KI_POSITION * (*error_sum);
    float d_term = KD_POSITION * (error - *last_error);

    *last_error = error;

    return constrain((int)(p_term + i_term + d_term), -255, 255);
}

// Task 1: Motor Speed Control (10Hz)
void TaskMotorSpeedControl(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10Hz

    int32_t last_enc1 = 0;
    int32_t last_enc2 = 0;

    for (;;)
    {
        // Read encoder values
        int32_t current_enc1 = encoder.read_encoder1();
        int32_t current_enc2 = encoder.read_encoder2();

        // Calculate speed (encoder counts per 100ms)
        int32_t speed1 = current_enc1 - last_enc1;
        int32_t speed2 = current_enc2 - last_enc2;

        last_enc1 = current_enc1;
        last_enc2 = current_enc2;

        // Motor 1 speed control
        if (xSemaphoreTake(xMotor1Semaphore, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            motor1_ctrl.current_speed = speed1;
            motor1_ctrl.current_position = current_enc1;

            if (motor1_ctrl.mode == MODE_SPEED_CONTROL)
            {
                int output = calculate_speed_pid(motor1_ctrl.target_speed, speed1, &motor1_speed_error_sum,
                                                 &motor1_speed_last_error);
                motor1_speed_control(output);
            }
            xSemaphoreGive(xMotor1Semaphore);
        }

        // Motor 2 speed control
        if (xSemaphoreTake(xMotor2Semaphore, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            motor2_ctrl.current_speed = speed2;
            motor2_ctrl.current_position = current_enc2;

            if (motor2_ctrl.mode == MODE_SPEED_CONTROL)
            {
                int output = calculate_speed_pid(motor2_ctrl.target_speed, speed2, &motor2_speed_error_sum,
                                                 &motor2_speed_last_error);
                motor2_speed_control(output);
            }
            xSemaphoreGive(xMotor2Semaphore);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Task 2: Motor Position Control (20Hz)
void TaskMotorPositionControl(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20Hz

    for (;;)
    {
        // Motor 1 position control
        if (xSemaphoreTake(xMotor1Semaphore, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            if (motor1_ctrl.mode == MODE_POSITION_CONTROL)
            {
                int output = calculate_position_pid(motor1_ctrl.target_position, motor1_ctrl.current_position,
                                                    &motor1_pos_error_sum, &motor1_pos_last_error);
                motor1_speed_control(output);
            }
            else if (motor1_ctrl.mode == MODE_IDLE)
            {
                motor1_speed_control(0);
            }
            xSemaphoreGive(xMotor1Semaphore);
        }

        // Motor 2 position control
        if (xSemaphoreTake(xMotor2Semaphore, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            if (motor2_ctrl.mode == MODE_POSITION_CONTROL)
            {
                int output = calculate_position_pid(motor2_ctrl.target_position, motor2_ctrl.current_position,
                                                    &motor2_pos_error_sum, &motor2_pos_last_error);
                motor2_speed_control(output);
            }
            else if (motor2_ctrl.mode == MODE_IDLE)
            {
                motor2_speed_control(0);
            }
            xSemaphoreGive(xMotor2Semaphore);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Task 3: Serial Communication (check every 50ms)
void TaskSerialCommunication(void *pvParameters)
{
    (void)pvParameters;
    String inputString = "";

    for (;;)
    {
        if (xSemaphoreTake(xSerialSemaphore, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            while (Serial.available())
            {
                char inChar = (char)Serial.read();
                if (inChar == '\n')
                {
                    // Parse command
                    // Format: "M1S100" - Motor 1 Speed 100
                    // Format: "M2P1000" - Motor 2 Position 1000
                    // Format: "M1I" - Motor 1 Idle
                    // Format: "RESET" - Reset encoders
                    // Format: "STATUS" - Get status

                    if (inputString.startsWith("M1S"))
                    {
                        // Motor 1 Speed control
                        int speed = inputString.substring(3).toInt();
                        if (xSemaphoreTake(xMotor1Semaphore, pdMS_TO_TICKS(10)) == pdTRUE)
                        {
                            motor1_ctrl.mode = MODE_SPEED_CONTROL;
                            motor1_ctrl.target_speed = speed;
                            motor1_speed_error_sum = 0;
                            motor1_speed_last_error = 0;
                            xSemaphoreGive(xMotor1Semaphore);
                        }
                        Serial.println("M1 Speed: " + String(speed));
                    }
                    else if (inputString.startsWith("M2S"))
                    {
                        // Motor 2 Speed control
                        int speed = inputString.substring(3).toInt();
                        if (xSemaphoreTake(xMotor2Semaphore, pdMS_TO_TICKS(10)) == pdTRUE)
                        {
                            motor2_ctrl.mode = MODE_SPEED_CONTROL;
                            motor2_ctrl.target_speed = speed;
                            motor2_speed_error_sum = 0;
                            motor2_speed_last_error = 0;
                            xSemaphoreGive(xMotor2Semaphore);
                        }
                        Serial.println("M2 Speed: " + String(speed));
                    }
                    else if (inputString.startsWith("M1P"))
                    {
                        // Motor 1 Position control
                        int32_t position = inputString.substring(3).toInt();
                        if (xSemaphoreTake(xMotor1Semaphore, pdMS_TO_TICKS(10)) == pdTRUE)
                        {
                            motor1_ctrl.mode = MODE_POSITION_CONTROL;
                            motor1_ctrl.target_position = position;
                            motor1_pos_error_sum = 0;
                            motor1_pos_last_error = 0;
                            xSemaphoreGive(xMotor1Semaphore);
                        }
                        Serial.println("M1 Position: " + String(position));
                    }
                    else if (inputString.startsWith("M2P"))
                    {
                        // Motor 2 Position control
                        int32_t position = inputString.substring(3).toInt();
                        if (xSemaphoreTake(xMotor2Semaphore, pdMS_TO_TICKS(10)) == pdTRUE)
                        {
                            motor2_ctrl.mode = MODE_POSITION_CONTROL;
                            motor2_ctrl.target_position = position;
                            motor2_pos_error_sum = 0;
                            motor2_pos_last_error = 0;
                            xSemaphoreGive(xMotor2Semaphore);
                        }
                        Serial.println("M2 Position: " + String(position));
                    }
                    else if (inputString == "M1I")
                    {
                        // Motor 1 Idle
                        if (xSemaphoreTake(xMotor1Semaphore, pdMS_TO_TICKS(10)) == pdTRUE)
                        {
                            motor1_ctrl.mode = MODE_IDLE;
                            xSemaphoreGive(xMotor1Semaphore);
                        }
                        Serial.println("M1 Idle");
                    }
                    else if (inputString == "M2I")
                    {
                        // Motor 2 Idle
                        if (xSemaphoreTake(xMotor2Semaphore, pdMS_TO_TICKS(10)) == pdTRUE)
                        {
                            motor2_ctrl.mode = MODE_IDLE;
                            xSemaphoreGive(xMotor2Semaphore);
                        }
                        Serial.println("M2 Idle");
                    }
                    else if (inputString == "RESET")
                    {
                        encoder.reset_encoders();
                        Serial.println("Encoders reset");
                    }
                    else if (inputString == "STATUS")
                    {
                        String status = "M1: ";
                        if (xSemaphoreTake(xMotor1Semaphore, pdMS_TO_TICKS(10)) == pdTRUE)
                        {
                            status += "Pos=" + String(motor1_ctrl.current_position);
                            status += " Speed=" + String(motor1_ctrl.current_speed);
                            status += " Mode=" + String(motor1_ctrl.mode);
                            xSemaphoreGive(xMotor1Semaphore);
                        }
                        status += " | M2: ";
                        if (xSemaphoreTake(xMotor2Semaphore, pdMS_TO_TICKS(10)) == pdTRUE)
                        {
                            status += "Pos=" + String(motor2_ctrl.current_position);
                            status += " Speed=" + String(motor2_ctrl.current_speed);
                            status += " Mode=" + String(motor2_ctrl.mode);
                            xSemaphoreGive(xMotor2Semaphore);
                        }
                        Serial.println(status);
                    }

                    inputString = "";
                }
                else
                {
                    inputString += inChar;
                }
            }
            xSemaphoreGive(xSerialSemaphore);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        ;
    }
    // Initialize hardware
    pinMode(MOTOR1_EN1, OUTPUT);
    pinMode(MOTOR1_EN2, OUTPUT);
    pinMode(MOTOR2_EN1, OUTPUT);
    pinMode(MOTOR2_EN2, OUTPUT);

    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo1.write(SERVO1_NEUTRAL);
    servo2.write(SERVO2_NEUTRAL);

    // Initialize encoder
    encoder.begin();

    // Create semaphores
    xMotor1Semaphore = xSemaphoreCreateMutex();
    xMotor2Semaphore = xSemaphoreCreateMutex();
    xSerialSemaphore = xSemaphoreCreateMutex();

    // Create tasks
    xTaskCreate(TaskMotorSpeedControl, "SpeedCtrl",
                128, // Stack size
                NULL,
                3, // Priority (highest)
                NULL);

    xTaskCreate(TaskMotorPositionControl, "PosCtrl",
                128, // Stack size
                NULL,
                2, // Priority
                NULL);

    xTaskCreate(TaskSerialCommunication, "SerialComm",
                256, // Stack size (larger for string handling)
                NULL,
                1, // Priority (lowest)
                NULL);

    Serial.println("RTOS Motor Control System Started");
    Serial.println("Commands:");
    Serial.println("  M1S[speed] - Motor1 speed control (-255 to 255)");
    Serial.println("  M2S[speed] - Motor2 speed control (-255 to 255)");
    Serial.println("  M1P[position] - Motor1 position control");
    Serial.println("  M2P[position] - Motor2 position control");
    Serial.println("  M1I - Motor1 idle");
    Serial.println("  M2I - Motor2 idle");
    Serial.println("  RESET - Reset encoders");
    Serial.println("  STATUS - Get current status");
}

void loop()
{
    // Empty. Things are done in Tasks.
}